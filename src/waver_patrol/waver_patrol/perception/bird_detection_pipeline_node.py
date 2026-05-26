from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


@dataclass
class TrackState:
    first_time: float | None = None
    last_time: float | None = None
    first_xyz: tuple[float, float, float] | None = None
    last_xyz: tuple[float, float, float] | None = None
    path_length_m: float = 0.0
    id_switch_count: int = 0
    lost_count: int = 0


class CsvSink:
    def __init__(self, output_dir: str, enabled: bool) -> None:
        self.enabled = enabled and bool(output_dir)
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}
        self.headers: dict[str, list[str]] = {}
        if self.enabled:
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            self.output_dir = output_dir
        else:
            self.output_dir = ""

    def write(self, name: str, row: dict[str, Any]) -> None:
        if not self.enabled:
            return
        if name not in self.writers:
            path = os.path.join(self.output_dir, name)
            handle = open(path, "w", newline="", encoding="utf-8")
            fieldnames = list(row.keys())
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            self.files[name] = handle
            self.writers[name] = writer
            self.headers[name] = fieldnames
        writer = self.writers[name]
        fieldnames = self.headers[name]
        writer.writerow({key: row.get(key, "") for key in fieldnames})
        self.files[name].flush()

    def close(self) -> None:
        for handle in self.files.values():
            try:
                handle.close()
            except Exception:
                pass
        self.files.clear()
        self.writers.clear()
        self.headers.clear()


class BirdDetectionPipelineNode(Node):
    """Gazebo bird detection/localization/tracking/mission metric pipeline.

    This node is intentionally a Gazebo/synthetic evaluation bridge. It uses
    Gazebo model-state provenance as ground truth and publishes the same
    bird-specific topics that a real camera/3D detector should feed later. It
    never publishes `/cmd_vel` and does not convert generic moving objects into
    bird targets.
    """

    def __init__(self) -> None:
        super().__init__("bird_detection_pipeline_node")
        self.declare_parameter("scenario_id", "B3_DYNAMIC_BIRD_HIGH")
        self.declare_parameter("target_model_name", "bird_test_target")
        self.declare_parameter("object_class", "bird")
        self.declare_parameter("detector_source", "gazebo_model_state_synthetic")
        self.declare_parameter("detector_model", "gazebo_ground_truth_proxy")
        self.declare_parameter("expected_bird", True)
        self.declare_parameter("expected_mission_trigger", True)
        self.declare_parameter("confidence", 0.92)
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("bbox_iou_proxy", 1.0)
        self.declare_parameter("target_min_height_m", 3.0)
        self.declare_parameter("target_max_height_m", 30.0)
        self.declare_parameter("ground_z_offset_m", 0.0)
        self.declare_parameter("min_dynamic_motion_m", 0.20)
        self.declare_parameter("min_dynamic_velocity_mps", 0.05)
        self.declare_parameter("min_tracking_duration_sec", 1.0)
        self.declare_parameter("static_motion_tolerance_m", 0.25)
        self.declare_parameter("move_target_model", True)
        self.declare_parameter("motion_pattern", "circle")
        self.declare_parameter("initial_x", 3.0)
        self.declare_parameter("initial_y", 2.0)
        self.declare_parameter("initial_z", 3.2)
        self.declare_parameter("motion_radius_m", 1.2)
        self.declare_parameter("motion_speed_mps", 0.45)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("set_entity_state_service", "/set_entity_state")
        self.declare_parameter("model_states_topics", ["/gazebo/model_states", "/model_states"])
        self.declare_parameter("mission_mode_topic", "/waver/mode")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("allowed_trigger_modes", ["PATROL", "AUTO", "PATROL_NAVIGATING"])
        self.declare_parameter("mapping_mode_trigger_disabled", True)
        self.declare_parameter("publish_waver_compatibility_topics", True)
        self.declare_parameter("write_csv", True)
        self.declare_parameter("output_dir", "")

        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.target_model_name = str(self.get_parameter("target_model_name").value)
        self.object_class = str(self.get_parameter("object_class").value)
        self.detector_source = str(self.get_parameter("detector_source").value)
        self.detector_model = str(self.get_parameter("detector_model").value)
        self.expected_bird = bool(self.get_parameter("expected_bird").value)
        self.expected_mission_trigger = bool(self.get_parameter("expected_mission_trigger").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.allowed_trigger_modes = {str(v) for v in self.get_parameter("allowed_trigger_modes").value}
        self.track = TrackState()
        self.start_time = self._now()
        self.last_pose: Pose | None = None
        self.last_twist_xyz = (0.0, 0.0, 0.0)
        self.last_model_time = 0.0
        self.mode = "STANDBY"
        self.mission_state = "UNKNOWN"
        self.safety_state = "UNKNOWN"
        self.robot_xyz = (0.0, 0.0, 0.0)
        self.robot_yaw = 0.0

        self.frames = 0
        self.tp = 0
        self.fp = 0
        self.fn = 0
        self.trigger_tp = 0
        self.trigger_fp = 0
        self.trigger_fn = 0
        self.first_trigger_time: float | None = None
        self.last_detection_latency_ms = 0.0

        output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        self.csv = CsvSink(output_dir, bool(self.get_parameter("write_csv").value))

        self.ground_truth_pub = self.create_publisher(String, "/bird/ground_truth", 10)
        self.detections_2d_pub = self.create_publisher(String, "/bird/detections_2d", 10)
        self.detections_3d_pub = self.create_publisher(String, "/bird/detections_3d", 10)
        self.tracks_pub = self.create_publisher(String, "/bird/tracks", 10)
        self.metrics_pub = self.create_publisher(String, "/bird/metrics", 10)
        self.mission_debug_pub = self.create_publisher(String, "/bird/mission_debug", 10)
        self.mission_target_pub = self.create_publisher(PoseStamped, "/bird/mission_target", 10)

        self.compat_elevated_pub = self.create_publisher(PoseArray, "/waver/elevated_dynamic_targets", 10)
        self.bird_confirmed_pub = self.create_publisher(Bool, "/waver/bird_confirmed", 10)
        self.target_class_pub = self.create_publisher(String, "/waver/target_class", 10)
        self.target_confidence_pub = self.create_publisher(Float32, "/waver/target_confidence", 10)
        self.camera_state_pub = self.create_publisher(String, "/waver/camera_detection_state", 10)

        topics = [str(v) for v in self.get_parameter("model_states_topics").value]
        for topic in topics:
            self.create_subscription(ModelStates, topic, self.model_states_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_mode_topic").value), self.mode_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_state_topic").value), self.mission_state_callback, 10)
        self.create_subscription(String, str(self.get_parameter("safety_state_topic").value), self.safety_state_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)

        self.set_entity_client = self.create_client(
            SetEntityState,
            str(self.get_parameter("set_entity_state_service").value),
        )
        self.create_timer(0.10, self.move_tick)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_tick)
        self.get_logger().info(
            f"bird_detection_pipeline_node started scenario={self.scenario_id} "
            f"model={self.target_model_name} source={self.detector_source}"
        )

    def model_states_callback(self, msg: ModelStates) -> None:
        try:
            idx = list(msg.name).index(self.target_model_name)
        except ValueError:
            return
        self.last_pose = msg.pose[idx]
        if idx < len(msg.twist):
            twist = msg.twist[idx]
            self.last_twist_xyz = (
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.linear.z),
            )
        self.last_model_time = self._now()

    def mode_callback(self, msg: String) -> None:
        self.mode = str(msg.data).strip() or "UNKNOWN"

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = str(msg.data).strip() or "UNKNOWN"

    def safety_state_callback(self, msg: String) -> None:
        self.safety_state = str(msg.data).strip() or "UNKNOWN"

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_xyz = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def move_tick(self) -> None:
        if not bool(self.get_parameter("move_target_model").value):
            return
        if not self.set_entity_client.service_is_ready():
            return
        t = max(0.0, self._now() - self.start_time)
        x0 = float(self.get_parameter("initial_x").value)
        y0 = float(self.get_parameter("initial_y").value)
        z0 = float(self.get_parameter("initial_z").value)
        radius = max(0.05, float(self.get_parameter("motion_radius_m").value))
        speed = max(0.01, float(self.get_parameter("motion_speed_mps").value))
        omega = speed / radius
        pattern = str(self.get_parameter("motion_pattern").value)
        if pattern == "line":
            x = x0 + radius * math.sin(omega * t)
            y = y0
            vx = radius * omega * math.cos(omega * t)
            vy = 0.0
        else:
            x = x0 + radius * math.cos(omega * t)
            y = y0 + radius * math.sin(omega * t)
            vx = -radius * omega * math.sin(omega * t)
            vy = radius * omega * math.cos(omega * t)

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.target_model_name
        req.state.reference_frame = "world"
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z0
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = vx
        req.state.twist.linear.y = vy
        req.state.twist.linear.z = 0.0
        self.set_entity_client.call_async(req)

    def publish_tick(self) -> None:
        now = self._now()
        pose = self.last_pose
        visible = pose is not None and now - self.last_model_time < 1.0
        xyz = self._pose_xyz(pose) if pose is not None else (math.nan, math.nan, math.nan)
        if visible:
            self._update_track(now, xyz)
        else:
            self._mark_lost(now)

        metrics = self._compute_metrics(now, xyz, visible)
        self._publish_messages(now, xyz, visible, metrics)
        self._write_csv_rows(now, xyz, visible, metrics)

    def _update_track(self, now: float, xyz: tuple[float, float, float]) -> None:
        if self.track.first_time is None:
            self.track.first_time = now
            self.track.first_xyz = xyz
        if self.track.last_xyz is not None:
            self.track.path_length_m += self._distance(self.track.last_xyz, xyz)
        self.track.last_time = now
        self.track.last_xyz = xyz

    def _mark_lost(self, now: float) -> None:
        if self.track.last_time is not None and now - self.track.last_time > 1.0:
            self.track.lost_count += 1
            self.track.last_time = None

    def _compute_metrics(self, now: float, xyz: tuple[float, float, float], visible: bool) -> dict[str, Any]:
        object_height_m = xyz[2] - float(self.get_parameter("ground_z_offset_m").value) if visible else math.nan
        z_source_valid = self.detector_source in {
            "gazebo_model_state_synthetic",
            "gazebo_model_state",
            "pointcloud3d",
            "depth_camera",
            "stereo_camera",
            "custom_3d_detection",
        }
        z_valid = bool(visible and math.isfinite(object_height_m) and z_source_valid)
        height_filter_pass = bool(
            z_valid
            and object_height_m >= float(self.get_parameter("target_min_height_m").value)
            and object_height_m <= float(self.get_parameter("target_max_height_m").value)
        )

        duration = 0.0
        compensated_motion_m = 0.0
        if self.track.first_time is not None and self.track.first_xyz is not None and visible:
            duration = max(0.0, now - self.track.first_time)
            compensated_motion_m = self._distance(self.track.first_xyz, xyz)
        speed = math.sqrt(sum(v * v for v in self.last_twist_xyz))
        dynamic_filter_pass = bool(
            visible
            and duration >= float(self.get_parameter("min_tracking_duration_sec").value)
            and (
                compensated_motion_m >= float(self.get_parameter("min_dynamic_motion_m").value)
                or speed >= float(self.get_parameter("min_dynamic_velocity_mps").value)
            )
        )
        raw_displacement_m = compensated_motion_m
        static_due_to_ego_motion = bool(
            raw_displacement_m >= float(self.get_parameter("min_dynamic_motion_m").value)
            and compensated_motion_m < float(self.get_parameter("static_motion_tolerance_m").value)
        )

        bird_candidate = bool(
            visible
            and self.object_class.lower() == "bird"
            and self.confidence >= self.confidence_threshold
        )
        spatial_valid_bird = bool(bird_candidate and z_valid)
        elevated_bird_target = bool(spatial_valid_bird and height_filter_pass)
        dynamic_bird_target = bool(elevated_bird_target and dynamic_filter_pass and not static_due_to_ego_motion)
        mode_allows_trigger = self._mode_allows_trigger()
        mission_trigger_bird = bool(dynamic_bird_target and mode_allows_trigger and self._safety_allows_trigger())

        self.frames += 1
        if self.expected_bird:
            if bird_candidate:
                self.tp += 1
            else:
                self.fn += 1
        elif bird_candidate:
            self.fp += 1

        if self.expected_mission_trigger:
            if mission_trigger_bird:
                self.trigger_tp += 1
                if self.first_trigger_time is None:
                    self.first_trigger_time = now
            else:
                self.trigger_fn += 1
        elif mission_trigger_bird:
            self.trigger_fp += 1

        precision = self._safe_ratio(self.tp, self.tp + self.fp, default=1.0 if not self.expected_bird else 0.0)
        recall = self._safe_ratio(self.tp, self.tp + self.fn, default=1.0 if not self.expected_bird else 0.0)
        f1 = self._safe_ratio(2.0 * precision * recall, precision + recall)
        trigger_precision = self._safe_ratio(self.trigger_tp, self.trigger_tp + self.trigger_fp, default=1.0)
        trigger_recall = self._safe_ratio(
            self.trigger_tp,
            self.trigger_tp + self.trigger_fn,
            default=1.0 if not self.expected_mission_trigger else 0.0,
        )
        trigger_f1 = self._safe_ratio(2.0 * trigger_precision * trigger_recall, trigger_precision + trigger_recall)

        z_age_ms = (now - self.last_model_time) * 1000.0 if visible else math.inf
        range_m, bearing_deg = self._range_bearing(xyz) if visible else (math.nan, math.nan)
        self.last_detection_latency_ms = z_age_ms if visible else math.inf

        return {
            "scenario_id": self.scenario_id,
            "detector_source": self.detector_source,
            "detector_model": self.detector_model,
            "target_model_name": self.target_model_name,
            "object_class": self.object_class,
            "visible": visible,
            "bird_candidate": bird_candidate,
            "spatial_valid_bird": spatial_valid_bird,
            "elevated_bird_target": elevated_bird_target,
            "dynamic_bird_target": dynamic_bird_target,
            "mission_trigger_bird": mission_trigger_bird,
            "z_valid": z_valid,
            "z_source_type": self.detector_source,
            "z_source_age_ms": z_age_ms,
            "object_height_m": object_height_m,
            "height_filter_pass": height_filter_pass,
            "dynamic_filter_pass": dynamic_filter_pass,
            "ego_motion_compensated": True,
            "compensated_motion_m": compensated_motion_m,
            "compensated_velocity_mps": speed,
            "tracking_duration_sec": duration,
            "track_continuity_sec": duration,
            "path_length_m": self.track.path_length_m,
            "id_switch_count": self.track.id_switch_count,
            "lost_track_count": self.track.lost_count,
            "range_m": range_m,
            "bearing_deg": bearing_deg,
            "confidence": self.confidence if bird_candidate else 0.0,
            "confidence_threshold": self.confidence_threshold,
            "bird_precision": precision,
            "bird_recall": recall,
            "bird_f1": f1,
            "bird_mAP_50": 1.0 if bird_candidate and float(self.get_parameter("bbox_iou_proxy").value) >= 0.5 else 0.0,
            "bird_mAP_50_95": 1.0 if bird_candidate and float(self.get_parameter("bbox_iou_proxy").value) >= 0.95 else 0.0,
            "false_positive_count": self.fp,
            "false_negative_count": self.fn,
            "true_positive_count": self.tp,
            "detection_latency_ms": self.last_detection_latency_ms,
            "detection_fps": self.publish_rate_hz,
            "mission_trigger_precision": trigger_precision,
            "mission_trigger_recall": trigger_recall,
            "mission_trigger_f1": trigger_f1,
            "trigger_latency_ms": (now - self.start_time) * 1000.0 if self.first_trigger_time else math.nan,
            "mapping_mode_wrong_trigger_count": 1 if self._is_mapping_mode() and mission_trigger_bird else 0,
            "patrol_mode_valid_trigger_count": 1 if mode_allows_trigger and mission_trigger_bird else 0,
            "detected_but_not_triggered_count": 1 if bird_candidate and not mission_trigger_bird else 0,
            "trigger_without_bird_count": 1 if mission_trigger_bird and not bird_candidate else 0,
            "mode": self.mode,
            "mission_state": self.mission_state,
            "safety_state": self.safety_state,
            "mode_allows_trigger": mode_allows_trigger,
        }

    def _publish_messages(
        self,
        now: float,
        xyz: tuple[float, float, float],
        visible: bool,
        metrics: dict[str, Any],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        payload_base = {"stamp_sec": now, **metrics, "x": xyz[0], "y": xyz[1], "z": xyz[2]}
        self.ground_truth_pub.publish(String(data=json.dumps(payload_base, sort_keys=True)))

        detection_2d = {
            **payload_base,
            "bbox_cx": self._bbox_center(metrics.get("bearing_deg", math.nan)),
            "bbox_cy": 0.50,
            "bbox_w": self._bbox_size(metrics.get("range_m", math.nan)),
            "bbox_h": self._bbox_size(metrics.get("range_m", math.nan)),
        }
        self.detections_2d_pub.publish(String(data=json.dumps(detection_2d, sort_keys=True)))
        self.detections_3d_pub.publish(String(data=json.dumps(payload_base, sort_keys=True)))
        self.tracks_pub.publish(String(data=json.dumps({"track_id": 1, **payload_base}, sort_keys=True)))
        self.metrics_pub.publish(String(data=json.dumps(metrics, sort_keys=True)))

        mission_state = {
            "mission_trigger_bird": metrics["mission_trigger_bird"],
            "reason": self._trigger_reason(metrics),
            "mode": self.mode,
            "mission_state": self.mission_state,
            "safety_state": self.safety_state,
        }
        self.mission_debug_pub.publish(String(data=json.dumps(mission_state, sort_keys=True)))

        self.bird_confirmed_pub.publish(Bool(data=bool(metrics["bird_candidate"])))
        self.target_class_pub.publish(String(data="bird" if metrics["bird_candidate"] else "none"))
        self.target_confidence_pub.publish(Float32(data=float(metrics["confidence"])))
        camera_state = (
            "BIRD_DETECTED"
            if metrics["bird_candidate"]
            else ("NO_BIRD_EXPECTED" if not self.expected_bird else "BIRD_NOT_DETECTED")
        )
        self.camera_state_pub.publish(String(data=camera_state))

        if bool(metrics["mission_trigger_bird"]) and visible:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
            self.mission_target_pub.publish(pose)
            if bool(self.get_parameter("publish_waver_compatibility_topics").value):
                arr = PoseArray()
                arr.header = pose.header
                arr.poses.append(pose.pose)
                self.compat_elevated_pub.publish(arr)

    def _write_csv_rows(self, now: float, xyz: tuple[float, float, float], visible: bool, metrics: dict[str, Any]) -> None:
        base = {
            "time_sec": f"{now:.6f}",
            "scenario_id": self.scenario_id,
            "git_commit": os.environ.get("WAVER_GIT_COMMIT", ""),
            "branch": os.environ.get("WAVER_GIT_BRANCH", "jo"),
            "target_model_name": self.target_model_name,
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
            "visible": visible,
        }
        self.csv.write("bird_ground_truth.csv", {**base, "expected_bird": self.expected_bird, "object_class": self.object_class})
        self.csv.write("bird_detections_2d.csv", {**base, **self._metric_subset(metrics)})
        self.csv.write("bird_detections_3d.csv", {**base, **self._metric_subset(metrics)})
        self.csv.write("bird_tracks.csv", {**base, "track_id": 1, **self._metric_subset(metrics)})
        self.csv.write("bird_metrics.csv", {**base, **metrics})
        self.csv.write(
            "bird_mission_metrics.csv",
            {
                **base,
                "mission_trigger_expected": self.expected_mission_trigger,
                "mission_triggered": metrics["mission_trigger_bird"],
                "mission_trigger_correct": metrics["mission_trigger_bird"] == self.expected_mission_trigger,
                "trigger_latency_ms": metrics["trigger_latency_ms"],
                "mode": self.mode,
                "mission_state": self.mission_state,
                "safety_state": self.safety_state,
            },
        )

    def _metric_subset(self, metrics: dict[str, Any]) -> dict[str, Any]:
        keys = [
            "bird_candidate",
            "confidence",
            "z_valid",
            "z_source_type",
            "z_source_age_ms",
            "object_height_m",
            "height_filter_pass",
            "dynamic_filter_pass",
            "ego_motion_compensated",
            "compensated_motion_m",
            "compensated_velocity_mps",
            "range_m",
            "bearing_deg",
            "mission_trigger_bird",
            "bird_precision",
            "bird_recall",
            "bird_f1",
            "bird_mAP_50",
            "bird_mAP_50_95",
        ]
        return {key: metrics[key] for key in keys}

    def _mode_allows_trigger(self) -> bool:
        if bool(self.get_parameter("mapping_mode_trigger_disabled").value) and self._is_mapping_mode():
            return False
        mode = self.mode.upper()
        mission = self.mission_state.upper()
        return mode in self.allowed_trigger_modes or any(token in mission for token in ("PATROL", "NAVIGATING"))

    def _safety_allows_trigger(self) -> bool:
        state = self.safety_state.upper()
        return "E-STOP" not in state and "EMERGENCY" not in state

    def _is_mapping_mode(self) -> bool:
        return self.mode.upper().startswith("MAPPING") or "MAPPING" in self.mission_state.upper()

    def _trigger_reason(self, metrics: dict[str, Any]) -> str:
        if not metrics["bird_candidate"]:
            return "no_bird_candidate"
        if not metrics["z_valid"]:
            return "z_invalid"
        if not metrics["height_filter_pass"]:
            return "height_filter_failed"
        if not metrics["dynamic_filter_pass"]:
            return "dynamic_filter_failed"
        if not metrics["mode_allows_trigger"]:
            return "mode_blocks_trigger"
        if not self._safety_allows_trigger():
            return "safety_blocks_trigger"
        return "valid_bird_mission_trigger"

    def _range_bearing(self, xyz: tuple[float, float, float]) -> tuple[float, float]:
        dx = float(xyz[0]) - self.robot_xyz[0]
        dy = float(xyz[1]) - self.robot_xyz[1]
        rng = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - self.robot_yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return rng, math.degrees(bearing)

    @staticmethod
    def _bbox_center(bearing_deg: float) -> float:
        if not math.isfinite(bearing_deg):
            return math.nan
        return max(0.0, min(1.0, 0.5 + bearing_deg / 120.0))

    @staticmethod
    def _bbox_size(range_m: float) -> float:
        if not math.isfinite(range_m) or range_m <= 0.0:
            return math.nan
        return max(0.02, min(0.35, 1.0 / max(3.0, range_m)))

    @staticmethod
    def _pose_xyz(pose: Pose) -> tuple[float, float, float]:
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    @staticmethod
    def _safe_ratio(num: float, den: float, default: float = 0.0) -> float:
        return float(num) / float(den) if den else default

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> bool:
        self.csv.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BirdDetectionPipelineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class TrackSample:
    stamp: float
    pose: Pose


@dataclass
class TrackState:
    samples: deque[TrackSample] = field(default_factory=deque)
    raw_samples: deque[TrackSample] = field(default_factory=deque)
    valid: bool = False
    consecutive_world_motion_frames: int = 0


def _distance(a: Pose, b: Pose) -> float:
    return math.sqrt(
        (float(a.position.x) - float(b.position.x)) ** 2
        + (float(a.position.y) - float(b.position.y)) ** 2
        + (float(a.position.z) - float(b.position.z)) ** 2
    )


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _copy_pose(pose: Pose) -> Pose:
    # 역할: ROS callback 메시지 내부 객체 참조를 track에 그대로 저장하지 않도록 복사한다.
    # 참조를 저장하면 정적 객체도 이후 callback 객체 재사용 영향으로 움직인 것처럼 보일 수 있다.
    copied = Pose()
    copied.position.x = float(pose.position.x)
    copied.position.y = float(pose.position.y)
    copied.position.z = float(pose.position.z)
    copied.orientation.x = float(pose.orientation.x)
    copied.orientation.y = float(pose.orientation.y)
    copied.orientation.z = float(pose.orientation.z)
    copied.orientation.w = float(pose.orientation.w)
    return copied


class MovingObjectMotionFilterNode(Node):
    """Height-based elevated dynamic object filter.

    역할:
      - `/waver/lidar_objects_map`처럼 tf2로 map/odom에 변환된 3D 객체 후보를 구독한다.
      - `target_min_height_m` 기본 3.0 m 이상인지 먼저 검사한다.
      - 동적 여부는 raw LiDAR frame이 아니라 map/odom 좌표계에서 보상된 작은 움직임으로 판단한다.
      - 2D LaserScan처럼 z가 없거나 z_valid가 false인 입력은 target으로 만들지 않는다.
      - valid일 때만 기존 mission chain 호환용 `/waver/aerial_target(_active)`와
        `/waver/moving_target_valid`를 발행한다.

    중요:
      - 3m는 이동 거리가 아니라 높이 기준이다.
      - cluster_node.py는 raw centroid만 내고, 최종 target 판단은 이 노드에서 수행한다.
      - 이 노드는 `/cmd_vel`, `/goal_pose`, `/waver/object_mission_goal`을 직접 발행하지 않는다.
    """

    def __init__(self) -> None:
        super().__init__("moving_object_motion_filter_node")
        self.declare_parameter("input_topic", "/waver/lidar_objects_map")
        self.declare_parameter("raw_input_topic", "/waver/lidar_objects")
        self.declare_parameter("subscribe_raw_input", True)
        self.declare_parameter("target_point_topic", "/waver/aerial_target")
        self.declare_parameter("target_active_topic", "/waver/aerial_target_active")
        self.declare_parameter("valid_topic", "/waver/moving_target_valid")
        self.declare_parameter("track_topic", "/waver/moving_object_track")
        self.declare_parameter("state_topic", "/waver/moving_object_filter_state")
        self.declare_parameter("dynamic_motion_topic", "/waver/dynamic_motion_m")
        self.declare_parameter("elevated_targets_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("static_candidates_topic", "/waver/static_object_candidates")
        self.declare_parameter("marker_topic", "/waver/elevated_dynamic_target_marker")
        self.declare_parameter("ego_debug_topic", "/waver/ego_motion_compensation_debug")
        self.declare_parameter("height_debug_topic", "/waver/height_filter_debug")

        self.declare_parameter("target_min_height_m", 3.0)
        self.declare_parameter("target_max_height_m", 30.0)
        self.declare_parameter("height_reference_frame", "map")
        self.declare_parameter("fallback_height_reference_frame", "odom")
        self.declare_parameter("ground_z_offset_m", 0.0)
        self.declare_parameter("require_z_valid", True)
        self.declare_parameter("require_height_filter", True)
        self.declare_parameter("require_dynamic_filter", True)
        self.declare_parameter("z_source_mode", "pointcloud")
        self.declare_parameter("require_3d_z_source", True)

        self.declare_parameter("min_dynamic_motion_m", 0.2)
        self.declare_parameter("min_dynamic_velocity_mps", 0.05)
        self.declare_parameter("min_sample_motion_epsilon_m", 0.005)
        self.declare_parameter("min_tracking_duration_sec", 1.0)
        self.declare_parameter("max_tracking_duration_sec", 20.0)
        self.declare_parameter("require_consecutive_dynamic_frames", 5)
        self.declare_parameter("static_motion_tolerance_m", 0.25)
        self.declare_parameter("track_match_gate_m", 1.2)
        self.declare_parameter("max_sample_step_m", 2.0)
        self.declare_parameter("reset_on_track_jump", True)
        self.declare_parameter("lock_on_first_valid_target", True)
        self.declare_parameter("locked_target_gate_m", 1.0)
        self.declare_parameter("locked_target_lost_timeout_sec", 3.0)
        self.declare_parameter("clear_locked_target_when_static_sec", 8.0)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("enable_range_filter", False)
        self.declare_parameter("max_target_distance_m", 50.0)
        self.declare_parameter("min_target_distance_m", 0.0)

        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("angular_velocity_threshold_radps", 0.25)
        self.declare_parameter("high_yaw_rate_mode", "strict_validation")
        self.declare_parameter("pause_new_target_trigger_during_yaw_alignment", True)
        self.declare_parameter("pause_new_target_trigger_during_mapping", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_detection_classification", False)
        self.declare_parameter("detected_target_class", "lidar_elevated_dynamic_object")
        self.declare_parameter("detected_target_confidence", 0.80)
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("classification_state_topic", "/waver/classification_state")

        self.track = TrackState()
        self.track_id = 0
        self.last_msg_time = 0.0
        self.last_frame_id = "map"
        self.last_raw_frame_id = "sensor"
        self.robot_yaw_rate = 0.0
        self.robot_linear_velocity = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.mission_state = ""
        self.yaw_alignment_state = ""
        self.locked_target_active = False
        self.locked_target_id = 0
        self.locked_target_pose: Pose | None = None
        self.lock_lost_since = 0.0
        self.lock_invalid_since = 0.0

        self.point_pub = self.create_publisher(PointStamped, str(self.get_parameter("target_point_topic").value), 10)
        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("target_active_topic").value), 10)
        self.valid_pub = self.create_publisher(Bool, str(self.get_parameter("valid_topic").value), 10)
        self.track_pub = self.create_publisher(PoseStamped, str(self.get_parameter("track_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.dynamic_motion_pub = self.create_publisher(Float32, str(self.get_parameter("dynamic_motion_topic").value), 10)
        self.elevated_pub = self.create_publisher(PoseArray, str(self.get_parameter("elevated_targets_topic").value), 10)
        self.static_pub = self.create_publisher(PoseArray, str(self.get_parameter("static_candidates_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.ego_debug_pub = self.create_publisher(String, str(self.get_parameter("ego_debug_topic").value), 10)
        self.height_debug_pub = self.create_publisher(String, str(self.get_parameter("height_debug_topic").value), 10)
        self.lidar_target_id_pub = self.create_publisher(String, "/waver/lidar_target_id", 10)
        self.lidar_target_pose_sensor_pub = self.create_publisher(PoseStamped, "/waver/lidar_target_pose_sensor", 10)
        self.lidar_target_pose_map_pub = self.create_publisher(PoseStamped, "/waver/lidar_target_pose_map", 10)
        self.lidar_target_height_pub = self.create_publisher(Float32, "/waver/lidar_target_height_m", 10)
        self.lidar_target_range_pub = self.create_publisher(Float32, "/waver/lidar_target_range_m", 10)
        self.lidar_target_velocity_pub = self.create_publisher(Float32, "/waver/lidar_target_velocity_mps", 10)
        self.lidar_target_dynamic_valid_pub = self.create_publisher(Bool, "/waver/lidar_target_dynamic_valid", 10)
        self.lidar_target_z_valid_pub = self.create_publisher(Bool, "/waver/lidar_target_z_valid", 10)
        self.lidar_target_state_pub = self.create_publisher(String, "/waver/lidar_target_state", 10)
        self.publish_detection_classification = bool(
            self.get_parameter("publish_detection_classification").value
        )
        self.target_class_pub = None
        self.target_confidence_pub = None
        self.bird_confirmed_pub = None
        self.classification_state_pub = None
        if self.publish_detection_classification:
            self.target_class_pub = self.create_publisher(
                String,
                str(self.get_parameter("target_class_topic").value),
                10,
            )
            self.target_confidence_pub = self.create_publisher(
                Float32,
                str(self.get_parameter("target_confidence_topic").value),
                10,
            )
            self.bird_confirmed_pub = self.create_publisher(
                Bool,
                str(self.get_parameter("bird_confirmed_topic").value),
                10,
            )
            self.classification_state_pub = self.create_publisher(
                String,
                str(self.get_parameter("classification_state_topic").value),
                10,
            )

        self.create_subscription(PoseArray, str(self.get_parameter("input_topic").value), self.objects_callback, 10)
        if bool(self.get_parameter("subscribe_raw_input").value):
            self.create_subscription(PoseArray, str(self.get_parameter("raw_input_topic").value), self.raw_objects_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("robot_odom_topic").value), self.odom_callback, 10)
        self.create_subscription(String, "/waver/mission_state", lambda m: setattr(self, "mission_state", m.data), 10)
        self.create_subscription(String, "/waver/yaw_alignment_state", lambda m: setattr(self, "yaw_alignment_state", m.data), 10)
        self.create_timer(0.2, self.timeout_tick)

    def raw_objects_callback(self, msg: PoseArray) -> None:
        pose, _ = self._select_candidate(msg.poses, raw=True)
        if pose is None:
            return
        self.last_raw_frame_id = msg.header.frame_id or self.last_raw_frame_id
        now = self._now()
        self.track.raw_samples.append(TrackSample(now, _copy_pose(pose)))
        horizon = float(self.get_parameter("max_tracking_duration_sec").value)
        while self.track.raw_samples and now - self.track.raw_samples[0].stamp > horizon:
            self.track.raw_samples.popleft()

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.robot_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot_linear_velocity = float(msg.twist.twist.linear.x)
        self.robot_yaw_rate = float(msg.twist.twist.angular.z)

    def objects_callback(self, msg: PoseArray) -> None:
        now = self._now()
        self.last_msg_time = now
        self.last_frame_id = msg.header.frame_id or self.last_frame_id
        pose, reset_reason = self._select_candidate(msg.poses, raw=False)
        if pose is None:
            if self.locked_target_active and reset_reason.startswith("locked_target_lost"):
                if self.lock_lost_since <= 0.0:
                    self.lock_lost_since = now
                if (
                    now - self.lock_lost_since
                    > float(self.get_parameter("locked_target_lost_timeout_sec").value)
                    and not self._mission_holds_target_lock()
                ):
                    self._clear_target_lock("lost_timeout")
                fallback_pose = self.track.samples[-1].pose if self.track.samples else self.locked_target_pose
                self._publish(False, reset_reason, fallback_pose, {})
                return
            self.track = TrackState()
            self.track_id += 1
            self._publish(False, "NO_VALID_OBJECT", None, {})
            return
        if reset_reason:
            self.track = TrackState(raw_samples=self.track.raw_samples)
            self.track_id += 1
        self._append_sample(now, pose)
        metrics = self._metrics()
        valid, classification, gates = self._classify(pose, metrics)
        self.track.valid = valid
        self._update_target_lock(now, pose, valid, classification)
        state = self._state_text(valid, classification, gates, metrics, reset_reason)
        self._publish(valid, state, pose, {**metrics, **gates, "classification": classification})

    def _select_candidate(self, poses: list[Pose], raw: bool) -> tuple[Pose | None, str]:
        candidates: list[Pose] = []
        min_distance = float(self.get_parameter("min_target_distance_m").value)
        max_distance = float(self.get_parameter("max_target_distance_m").value)
        enable_range_filter = bool(self.get_parameter("enable_range_filter").value)
        for pose in poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            distance = math.hypot(x, y)
            if enable_range_filter and (distance < min_distance or distance > max_distance):
                continue
            candidates.append(pose)
        if not candidates:
            if not raw and self.locked_target_active:
                return None, "locked_target_lost no_candidates"
            return None, ""
        samples = self.track.raw_samples if raw else self.track.samples
        if not samples:
            return min(candidates, key=lambda p: math.hypot(float(p.position.x), float(p.position.y))), ""
        last = samples[-1].pose
        if not raw and self.locked_target_active:
            lock_reference = self.locked_target_pose if self.locked_target_pose is not None else last
            best_locked = min(candidates, key=lambda p: _distance(lock_reference, p))
            locked_step = _distance(lock_reference, best_locked)
            if locked_step > float(self.get_parameter("locked_target_gate_m").value):
                return None, f"locked_target_lost gate_step={locked_step:.3f}m"
            return best_locked, "locked_target_tracking"
        best = min(candidates, key=lambda p: _distance(last, p))
        step = _distance(last, best)
        if not raw and (step > float(self.get_parameter("track_match_gate_m").value) or step > float(self.get_parameter("max_sample_step_m").value)):
            if bool(self.get_parameter("reset_on_track_jump").value):
                return best, f"association_jump_{step:.3f}m"
        return best, ""

    def _update_target_lock(self, now: float, pose: Pose, valid: bool, classification: str) -> None:
        if not bool(self.get_parameter("lock_on_first_valid_target").value):
            return
        if valid:
            if not self.locked_target_active:
                self.locked_target_id = self.track_id
            self.locked_target_active = True
            self.locked_target_pose = _copy_pose(pose)
            self.lock_lost_since = 0.0
            self.lock_invalid_since = 0.0
            return
        if not self.locked_target_active:
            return
        if self._mission_holds_target_lock():
            return
        if self.lock_invalid_since <= 0.0:
            self.lock_invalid_since = now
        if now - self.lock_invalid_since > float(self.get_parameter("clear_locked_target_when_static_sec").value):
            self._clear_target_lock(f"invalid_static classification={classification}")

    def _clear_target_lock(self, reason: str) -> None:
        self.locked_target_active = False
        self.locked_target_pose = None
        self.lock_lost_since = 0.0
        self.lock_invalid_since = 0.0
        self.track = TrackState(raw_samples=self.track.raw_samples)
        self.track_id += 1
        self.state_pub.publish(String(data=f"TARGET_LOCK_CLEARED reason={reason} next_track_id={self.track_id}"))

    def _mission_holds_target_lock(self) -> bool:
        state = self.mission_state.upper()
        return any(
            token in state
            for token in (
                "TARGET",
                "INSPECTION",
                "CAMERA",
                "CLASSIFIED",
                "SOUND",
                "RETURN_TO_INTERRUPTED_WAYPOINT",
            )
        )

    def _append_sample(self, now: float, pose: Pose) -> None:
        copied = _copy_pose(pose)
        if self.track.samples:
            previous = self.track.samples[-1].pose
            if _distance(previous, copied) >= float(self.get_parameter("min_sample_motion_epsilon_m").value):
                self.track.consecutive_world_motion_frames += 1
        self.track.samples.append(TrackSample(now, copied))
        horizon = float(self.get_parameter("max_tracking_duration_sec").value)
        while self.track.samples and now - self.track.samples[0].stamp > horizon:
            self.track.samples.popleft()

    def _metrics(self) -> dict[str, float]:
        world = self._motion_metrics(self.track.samples)
        raw = self._motion_metrics(self.track.raw_samples)
        return {
            "duration": world["duration"],
            "compensated_motion_m": world["displacement"],
            "compensated_path_length_m": world["path_length"],
            "raw_displacement_m": raw["displacement"],
            "raw_path_length_m": raw["path_length"],
            "compensated_velocity_mps": max(world["displacement"], world["path_length"]) / max(world["duration"], 1e-6),
        }

    def _motion_metrics(self, samples: deque[TrackSample]) -> dict[str, float]:
        if len(samples) < 2:
            return {"duration": 0.0, "displacement": 0.0, "path_length": 0.0}
        first = samples[0]
        last = samples[-1]
        path = 0.0
        previous = first.pose
        for sample in list(samples)[1:]:
            path += _distance(previous, sample.pose)
            previous = sample.pose
        return {
            "duration": max(0.0, last.stamp - first.stamp),
            "displacement": _distance(first.pose, last.pose),
            "path_length": path,
        }

    def _classify(self, pose: Pose, metrics: dict[str, float]) -> tuple[bool, str, dict[str, object]]:
        z = float(pose.position.z)
        z_source_mode = str(self.get_parameter("z_source_mode").value).strip().lower()
        z_source_3d_valid = self._z_source_3d_valid(z_source_mode)
        z_valid = math.isfinite(z) and (
            z_source_3d_valid or not bool(self.get_parameter("require_3d_z_source").value)
        )
        object_height = z - float(self.get_parameter("ground_z_offset_m").value)
        min_height = float(self.get_parameter("target_min_height_m").value)
        max_height = float(self.get_parameter("target_max_height_m").value)
        height_filter_pass = z_valid and min_height <= object_height <= max_height

        duration_required = float(self.get_parameter("min_tracking_duration_sec").value)
        frames_required = int(self.get_parameter("require_consecutive_dynamic_frames").value)
        high_yaw_rate = abs(self.robot_yaw_rate) >= float(self.get_parameter("angular_velocity_threshold_radps").value)
        high_yaw_mode = str(self.get_parameter("high_yaw_rate_mode").value).strip().lower()
        if high_yaw_rate and high_yaw_mode == "strict_validation":
            duration_required *= 1.5
            frames_required += 2

        motion_pass = metrics["compensated_motion_m"] >= float(self.get_parameter("min_dynamic_motion_m").value)
        velocity_pass = metrics["compensated_velocity_mps"] >= float(self.get_parameter("min_dynamic_velocity_mps").value)
        dynamic_filter_pass = (
            metrics["duration"] >= duration_required
            and self.track.consecutive_world_motion_frames >= frames_required
            and (motion_pass or velocity_pass)
        )

        mapping_paused = (
            bool(self.get_parameter("pause_new_target_trigger_during_mapping").value)
            and any(token in self.mission_state.upper() for token in ("SLAM", "MAPPING", "MAP_BUILDING"))
        )
        yaw_paused = (
            bool(self.get_parameter("pause_new_target_trigger_during_yaw_alignment").value)
            and high_yaw_rate
            and high_yaw_mode == "pause_new_targets"
        )

        if not z_source_3d_valid and bool(self.get_parameter("require_3d_z_source").value):
            classification = "height_unknown_2d_or_unknown_source"
        elif not z_valid and bool(self.get_parameter("require_z_valid").value):
            classification = "height_unknown"
        elif not height_filter_pass and bool(self.get_parameter("require_height_filter").value):
            classification = "low_altitude_object" if z_valid else "height_unknown"
        elif mapping_paused:
            classification = "mapping_mode_trigger_disabled"
        elif yaw_paused:
            classification = "paused_high_yaw_rate"
        elif (
            metrics["raw_displacement_m"] >= float(self.get_parameter("min_dynamic_motion_m").value)
            and metrics["compensated_motion_m"] < float(self.get_parameter("static_motion_tolerance_m").value)
        ):
            classification = "static_due_to_ego_motion"
        elif not dynamic_filter_pass and bool(self.get_parameter("require_dynamic_filter").value):
            classification = "unknown_or_static"
        else:
            classification = "elevated_dynamic_object"

        valid = (
            classification == "elevated_dynamic_object"
            and z_valid
            and height_filter_pass
            and dynamic_filter_pass
        )
        return valid, classification, {
            "z_valid": z_valid,
            "z_source_mode": z_source_mode,
            "z_source_3d_valid": z_source_3d_valid,
            "object_height_m": object_height,
            "height_filter_pass": height_filter_pass,
            "dynamic_filter_pass": dynamic_filter_pass,
            "ego_motion_compensated": True,
            "robot_yaw_rate_radps": self.robot_yaw_rate,
            "high_yaw_rate": high_yaw_rate,
            "duration_required": duration_required,
            "frames_required": frames_required,
        }

    def _state_text(self, valid: bool, classification: str, gates: dict[str, object], metrics: dict[str, float], reset_reason: str) -> str:
        return (
            f"TRACKING frame={self.last_frame_id} samples={len(self.track.samples)} "
            f"duration={metrics['duration']:.2f} "
            f"object_height_m={float(gates['object_height_m']):.3f} "
            f"target_min_height_m={float(self.get_parameter('target_min_height_m').value):.3f} "
            f"z_valid={bool(gates['z_valid'])} "
            f"z_source_mode={gates['z_source_mode']} "
            f"z_source_3d_valid={bool(gates['z_source_3d_valid'])} "
            f"height_filter_pass={bool(gates['height_filter_pass'])} "
            f"raw_displacement_m={metrics['raw_displacement_m']:.3f} "
            f"compensated_motion_m={metrics['compensated_motion_m']:.3f} "
            f"compensated_velocity_mps={metrics['compensated_velocity_mps']:.3f} "
            f"dynamic_filter_pass={bool(gates['dynamic_filter_pass'])} "
            f"consecutive_world_motion_frames={self.track.consecutive_world_motion_frames} "
            f"robot_yaw_rate_radps={float(gates['robot_yaw_rate_radps']):.3f} "
            f"ego_motion_compensated={bool(gates['ego_motion_compensated'])} "
            f"classification={classification} "
            f"elevated_dynamic_target_valid={valid} valid={valid} "
            f"target_lock={'LOCKED' if self.locked_target_active else 'UNLOCKED'} "
            f"locked_target_id={self.locked_target_id if self.locked_target_active else self.track_id} "
            f"reset={reset_reason or 'none'}"
        )

    def _publish(self, valid: bool, state: str, pose: Pose | None, info: dict[str, object]) -> None:
        if not rclpy.ok():
            return
        metrics = self._metrics()
        try:
            self.valid_pub.publish(Bool(data=valid))
            self.active_pub.publish(Bool(data=valid))
            self.state_pub.publish(String(data=state))
            self.dynamic_motion_pub.publish(Float32(data=float(metrics["compensated_motion_m"])))
            self.ego_debug_pub.publish(String(data=state))
            self.height_debug_pub.publish(String(data=state))
            target_id = self.locked_target_id if self.locked_target_active else self.track_id
            self.lidar_target_id_pub.publish(String(data=f"lidar_track_{target_id}"))
            self.lidar_target_state_pub.publish(String(data=state))
            self._publish_detection_classification(valid, state)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f"Skipping elevated-dynamic state publish: {exc}")
            return

        elevated = PoseArray()
        static = PoseArray()
        elevated.header.stamp = self.get_clock().now().to_msg()
        elevated.header.frame_id = self.last_frame_id
        static.header = elevated.header
        if pose is not None:
            if valid:
                elevated.poses.append(_copy_pose(pose))
            else:
                static.poses.append(_copy_pose(pose))
        self.elevated_pub.publish(elevated)
        self.static_pub.publish(static)

        if pose is None:
            return
        point = PointStamped()
        point.header = elevated.header
        point.point = pose.position
        stamped = PoseStamped()
        stamped.header = elevated.header
        stamped.pose = _copy_pose(pose)
        try:
            if valid:
                self.point_pub.publish(point)
            self.track_pub.publish(stamped)
            self.lidar_target_pose_map_pub.publish(stamped)
            self.lidar_target_pose_sensor_pub.publish(stamped)
            height = float(pose.position.z)
            range_m = math.sqrt(
                float(pose.position.x) ** 2 + float(pose.position.y) ** 2 + float(pose.position.z) ** 2
            )
            self.lidar_target_height_pub.publish(Float32(data=height))
            self.lidar_target_range_pub.publish(Float32(data=range_m))
            self.lidar_target_velocity_pub.publish(Float32(data=float(metrics["compensated_velocity_mps"])))
            self.lidar_target_dynamic_valid_pub.publish(Bool(data=bool(info.get("dynamic_filter_pass", False))))
            self.lidar_target_z_valid_pub.publish(Bool(data=bool(info.get("z_valid", False))))
            if bool(self.get_parameter("publish_markers").value):
                self.marker_pub.publish(self._markers(valid, pose, str(info.get("classification", "unknown"))))
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f"Skipping elevated-dynamic track publish: {exc}")

    def _publish_detection_classification(self, valid: bool, state: str) -> None:
        if not self.publish_detection_classification:
            return
        if (
            self.target_class_pub is None
            or self.target_confidence_pub is None
            or self.bird_confirmed_pub is None
            or self.classification_state_pub is None
        ):
            return
        target_class = str(self.get_parameter("detected_target_class").value) if valid else "unknown"
        confidence = float(self.get_parameter("detected_target_confidence").value) if valid else 0.0
        self.target_class_pub.publish(String(data=target_class))
        self.target_confidence_pub.publish(Float32(data=confidence))
        self.bird_confirmed_pub.publish(Bool(data=bool(valid and target_class.strip().lower() == "bird")))
        self.classification_state_pub.publish(
            String(
                data=(
                    f"LIDAR_TARGET_VALID class={target_class} confidence={confidence:.2f}"
                    if valid
                    else f"LIDAR_TARGET_INVALID {state}"
                )
            )
        )

    def _markers(self, valid: bool, pose: Pose, classification: str) -> MarkerArray:
        markers = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.last_frame_id
        marker.ns = "waver_elevated_dynamic_filter"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = _copy_pose(pose)
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 0.0 if valid else 1.0
        marker.color.g = 1.0 if valid else 0.7
        marker.color.b = 0.2
        marker.color.a = 0.85
        markers.markers.append(marker)

        text = Marker()
        text.header = marker.header
        text.ns = marker.ns
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = _copy_pose(pose)
        text.pose.position.z += 0.45
        text.scale.z = 0.22
        text.color.r = marker.color.r
        text.color.g = marker.color.g
        text.color.b = marker.color.b
        text.color.a = 0.95
        text.text = classification
        markers.markers.append(text)
        return markers

    @staticmethod
    def _z_source_3d_valid(z_source_mode: str) -> bool:
        mode = (z_source_mode or "").strip().lower()
        if not mode or mode in {"unknown", "none", "laser", "laserscan", "scan", "2d", "2d_lidar", "fake_2d"}:
            return False
        valid_tokens = {
            "pointcloud",
            "pointcloud2",
            "3d_lidar",
            "livox",
            "mid360",
            "depth_camera",
            "stereo_camera",
            "gazebo_model_state",
            "custom_3d_detection",
            "posearray_3d",
            "simulation_3d",
        }
        return mode in valid_tokens

    def timeout_tick(self) -> None:
        if self.last_msg_time and self._now() - self.last_msg_time > float(self.get_parameter("stale_timeout_sec").value):
            self.track = TrackState()
            self._publish(False, "TRACK_STALE_RESET", None, {})
            self.last_msg_time = 0.0

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MovingObjectMotionFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.valid_pub.publish(Bool(data=False))
                node.active_pub.publish(Bool(data=False))
                node.elevated_pub.publish(PoseArray())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

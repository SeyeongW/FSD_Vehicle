from __future__ import annotations

import csv
import math
import os
import re
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from waver_patrol.mission.mission_utils import quaternion_to_yaw


SCHEMAS = {
    "mission_events.csv": [
        "time_sec", "trial_id", "mission_state", "event_name", "event_detail",
        "current_waypoint", "target_id", "success_flag",
    ],
    "lidar_clusters.csv": [
        "time_sec", "trial_id", "cluster_id", "frame_id", "x_raw", "y_raw", "z_raw",
        "x_map", "y_map", "z_map", "distance_to_robot", "cluster_size", "valid_cluster",
    ],
    "moving_object_tracks.csv": [
        "time_sec", "trial_id", "track_id", "source_frame", "target_frame",
        "x_raw", "y_raw", "z_raw", "x_map", "y_map", "z_map",
        "object_height_m", "z_valid", "height_reference_frame", "height_filter_pass",
        "raw_displacement_m", "compensated_motion_m", "compensated_velocity_mps",
        "dynamic_filter_pass", "tracking_duration_sec", "robot_yaw_rate_radps",
        "ego_motion_compensated", "classification", "elevated_dynamic_target_valid",
    ],
    "height_filter_debug.csv": [
        "time_sec", "trial_id", "track_id", "z_raw", "z_map", "object_height_m",
        "target_min_height_m", "target_max_height_m", "z_valid", "height_filter_pass",
        "failure_reason",
    ],
    "ego_motion_compensation.csv": [
        "time_sec", "trial_id", "track_id", "robot_x", "robot_y", "robot_yaw",
        "robot_linear_velocity", "robot_angular_velocity_z", "raw_displacement_m",
        "compensated_motion_m", "static_motion_tolerance_m", "high_yaw_rate",
        "classification", "dynamic_filter_pass",
    ],
    "target_map_coordinates.csv": [
        "time_sec", "trial_id", "target_id", "frame_id", "x_map", "y_map", "z_map",
        "transform_success",
    ],
    "nav_goal_results.csv": [
        "time_sec", "trial_id", "goal_x", "goal_y", "goal_yaw", "robot_x", "robot_y",
        "robot_yaw", "target_error_m", "nav_state", "nav_success",
    ],
    "yaw_alignment.csv": [
        "time_sec", "trial_id", "object_bearing_rad", "robot_yaw_rad", "yaw_error_rad",
        "cmd_angular_z", "aligned", "alignment_success",
    ],
    "camera_detections.csv": [
        "time_sec", "trial_id", "object_id", "class_name", "confidence", "bbox_x", "bbox_y",
        "bbox_w", "bbox_h", "center_u", "center_v", "tracking_state", "detection_success",
    ],
    "sound_mission.csv": [
        "time_sec", "trial_id", "sound_command", "repeat_index", "sound_started", "sound_done",
        "sound_success",
    ],
    "safety_check.csv": [
        "time_sec", "trial_id", "cmd_vel_publisher_count", "cmd_vel_publisher_names",
        "emergency_stop_active", "obstacle_stop_active", "dry_run", "enable_cmd_vel_control",
        "safety_pass",
    ],
    "ui_visualization_check.csv": [
        "time_sec", "trial_id", "map_received", "map_mode", "slam_live", "map_fixed",
        "robot_pose_visible", "robot_yaw_visible", "global_path_visible", "local_path_visible",
        "waypoint_visible", "active_goal_visible", "object_goal_visible", "elevated_target_visible",
        "mission_state_visible", "safety_state_visible", "camera_state_visible", "sound_state_visible",
        "ui_command_panel_alive", "ui_direct_cmd_vel_disabled", "overall_ui_success",
    ],
    "experiment_summary.csv": [
        "trial_id", "start_time", "end_time", "total_duration_sec",
        "target_min_height_m", "target_object_height_m", "z_valid",
        "height_filter_pass", "dynamic_filter_pass", "elevated_dynamic_target_valid",
        "height_filter_false_positive_count", "height_filter_false_negative_count",
        "static_high_object_false_positive_count", "low_dynamic_object_false_positive_count",
        "height_unknown_rejection_success", "ego_motion_compensation_success",
        "dynamic_motion_m", "target_detected", "cluster_published", "map_transform_success", "moving_target_valid",
        "target_goal_success", "yaw_alignment_success", "camera_detection_success",
        "sound_mission_success", "patrol_resume_success", "safety_gate_pass",
        "overall_success", "failure_reason",
    ],
}


@dataclass
class CsvFile:
    path: str
    fields: list[str]
    file: Any = field(init=False)
    writer: csv.DictWriter = field(init=False)

    def __post_init__(self) -> None:
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields, extrasaction="ignore")
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        full = {field: "" for field in self.fields}
        full.update(row)
        self.writer.writerow(full)

    def close(self) -> None:
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()


class GazeboTrialLoggerNode(Node):
    """Gazebo trial CSV logger with paper-oriented success summary."""

    def __init__(self) -> None:
        super().__init__("gazebo_trial_logger_node")
        self.declare_parameter("trial_id", 1)
        self.declare_parameter("output_root", "~/ros2_ws5/FSD_Vehicle/experiments_result")
        self.declare_parameter("trial_name_prefix", "gazebo_trial")
        self.declare_parameter("target_min_height_m", 3.0)
        self.declare_parameter("min_dynamic_motion_m", 0.2)
        self.declare_parameter("yaw_alignment_tolerance_rad", 0.75)
        self.declare_parameter("success_hold_sec", 1.0)
        self.declare_parameter("summary_period_sec", 1.0)

        self.trial_id = int(self.get_parameter("trial_id").value)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        root = os.path.expanduser(os.path.expandvars(str(self.get_parameter("output_root").value)))
        self.run_dir = os.path.join(root, f"gazebo_trial_{self.trial_id:02d}_{stamp}")
        for sub in ("rosbag", "rviz_screenshots", "plots"):
            os.makedirs(os.path.join(self.run_dir, sub), exist_ok=True)
        self.files = {
            name: CsvFile(os.path.join(self.run_dir, name), fields)
            for name, fields in SCHEMAS.items()
            if name != "experiment_summary.csv"
        }
        self.start_wall = time.time()
        self.start_ros = self._now()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.goal_x = math.nan
        self.goal_y = math.nan
        self.goal_yaw = math.nan
        self.target_x = math.nan
        self.target_y = math.nan
        self.target_z = math.nan
        self.dynamic_motion_m = 0.0
        self.target_object_height_m = math.nan
        self.z_valid = False
        self.height_filter_pass = False
        self.dynamic_filter_pass = False
        self.elevated_dynamic_target_valid = False
        self.classification = "unknown"
        self.raw_displacement_m = 0.0
        self.compensated_motion_m = 0.0
        self.compensated_velocity_mps = 0.0
        self.tracking_duration_sec = 0.0
        self.mission_state = "UNKNOWN"
        self.nav_state = "UNKNOWN"
        self.cmd_angular_z = 0.0
        self.camera_class = "unknown"
        self.camera_confidence = 0.0
        self.flags = {
            "target_detected": False,
            "cluster_published": False,
            "map_transform_success": False,
            "moving_target_valid": False,
            "target_goal_success": False,
            "yaw_alignment_success": False,
            "camera_detection_success": False,
            "sound_mission_success": False,
            "patrol_resume_success": False,
            "safety_gate_pass": False,
        }
        self.emergency_stop_active = False
        self.obstacle_stop_active = False
        self.failure_reason = ""
        self.success_since = 0.0

        self.create_subscription(PoseArray, "/waver/lidar_objects", self.raw_objects_callback, 10)
        self.create_subscription(PoseArray, "/waver/lidar_objects_map", self.map_objects_callback, 10)
        self.create_subscription(Bool, "/waver/moving_target_valid", self.moving_valid_callback, 10)
        self.create_subscription(String, "/waver/moving_object_filter_state", self.filter_state_callback, 10)
        self.create_subscription(String, "/waver/height_filter_debug", self.height_debug_callback, 10)
        self.create_subscription(String, "/waver/ego_motion_compensation_debug", self.ego_debug_callback, 10)
        self.create_subscription(PoseArray, "/waver/elevated_dynamic_targets", self.elevated_targets_callback, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.object_goal_callback, 10)
        self.create_subscription(PoseStamped, "/waver/active_nav_goal", self.active_goal_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/waver/sim_nav2_state", self.nav_state_callback, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(String, "/waver/camera_detection_state", self.camera_state_callback, 10)
        self.create_subscription(String, "/waver/target_class", self.target_class_callback, 10)
        self.create_subscription(Float32, "/waver/target_confidence", self.target_conf_callback, 10)
        self.create_subscription(String, "/waver/sound_alert_state", self.sound_state_callback, 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.sound_done_callback, 10)
        self.create_subscription(Bool, "/waver/emergency_stop", self.estop_callback, 10)
        self.create_subscription(String, "/waver/safety_state", self.safety_state_callback, 10)
        self.create_timer(1.0, self.safety_tick)
        self.create_timer(float(self.get_parameter("summary_period_sec").value), self.summary_tick)
        self.get_logger().info(f"Gazebo trial CSV logging to {self.run_dir}")

    def raw_objects_callback(self, msg: PoseArray) -> None:
        self.flags["cluster_published"] = bool(msg.poses)
        for idx, pose in enumerate(msg.poses[:10]):
            self.files["lidar_clusters.csv"].write({
                "time_sec": self.ts(), "trial_id": self.trial_id, "cluster_id": idx,
                "frame_id": msg.header.frame_id, "x_raw": pose.position.x,
                "y_raw": pose.position.y, "z_raw": pose.position.z,
                "distance_to_robot": math.hypot(pose.position.x - self.robot_x, pose.position.y - self.robot_y),
                "cluster_size": 1, "valid_cluster": True,
            })

    def map_objects_callback(self, msg: PoseArray) -> None:
        self.flags["map_transform_success"] = bool(msg.poses)
        for idx, pose in enumerate(msg.poses[:10]):
            self.target_x = pose.position.x
            self.target_y = pose.position.y
            self.target_z = pose.position.z
            self.files["target_map_coordinates.csv"].write({
                "time_sec": self.ts(), "trial_id": self.trial_id, "target_id": idx,
                "frame_id": msg.header.frame_id, "x_map": pose.position.x,
                "y_map": pose.position.y, "z_map": pose.position.z,
                "transform_success": True,
            })

    def moving_valid_callback(self, msg: Bool) -> None:
        if msg.data:
            self.flags["target_detected"] = True
            self.flags["moving_target_valid"] = True
            self.elevated_dynamic_target_valid = True
        self.files["moving_object_tracks.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "track_id": 0,
            "target_frame": "map",
            "x_map": self.target_x, "y_map": self.target_y, "z_map": self.target_z,
            "object_height_m": self.target_object_height_m,
            "z_valid": self.z_valid,
            "height_reference_frame": "map",
            "height_filter_pass": self.height_filter_pass,
            "raw_displacement_m": self.raw_displacement_m,
            "compensated_motion_m": self.compensated_motion_m,
            "compensated_velocity_mps": self.compensated_velocity_mps,
            "dynamic_filter_pass": self.dynamic_filter_pass,
            "tracking_duration_sec": self.tracking_duration_sec,
            "robot_yaw_rate_radps": self.cmd_angular_z,
            "ego_motion_compensated": True,
            "classification": self.classification,
            "elevated_dynamic_target_valid": bool(msg.data),
        })

    def filter_state_callback(self, msg: String) -> None:
        self._parse_filter_debug(msg.data)
        self.files["mission_events.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "mission_state": self.mission_state,
            "event_name": "MOVING_OBJECT_FILTER", "event_detail": msg.data,
            "success_flag": "valid=True" in msg.data,
        })

    def height_debug_callback(self, msg: String) -> None:
        self._parse_filter_debug(msg.data)
        self.files["height_filter_debug.csv"].write({
            "time_sec": self.ts(),
            "trial_id": self.trial_id,
            "track_id": 0,
            "z_raw": self.target_z,
            "z_map": self.target_z,
            "object_height_m": self.target_object_height_m,
            "target_min_height_m": float(self.get_parameter("target_min_height_m").value),
            "target_max_height_m": "",
            "z_valid": self.z_valid,
            "height_filter_pass": self.height_filter_pass,
            "failure_reason": "" if self.height_filter_pass else self.classification,
        })

    def ego_debug_callback(self, msg: String) -> None:
        self._parse_filter_debug(msg.data)
        self.files["ego_motion_compensation.csv"].write({
            "time_sec": self.ts(),
            "trial_id": self.trial_id,
            "track_id": 0,
            "robot_x": self.robot_x,
            "robot_y": self.robot_y,
            "robot_yaw": self.robot_yaw,
            "robot_linear_velocity": "",
            "robot_angular_velocity_z": self.cmd_angular_z,
            "raw_displacement_m": self.raw_displacement_m,
            "compensated_motion_m": self.compensated_motion_m,
            "static_motion_tolerance_m": "",
            "high_yaw_rate": "",
            "classification": self.classification,
            "dynamic_filter_pass": self.dynamic_filter_pass,
        })

    def elevated_targets_callback(self, msg: PoseArray) -> None:
        if msg.poses:
            self.flags["target_detected"] = True
            self.elevated_dynamic_target_valid = True

    def object_goal_callback(self, msg: PoseStamped) -> None:
        self.flags["target_goal_success"] = True
        self.files["nav_goal_results.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "goal_x": msg.pose.position.x,
            "goal_y": msg.pose.position.y, "goal_yaw": quaternion_to_yaw(msg.pose.orientation),
            "robot_x": self.robot_x, "robot_y": self.robot_y, "robot_yaw": self.robot_yaw,
            "target_error_m": self._goal_error(msg.pose.position.x, msg.pose.position.y),
            "nav_state": "OBJECT_GOAL_PUBLISHED", "nav_success": "",
        })

    def active_goal_callback(self, msg: PoseStamped) -> None:
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.robot_x = p.x
        self.robot_y = p.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if math.isfinite(self.target_x):
            bearing = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x)
            yaw_error = self._normalize_angle(bearing - self.robot_yaw)
            aligned = abs(yaw_error) < float(self.get_parameter("yaw_alignment_tolerance_rad").value)
            if aligned and "TARGET" in self.mission_state:
                self.flags["yaw_alignment_success"] = True
            self.files["yaw_alignment.csv"].write({
                "time_sec": self.ts(), "trial_id": self.trial_id, "object_bearing_rad": bearing,
                "robot_yaw_rad": self.robot_yaw, "yaw_error_rad": yaw_error,
                "cmd_angular_z": self.cmd_angular_z, "aligned": aligned,
                "alignment_success": self.flags["yaw_alignment_success"],
            })

    def nav_state_callback(self, msg: String) -> None:
        self.nav_state = msg.data
        if "ARRIVED" in msg.data:
            self.flags["target_goal_success"] = True
        self.files["nav_goal_results.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "goal_x": self.goal_x,
            "goal_y": self.goal_y, "goal_yaw": self.goal_yaw, "robot_x": self.robot_x,
            "robot_y": self.robot_y, "robot_yaw": self.robot_yaw,
            "target_error_m": self._goal_error(self.goal_x, self.goal_y),
            "nav_state": msg.data, "nav_success": "ARRIVED" in msg.data,
        })

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = msg.data
        if "SOUND_TASK_DONE" in msg.data or "RETURN_TO_INTERRUPTED_WAYPOINT" in msg.data:
            self.flags["sound_mission_success"] = True
        if self.flags["sound_mission_success"] and (
            "RESUME_PATROL" in msg.data or "PATROL_NAVIGATING" in msg.data
        ):
            self.flags["patrol_resume_success"] = True
        self.files["mission_events.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "mission_state": msg.data,
            "event_name": "MISSION_STATE", "event_detail": msg.data,
            "success_flag": self.flags["patrol_resume_success"],
        })

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_angular_z = msg.angular.z

    def camera_state_callback(self, msg: String) -> None:
        if "DETECTED" in msg.data or "TRACKING" in msg.data:
            self.flags["camera_detection_success"] = True
        self.files["camera_detections.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "object_id": 0,
            "class_name": self.camera_class, "confidence": self.camera_confidence,
            "tracking_state": msg.data, "detection_success": self.flags["camera_detection_success"],
        })

    def target_class_callback(self, msg: String) -> None:
        self.camera_class = msg.data

    def target_conf_callback(self, msg: Float32) -> None:
        self.camera_confidence = float(msg.data)

    def sound_state_callback(self, msg: String) -> None:
        started = "RUNNING" in msg.data or "SIMULATED" in msg.data or "REQUESTED" in msg.data
        if "SOUND_TASK_DONE" in msg.data:
            self.flags["sound_mission_success"] = True
        self.files["sound_mission.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "sound_command": msg.data,
            "repeat_index": "", "sound_started": started,
            "sound_done": "DONE" in msg.data, "sound_success": self.flags["sound_mission_success"],
        })

    def sound_done_callback(self, msg: Bool) -> None:
        if msg.data:
            self.flags["sound_mission_success"] = True
        self.files["sound_mission.csv"].write({
            "time_sec": self.ts(), "trial_id": self.trial_id, "sound_command": "sound_task_done",
            "sound_started": True, "sound_done": bool(msg.data),
            "sound_success": self.flags["sound_mission_success"],
        })

    def estop_callback(self, msg: Bool) -> None:
        self.emergency_stop_active = bool(msg.data)

    def safety_state_callback(self, msg: String) -> None:
        text = msg.data.upper()
        self.obstacle_stop_active = "HARD_STOP" in text or "OBSTACLE" in text

    def safety_tick(self) -> None:
        publishers = self.get_publishers_info_by_topic("/cmd_vel")
        names = sorted({info.node_name for info in publishers})
        safety_pass = len(names) == 1 and "safety_cmd_mux_node" in names
        self.flags["safety_gate_pass"] = safety_pass
        self.files["safety_check.csv"].write({
            "time_sec": self.ts(),
            "trial_id": self.trial_id,
            "cmd_vel_publisher_count": len(names),
            "cmd_vel_publisher_names": ";".join(names),
            "emergency_stop_active": self.emergency_stop_active,
            "obstacle_stop_active": self.obstacle_stop_active,
            "dry_run": True,
            "enable_cmd_vel_control": False,
            "safety_pass": safety_pass,
        })

    def summary_tick(self) -> None:
        overall = self._overall_success()
        if overall:
            self.success_since = self.success_since or self._now()
        else:
            self.success_since = 0.0
        self._write_summary(overall)

    def _write_summary(self, overall: bool) -> None:
        path = os.path.join(self.run_dir, "experiment_summary.csv")
        # Replace the rolling summary so scripts can read the latest status while the node is alive.
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=SCHEMAS["experiment_summary.csv"], extrasaction="ignore")
            writer.writeheader()
            writer.writerow({
                "trial_id": self.trial_id,
                "start_time": self.start_wall,
                "end_time": time.time(),
                "total_duration_sec": self._now() - self.start_ros,
                "target_min_height_m": float(self.get_parameter("target_min_height_m").value),
                "target_object_height_m": self.target_object_height_m,
                "z_valid": self.z_valid,
                "height_filter_pass": self.height_filter_pass,
                "dynamic_filter_pass": self.dynamic_filter_pass,
                "elevated_dynamic_target_valid": self.elevated_dynamic_target_valid,
                "height_filter_false_positive_count": 0,
                "height_filter_false_negative_count": 0,
                "static_high_object_false_positive_count": 0,
                "low_dynamic_object_false_positive_count": 0,
                "height_unknown_rejection_success": "",
                "ego_motion_compensation_success": True,
                "dynamic_motion_m": self.dynamic_motion_m,
                "failure_reason": self._failure_reason(),
                "overall_success": overall,
                **self.flags,
            })

    def _overall_success(self) -> bool:
        return self.elevated_dynamic_target_valid and all(self.flags.values())

    def _failure_reason(self) -> str:
        if self._overall_success():
            return ""
        missing = [key for key, value in self.flags.items() if not value]
        if not self.height_filter_pass:
            missing.append("height_filter_not_passed")
        if not self.dynamic_filter_pass:
            missing.append("dynamic_filter_not_passed")
        if not self.elevated_dynamic_target_valid:
            missing.append("elevated_dynamic_target_invalid")
        return ",".join(missing)

    def _parse_filter_debug(self, text: str) -> None:
        def number(key: str, default: float = math.nan) -> float:
            match = re.search(rf"{key}=([-+0-9.eE]+)", text)
            return float(match.group(1)) if match else default

        def boolean(key: str, default: bool = False) -> bool:
            match = re.search(rf"{key}=(True|False|true|false|1|0)", text)
            if not match:
                return default
            return match.group(1).lower() in {"true", "1"}

        def token(key: str, default: str = "") -> str:
            match = re.search(rf"{key}=([^\s]+)", text)
            return match.group(1) if match else default

        self.target_object_height_m = number("object_height_m", self.target_object_height_m)
        self.dynamic_motion_m = max(self.dynamic_motion_m, number("compensated_motion_m", 0.0))
        self.raw_displacement_m = max(self.raw_displacement_m, number("raw_displacement_m", 0.0))
        self.compensated_motion_m = max(self.compensated_motion_m, number("compensated_motion_m", 0.0))
        self.compensated_velocity_mps = max(self.compensated_velocity_mps, number("compensated_velocity_mps", 0.0))
        self.tracking_duration_sec = max(self.tracking_duration_sec, number("duration", 0.0))
        self.z_valid = self.z_valid or boolean("z_valid", False)
        self.height_filter_pass = self.height_filter_pass or boolean("height_filter_pass", False)
        self.dynamic_filter_pass = self.dynamic_filter_pass or boolean("dynamic_filter_pass", False)
        self.elevated_dynamic_target_valid = self.elevated_dynamic_target_valid or boolean("elevated_dynamic_target_valid", False)
        self.classification = token("classification", self.classification)

    def _goal_error(self, x: float, y: float) -> float:
        if not math.isfinite(x) or not math.isfinite(y):
            return math.nan
        return math.hypot(x - self.robot_x, y - self.robot_y)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def ts(self) -> float:
        return self._now() - self.start_ros

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def close(self) -> None:
        self._write_summary(self._overall_success())
        for csv_file in self.files.values():
            csv_file.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboTrialLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok() and "context is not valid" not in str(exc):
            raise
    finally:
        try:
            node.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

import csv
import math
import os
import socket
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

from waver_patrol.mission.mission_utils import quaternion_to_yaw


SCHEMAS: dict[str, list[str]] = {
    "mission_events.csv": [
        "timestamp", "experiment_id", "mission_state", "event_type", "previous_state", "next_state",
        "reason", "waypoint_index", "waypoint_name", "target_id", "nav_goal_id",
        "battery_return_active", "emergency_stop", "external_stop",
    ],
    "waypoint_progress.csv": [
        "timestamp", "experiment_id", "route_name", "waypoint_index", "waypoint_name", "waypoint_x",
        "waypoint_y", "arrival_success", "arrival_time_sec", "dwell_time_sec", "interrupted_by_target",
        "resumed_after_target", "return_to_waypoint_success", "distance_to_waypoint_m",
    ],
    "robot_pose.csv": [
        "timestamp", "experiment_id", "frame_id", "x", "y", "yaw", "z", "yaw_rad",
        "linear_velocity", "angular_velocity", "linear_velocity_x", "angular_velocity_z",
        "distance_accumulated_m", "pose_source", "covariance_trace",
    ],
    "nav2_feedback.csv": [
        "timestamp", "nav_goal_type", "goal_x", "goal_y", "goal_yaw", "feedback_distance_remaining",
        "estimated_time_remaining", "navigation_time", "number_of_recoveries", "result_status", "failure_reason",
    ],
    "cmd_vel.csv": [
        "timestamp", "experiment_id", "source", "linear_x", "angular_z", "limited_linear_x",
        "limited_angular_z", "active_mode", "safety_state", "speed_limit", "is_zero_cmd",
    ],
    "safety_state.csv": [
        "timestamp", "experiment_id", "safety_state", "mode", "cmd_vel_linear", "cmd_vel_angular",
        "scan_age_sec", "odom_age_sec", "battery_state", "estop", "external_stop", "hard_stop_reason",
        "min_scan_range_m", "min_front_range_m", "min_rear_range_m", "scan_stale",
        "scan_valid_points", "hard_stop_active", "slow_down_active", "emergency_stop",
        "require_scan", "command_timeout",
    ],
    "radar_targets.csv": [
        "timestamp", "target_id", "frame_id", "x", "y", "z", "range_m", "azimuth_rad",
        "elevation_rad", "doppler_mps", "radial_velocity_mps", "confidence", "target_age_sec",
        "accepted_as_goal", "rejection_reason",
    ],
    "lidar_targets.csv": [
        "timestamp", "experiment_id", "target_id", "frame_id", "x", "y", "z", "map_x", "map_y",
        "map_z", "height_m", "range_m", "depth_m", "bearing_rad", "velocity_mps",
        "dynamic_motion_m", "consecutive_dynamic_frames", "z_valid", "height_valid", "dynamic_valid",
        "accepted_as_aerial_target", "reject_reason", "target_lost_reason", "cluster_size",
        "motion_mps", "moving_confirmed", "rejection_reason",
    ],
    "inspection_goals.csv": [
        "timestamp", "experiment_id", "target_id", "inspection_goal_id", "target_x", "target_y",
        "target_z", "goal_x", "goal_y", "goal_yaw", "goal_offset_distance_m",
        "candidate_to_goal_latency_ms", "goal_publish_success", "goal_reject_reason",
    ],
    "target_approach.csv": [
        "timestamp", "experiment_id", "target_id", "inspection_goal_id", "approach_start_time",
        "approach_arrival_time", "approach_duration_sec", "start_distance_to_target_m",
        "final_distance_to_target_m", "arrival_success", "nav_fail_reason", "recovery_count",
        "path_length_m", "path_efficiency", "min_obstacle_distance_m",
    ],
    "camera_alignment.csv": [
        "timestamp", "experiment_id", "target_id", "frame_id", "target_x", "target_y", "target_z",
        "pan_cmd_rad", "tilt_cmd_rad", "bearing_rad", "elevation_rad", "alignment_start_time",
        "alignment_done_time", "alignment_latency_ms", "centered", "pointing_error_rad",
        "alignment_timeout", "fallback_robot_body_alignment_used", "failure_reason",
    ],
    "camera_classification.csv": [
        "timestamp", "target_id", "image_frame_id", "target_class", "confidence", "bird_confirmed",
        "experiment_id", "drone_confirmed", "unknown", "bbox_x", "bbox_y", "bbox_w", "bbox_h",
        "bbox_area_fraction", "classification_latency_ms", "inference_fps", "model_backend",
        "model_path", "camera_tracking_active", "nof_m_window", "nof_m_required",
        "detection_bbox_x", "detection_bbox_y", "detection_bbox_w", "detection_bbox_h",
    ],
    "sound_events.csv": [
        "timestamp", "experiment_id", "target_id", "target_class", "target_confidence",
        "bird_confirmed", "sound_alert_request", "sound_request_accepted", "sound_block_reason",
        "sound_type", "enable_sound_output", "sound_task_started", "sound_start_time",
        "sound_end_time", "sound_duration_sec", "sound_task_done", "cooldown_active",
        "legal_safety_note",
    ],
    "return_to_patrol.csv": [
        "timestamp", "experiment_id", "target_id", "interrupted_waypoint_index", "interrupted_waypoint_x",
        "interrupted_waypoint_y", "return_goal_x", "return_goal_y", "return_start_time",
        "return_arrival_time", "return_duration_sec", "resumed_patrol", "resume_policy", "failure_reason",
    ],
    "battery.csv": [
        "timestamp", "percentage", "voltage", "current", "temperature", "source", "warning",
        "critical", "return_home_active", "return_goal_x", "return_goal_y", "battery_stale",
    ],
    "obstacle_metrics.csv": [
        "timestamp", "min_scan_range_m", "min_front_range_m", "min_rear_range_m",
        "obstacle_count_estimate", "nav2_replanning_event", "collision_monitor_state",
        "costmap_blocked", "recovery_triggered",
    ],
    "target_mission_summary.csv": [
        "experiment_id", "target_mission_id", "start_time", "end_time", "radar_target_received_time",
        "nav_start_time", "target_arrival_time", "classification_time", "sound_start_time",
        "sound_end_time", "return_to_waypoint_start_time", "return_to_waypoint_arrival_time",
        "total_duration_sec", "navigation_success", "classification_result", "bird_confirmed",
        "sound_executed", "resumed_patrol", "failure_reason",
    ],
    "experiment_summary.csv": [
        "experiment_id", "start_time", "end_time", "total_duration_sec", "total_distance_m",
        "waypoint_count", "patrol_waypoints_completed", "waypoint_success_rate",
        "target_missions_received", "target_inspections_started", "target_inspections_completed",
        "target_missions_completed", "birds_confirmed", "drones_confirmed", "unknown_targets",
        "sound_tasks_requested", "sound_tasks_completed", "sound_success_rate",
        "return_to_patrol_success_rate", "average_target_response_time_sec",
        "average_lidar_to_goal_latency_ms", "average_approach_duration_sec",
        "average_camera_alignment_latency_ms", "average_classification_latency_ms",
        "average_sound_duration_sec", "average_return_to_patrol_time_sec",
        "safety_stop_count", "estop_count", "scan_stale_count", "battery_return_count",
        "emergency_stop_count", "external_stop_count", "nav2_failure_count", "recovery_count",
        "notes",
    ],
}


@dataclass
class CsvSink:
    path: str
    header: list[str]
    file: Any = field(init=False)
    writer: csv.DictWriter = field(init=False)
    rows: int = 0

    def __post_init__(self) -> None:
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.header, extrasaction="ignore")
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        full = {key: "" for key in self.header}
        full.update(row)
        self.writer.writerow(full)
        self.rows += 1

    def flush(self) -> None:
        self.file.flush()
        os.fsync(self.file.fileno())

    def close(self) -> None:
        self.flush()
        self.file.close()


class ExperimentDataLoggerNode(Node):
    """CSV logger for thesis/research experiments.

    rosbag2 is still the source-of-truth for raw replay. This logger writes
    human-readable CSV tables with experiment_id, timestamps, mission events,
    target ids, waypoint ids, safety states, and perception summaries.
    """

    def __init__(self) -> None:
        super().__init__("experiment_data_logger_node")
        self.declare_parameter("experiment_name", "waver_lidar_first_bird_deterrence")
        self.declare_parameter("output_root", "~/ros2_ws2/FSD_Vehicle/experiments_result")
        self.declare_parameter("csv_flush_interval_sec", 1.0)
        self.declare_parameter("write_rosbag_command_file", True)
        self.declare_parameter("log_odom", True)
        self.declare_parameter("log_nav2", True)
        self.declare_parameter("log_radar", True)
        self.declare_parameter("log_lidar", True)
        self.declare_parameter("log_camera", True)
        self.declare_parameter("log_sound", True)
        self.declare_parameter("log_battery", True)
        self.declare_parameter("log_safety", True)
        self.declare_parameter("rotate_files", True)
        self.declare_parameter("max_csv_file_size_mb", 100)
        self.declare_parameter("anonymize_location", False)
        self.declare_parameter("include_raw_topic_snapshot", False)
        self.declare_parameter("experiment_notes", "")

        self.experiment_name = str(self.get_parameter("experiment_name").value)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.experiment_id = f"{stamp}_{self.experiment_name}"
        root = os.path.expanduser(os.path.expandvars(str(self.get_parameter("output_root").value)))
        self.run_dir = os.path.join(root, self.experiment_id)
        self.rosbag_dir = os.path.join(self.run_dir, "rosbag")
        os.makedirs(self.rosbag_dir, exist_ok=True)
        self.sinks = {name: CsvSink(os.path.join(self.run_dir, name), header) for name, header in SCHEMAS.items()}
        self.start_time = self._wall_time()
        self.last_flush = self._now()
        self.last_pose_xy: tuple[float, float] | None = None
        self.total_distance_m = 0.0
        self.current_mission_state = "UNKNOWN"
        self.current_safety_state = "UNKNOWN"
        self.current_mode = "UNKNOWN"
        self.current_waypoint_index = -1
        self.current_target_class = "unknown"
        self.current_confidence = 0.0
        self.bird_confirmed = False
        self.return_home_active = False
        self.estop = False
        self.external_stop = False
        self.counters = {
            "target_missions_received": 0,
            "birds_confirmed": 0,
            "sound_tasks_requested": 0,
            "sound_tasks_completed": 0,
            "battery_return_count": 0,
            "emergency_stop_count": 0,
            "external_stop_count": 0,
            "nav2_failure_count": 0,
            "recovery_count": 0,
            "patrol_waypoints_completed": 0,
        }
        self.write_metadata()
        if bool(self.get_parameter("write_rosbag_command_file").value):
            self.write_rosbag_command()
        self.create_subscriptions()
        self.create_timer(max(float(self.get_parameter("csv_flush_interval_sec").value), 0.2), self.flush_tick)
        self.get_logger().info(f"Experiment CSV logging to {self.run_dir}")

    def create_subscriptions(self) -> None:
        if bool(self.get_parameter("log_odom").value):
            self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(String, "/waver/mission_event", self.mission_event_callback, 10)
        self.create_subscription(PoseStamped, "/waver/current_waypoint", self.current_waypoint_callback, 10)
        self.create_subscription(PoseStamped, "/waver/active_nav_goal", self.active_goal_callback, 10)
        self.create_subscription(Int32, "/waver/patrol_index", lambda m: setattr(self, "current_waypoint_index", int(m.data)), 10)
        if bool(self.get_parameter("log_radar").value):
            self.create_subscription(PoseStamped, "/waver/radar_target_goal", self.radar_goal_callback, 10)
            self.create_subscription(Bool, "/waver/radar_target_active", self.radar_active_callback, 10)
            self.create_subscription(Float32, "/waver/radar_target_confidence", self.radar_conf_callback, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.object_goal_callback, 10)
        if bool(self.get_parameter("log_lidar").value):
            self.create_subscription(PointStamped, "/waver/aerial_target", self.aerial_target_callback, 10)
            self.create_subscription(PoseArray, "/waver/lidar_objects", self.lidar_objects_callback, 10)
            self.create_subscription(PoseArray, "/waver/lidar_objects_map", self.lidar_objects_callback, 10)
            self.create_subscription(PoseArray, "/lidar/detections", self.lidar_objects_callback, 10)
            self.create_subscription(PoseStamped, "/waver/inspection_target_pose_map", self.inspection_target_callback, 10)
            self.create_subscription(String, "/waver/inspection_target_state", self.inspection_state_callback, 10)
        if bool(self.get_parameter("log_camera").value):
            self.create_subscription(String, "/waver/target_class", self.target_class_callback, 10)
            self.create_subscription(Float32, "/waver/target_confidence", self.target_conf_callback, 10)
            self.create_subscription(Bool, "/waver/bird_confirmed", self.bird_callback, 10)
            self.create_subscription(Float32, "/waver/target_classification_latency_ms", self.classification_latency_callback, 10)
            self.create_subscription(String, "/waver/target_classification_state", self.classification_state_callback, 10)
            self.create_subscription(String, "/waver/camera_alignment_state", self.camera_alignment_state_callback, 10)
            self.create_subscription(Bool, "/waver/camera_target_centered", self.camera_centered_callback, 10)
            self.create_subscription(Float32, "/waver/camera_target_bearing", self.camera_bearing_callback, 10)
            self.create_subscription(Float32, "/waver/camera_target_elevation", self.camera_elevation_callback, 10)
        if bool(self.get_parameter("log_sound").value):
            self.create_subscription(Bool, "/waver/sound_alert_request", self.sound_request_callback, 10)
            self.create_subscription(Bool, "/waver/sound_task_done", self.sound_done_callback, 10)
            self.create_subscription(String, "/waver/sound_alert_state", self.sound_state_callback, 10)
        if bool(self.get_parameter("log_safety").value):
            self.create_subscription(String, "/waver/safety_state", self.safety_state_callback, 10)
            self.create_subscription(String, "/waver/mode", lambda m: setattr(self, "current_mode", m.data), 10)
            self.create_subscription(Twist, "/cmd_vel", lambda m: self.cmd_callback(m, "SAFETY_OUTPUT"), 10)
            self.create_subscription(Twist, "/waver/cmd_vel_nav2", lambda m: self.cmd_callback(m, "NAV2"), 10)
            self.create_subscription(Twist, "/waver/manual_cmd_vel", lambda m: self.cmd_callback(m, "MANUAL"), 10)
            self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
            self.create_subscription(Bool, "/waver/emergency_stop", self.estop_callback, 10)
            self.create_subscription(Bool, "/waver/external_stop", self.external_stop_callback, 10)
        if bool(self.get_parameter("log_battery").value):
            self.create_subscription(BatteryState, "/battery_state", self.battery_state_callback, 10)
            self.create_subscription(Float32, "/voltage", self.voltage_callback, 10)
            self.create_subscription(Bool, "/waver/return_home_active", self.return_home_callback, 10)

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.last_pose_xy is not None:
            self.total_distance_m += ((p.x - self.last_pose_xy[0]) ** 2 + (p.y - self.last_pose_xy[1]) ** 2) ** 0.5
        self.last_pose_xy = (p.x, p.y)
        cov_trace = sum(msg.pose.covariance[i] for i in (0, 7, 14, 21, 28, 35))
        self.write("robot_pose.csv", {
            "timestamp": self.ts(), "frame_id": msg.header.frame_id, "x": p.x, "y": p.y, "z": p.z,
            "yaw": yaw, "yaw_rad": yaw, "linear_velocity": msg.twist.twist.linear.x,
            "angular_velocity": msg.twist.twist.angular.z, "linear_velocity_x": msg.twist.twist.linear.x,
            "angular_velocity_z": msg.twist.twist.angular.z, "pose_source": "odom",
            "covariance_trace": cov_trace, "distance_accumulated_m": self.total_distance_m,
        })

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        cov_trace = sum(msg.pose.covariance[i] for i in (0, 7, 14, 21, 28, 35))
        self.write("robot_pose.csv", {
            "timestamp": self.ts(), "frame_id": msg.header.frame_id, "x": p.x, "y": p.y, "z": p.z,
            "yaw": quaternion_to_yaw(msg.pose.pose.orientation),
            "yaw_rad": quaternion_to_yaw(msg.pose.pose.orientation), "pose_source": "amcl_pose",
            "covariance_trace": cov_trace,
        })

    def mission_state_callback(self, msg: String) -> None:
        previous = self.current_mission_state
        self.current_mission_state = msg.data
        self.write("mission_events.csv", {
            "timestamp": self.ts(), "experiment_id": self.experiment_id, "mission_state": msg.data,
            "event_type": "STATE_SAMPLE", "previous_state": previous, "next_state": msg.data,
            "waypoint_index": self.current_waypoint_index, "battery_return_active": self.return_home_active,
            "emergency_stop": self.estop, "external_stop": self.external_stop,
        })

    def mission_event_callback(self, msg: String) -> None:
        if "NAV2_RECOVERY" in msg.data:
            self.counters["recovery_count"] += 1
        if "NAV2_GOAL_FAILED" in msg.data:
            self.counters["nav2_failure_count"] += 1
        if "INTERRUPT_PATROL_FOR_LIDAR_TARGET" in msg.data:
            self.counters["target_missions_received"] += 1
            self.write("target_approach.csv", {
                "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
                "inspection_goal_id": self.counters["target_missions_received"],
                "approach_start_time": self.ts(),
            })
        if "next=TARGET_REACHED" in msg.data or "inspection_offset_goal_reached" in msg.data:
            self.write("target_approach.csv", {
                "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
                "inspection_goal_id": self.counters["target_missions_received"],
                "approach_arrival_time": self.ts(), "arrival_success": True,
            })
        if "next=RETURN_TO_INTERRUPTED_WAYPOINT" in msg.data:
            self.write("return_to_patrol.csv", {
                "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
                "interrupted_waypoint_index": self.current_waypoint_index,
                "return_start_time": self.ts(), "resume_policy": "RETURN_TO_INTERRUPTED_WAYPOINT",
            })
        if "next=RESUME_PATROL" in msg.data:
            self.write("return_to_patrol.csv", {
                "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
                "return_arrival_time": self.ts(), "resumed_patrol": True,
                "resume_policy": "RETURN_TO_INTERRUPTED_WAYPOINT",
            })
        self.write("mission_events.csv", {
            "timestamp": self.ts(), "experiment_id": self.experiment_id, "mission_state": self.current_mission_state,
            "event_type": "MISSION_EVENT", "reason": msg.data, "waypoint_index": self.current_waypoint_index,
            "battery_return_active": self.return_home_active, "emergency_stop": self.estop,
            "external_stop": self.external_stop,
        })

    def current_waypoint_callback(self, msg: PoseStamped) -> None:
        self.write("waypoint_progress.csv", {
            "timestamp": self.ts(), "waypoint_index": self.current_waypoint_index,
            "waypoint_name": f"wp_{self.current_waypoint_index}", "waypoint_x": msg.pose.position.x,
            "waypoint_y": msg.pose.position.y, "dwell_time_sec": "",
            "dwell_active": "PATROL_DWELL" in self.current_mission_state,
            "interrupted_by_target": "TARGET" in self.current_mission_state,
            "resumed_after_target": "RESUME" in self.current_mission_state,
            "distance_to_waypoint_m": "",
        })

    def active_goal_callback(self, msg: PoseStamped) -> None:
        self.write("nav2_feedback.csv", {
            "timestamp": self.ts(), "nav_goal_type": self.infer_goal_type(),
            "goal_x": msg.pose.position.x, "goal_y": msg.pose.position.y,
            "goal_yaw": quaternion_to_yaw(msg.pose.orientation),
        })

    def radar_goal_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.counters["target_missions_received"] += 1
        self.write("radar_targets.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "frame_id": msg.header.frame_id, "x": p.x, "y": p.y, "z": p.z,
            "range_m": (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5,
            "azimuth_rad": math_atan2(p.y, p.x), "accepted_as_goal": True,
        })

    def radar_active_callback(self, msg: Bool) -> None:
        pass

    def radar_conf_callback(self, msg: Float32) -> None:
        self.write("radar_targets.csv", {"timestamp": self.ts(), "confidence": msg.data})

    def object_goal_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.write("target_mission_summary.csv", {
            "experiment_id": self.experiment_id, "target_mission_id": self.counters["target_missions_received"],
            "radar_target_received_time": self.ts(), "nav_start_time": self.ts(),
            "navigation_success": "", "failure_reason": "",
        })
        self.write("inspection_goals.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "inspection_goal_id": self.counters["target_missions_received"],
            "goal_x": p.x, "goal_y": p.y, "goal_yaw": quaternion_to_yaw(msg.pose.orientation),
            "goal_publish_success": True,
        })

    def inspection_target_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        target_id = self.counters["target_missions_received"]
        self.write("lidar_targets.csv", {
            "timestamp": self.ts(), "target_id": target_id, "frame_id": msg.header.frame_id,
            "x": p.x, "y": p.y, "z": p.z, "map_x": p.x, "map_y": p.y, "map_z": p.z,
            "height_m": p.z, "range_m": (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5,
            "bearing_rad": math_atan2(p.y, p.x), "accepted_as_aerial_target": True,
        })
        self.write("inspection_goals.csv", {
            "timestamp": self.ts(), "target_id": target_id, "inspection_goal_id": target_id,
            "target_x": p.x, "target_y": p.y, "target_z": p.z,
            "goal_publish_success": True,
        })

    def inspection_state_callback(self, msg: String) -> None:
        self.write("inspection_goals.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "goal_reject_reason": msg.data if "REJECT" in msg.data else "",
            "goal_publish_success": "ACCEPT_INSPECTION_GOAL" in msg.data,
        })

    def aerial_target_callback(self, msg: PointStamped) -> None:
        p = msg.point
        self.write("lidar_targets.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "frame_id": msg.header.frame_id, "x": p.x, "y": p.y, "z": p.z,
            "height_m": p.z, "depth_m": (p.x * p.x + p.y * p.y) ** 0.5,
            "bearing_rad": math_atan2(p.y, p.x), "accepted_as_aerial_target": True,
        })

    def lidar_objects_callback(self, msg: PoseArray) -> None:
        for index, pose in enumerate(msg.poses[:20]):
            p = pose.position
            self.write("lidar_targets.csv", {
                "timestamp": self.ts(), "target_id": index, "frame_id": msg.header.frame_id,
                "x": p.x, "y": p.y, "z": p.z, "height_m": p.z,
                "depth_m": (p.x * p.x + p.y * p.y) ** 0.5, "bearing_rad": math_atan2(p.y, p.x),
                "cluster_size": "", "moving_confirmed": "",
            })

    def target_class_callback(self, msg: String) -> None:
        self.current_target_class = msg.data
        self.write_classification()

    def target_conf_callback(self, msg: Float32) -> None:
        self.current_confidence = float(msg.data)
        self.write_classification()

    def bird_callback(self, msg: Bool) -> None:
        if bool(msg.data) and not self.bird_confirmed:
            self.counters["birds_confirmed"] += 1
        self.bird_confirmed = bool(msg.data)
        self.write_classification()

    def write_classification(self) -> None:
        self.write("camera_classification.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "target_class": self.current_target_class, "confidence": self.current_confidence,
            "bird_confirmed": self.bird_confirmed,
            "drone_confirmed": self.current_target_class == "drone",
            "unknown": self.current_target_class == "unknown",
        })

    def classification_latency_callback(self, msg: Float32) -> None:
        self.write("camera_classification.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "target_class": self.current_target_class, "confidence": self.current_confidence,
            "bird_confirmed": self.bird_confirmed,
            "classification_latency_ms": float(msg.data),
        })

    def classification_state_callback(self, msg: String) -> None:
        self.write("camera_classification.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "target_class": self.current_target_class, "confidence": self.current_confidence,
            "bird_confirmed": self.bird_confirmed, "camera_tracking_active": msg.data,
        })

    def camera_alignment_state_callback(self, msg: String) -> None:
        self.write("camera_alignment.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "centered": "centered=true" in msg.data.lower(),
            "alignment_timeout": "timeout" in msg.data.lower(),
            "fallback_robot_body_alignment_used": "fallback" in msg.data.lower(),
            "failure_reason": msg.data if "FAILED" in msg.data else "",
        })

    def camera_centered_callback(self, msg: Bool) -> None:
        self.write("camera_alignment.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "centered": bool(msg.data),
        })

    def camera_bearing_callback(self, msg: Float32) -> None:
        self.write("camera_alignment.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "bearing_rad": float(msg.data),
        })

    def camera_elevation_callback(self, msg: Float32) -> None:
        self.write("camera_alignment.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "elevation_rad": float(msg.data),
        })

    def sound_request_callback(self, msg: Bool) -> None:
        if msg.data:
            self.counters["sound_tasks_requested"] += 1
        self.write("sound_events.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "target_class": self.current_target_class, "target_confidence": self.current_confidence,
            "sound_alert_request": bool(msg.data), "bird_confirmed": self.bird_confirmed,
            "sound_request_accepted": bool(msg.data) and self.current_target_class == "bird",
            "sound_type": "SIMULATED_DETERRENT", "enable_sound_output": False,
            "legal_safety_note": "stub_only_no_weapon_no_high_power_output",
        })

    def sound_done_callback(self, msg: Bool) -> None:
        if msg.data:
            self.counters["sound_tasks_completed"] += 1
        self.write("sound_events.csv", {"timestamp": self.ts(), "sound_task_done": bool(msg.data)})

    def sound_state_callback(self, msg: String) -> None:
        self.write("sound_events.csv", {
            "timestamp": self.ts(), "target_id": self.counters["target_missions_received"],
            "target_class": self.current_target_class,
            "target_confidence": self.current_confidence,
            "sound_block_reason": msg.data if "BLOCKED" in msg.data else "",
            "sound_task_started": "RUNNING" in msg.data,
            "sound_task_done": "DONE" in msg.data,
            "legal_safety_note": msg.data,
        })

    def safety_state_callback(self, msg: String) -> None:
        self.current_safety_state = msg.data
        self.write("safety_state.csv", {
            "timestamp": self.ts(), "safety_state": msg.data, "mode": self.current_mode,
            "hard_stop_active": "HARD_STOP" in msg.data, "slow_down_active": "SLOW" in msg.data,
            "emergency_stop": self.estop, "estop": self.estop, "external_stop": self.external_stop,
            "hard_stop_reason": msg.data if "STOP" in msg.data else "",
        })

    def cmd_callback(self, msg: Twist, source: str) -> None:
        self.write("cmd_vel.csv", {
            "timestamp": self.ts(), "source": source, "linear_x": msg.linear.x,
            "angular_z": msg.angular.z, "limited_linear_x": msg.linear.x,
            "limited_angular_z": msg.angular.z, "active_mode": self.current_mode,
            "safety_state": self.current_safety_state,
            "is_zero_cmd": abs(msg.linear.x) + abs(msg.angular.z) < 1e-6,
        })

    def scan_callback(self, msg: LaserScan) -> None:
        valid = [float(r) for r in msg.ranges if math_isfinite(r) and msg.range_min <= r <= msg.range_max]
        min_range = min(valid) if valid else -1.0
        self.write("obstacle_metrics.csv", {
            "timestamp": self.ts(), "min_scan_range_m": min_range,
            "obstacle_count_estimate": len([v for v in valid if v < 1.2]),
        })

    def estop_callback(self, msg: Bool) -> None:
        if msg.data and not self.estop:
            self.counters["emergency_stop_count"] += 1
        self.estop = bool(msg.data)

    def external_stop_callback(self, msg: Bool) -> None:
        if msg.data and not self.external_stop:
            self.counters["external_stop_count"] += 1
        self.external_stop = bool(msg.data)

    def battery_state_callback(self, msg: BatteryState) -> None:
        critical = 0.0 <= msg.percentage <= 0.20
        warning = 0.0 <= msg.percentage <= 0.30
        self.write("battery.csv", {
            "timestamp": self.ts(), "percentage": msg.percentage, "voltage": msg.voltage,
            "current": msg.current, "temperature": msg.temperature, "source": "BatteryState",
            "warning": warning, "critical": critical, "return_home_active": self.return_home_active,
        })

    def voltage_callback(self, msg: Float32) -> None:
        self.write("battery.csv", {
            "timestamp": self.ts(), "voltage": msg.data, "source": "VoltageFloat32",
            "warning": msg.data < 9.8, "critical": msg.data < 9.3,
            "return_home_active": self.return_home_active,
        })

    def return_home_callback(self, msg: Bool) -> None:
        if msg.data and not self.return_home_active:
            self.counters["battery_return_count"] += 1
        self.return_home_active = bool(msg.data)

    def flush_tick(self) -> None:
        try:
            for sink in self.sinks.values():
                sink.flush()
        except OSError as exc:
            self.get_logger().error(f"CSV flush failed: {exc}")

    def write_metadata(self) -> None:
        meta = {
            "experiment_id": self.experiment_id,
            "date_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "robot_name": "Waver",
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            "package_version": "0.1.0",
            "host": socket.gethostname(),
            "sound_output_enabled": False,
            "notes": str(self.get_parameter("experiment_notes").value),
        }
        with open(os.path.join(self.run_dir, "metadata.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(meta, f, sort_keys=False)

    def write_rosbag_command(self) -> None:
        bag_path = os.path.join(self.run_dir, "rosbag", "waver_experiment_bag")
        command = f"""#!/usr/bin/env bash
set -euo pipefail
ros2 bag record \\
  /tf /tf_static /odom /amcl_pose /scan /mid360_PointCloud2 /map /plan /local_plan \\
  /cmd_vel /waver/manual_cmd_vel /waver/cmd_vel_nav2_raw /waver/cmd_vel_nav2_smooth /waver/cmd_vel_nav2 \\
  /waver/mode /waver/mission_state /waver/mission_event /waver/current_waypoint /waver/active_nav_goal \\
  /waver/lidar_objects /waver/lidar_objects_map /waver/elevated_dynamic_targets /waver/moving_target_valid \\
  /waver/lidar_target_id /waver/lidar_target_pose_sensor /waver/lidar_target_pose_map \\
  /waver/lidar_target_height_m /waver/lidar_target_range_m /waver/lidar_target_velocity_mps \\
  /waver/lidar_target_dynamic_valid /waver/lidar_target_z_valid /waver/lidar_target_state \\
  /waver/height_filter_debug /waver/ego_motion_compensation_debug \\
  /waver/inspection_target_pose_map /waver/inspection_target_state \\
  /waver/camera_aim_target_pose /waver/camera_gimbal_cmd /waver/camera_alignment_state \\
  /waver/camera_target_centered /waver/bird_detections_2d /waver/bird_confirmed \\
  /waver/target_class /waver/target_confidence /waver/target_classification_state \\
  /waver/sound_alert_request /waver/sound_alert_state /waver/sound_task_done \\
  /waver/battery_safety_state /waver/base_driver_state /waver/safety_state /battery_state /voltage \\
  -o "{bag_path}"
"""
        path = os.path.join(self.run_dir, "record_rosbag.sh")
        with open(path, "w", encoding="utf-8") as f:
            f.write(command)
        os.chmod(path, 0o755)

    def close(self) -> None:
        self.write_summary()
        for sink in self.sinks.values():
            sink.close()

    def write_summary(self) -> None:
        end = self._wall_time()
        row = {
            "experiment_id": self.experiment_id,
            "start_time": self.start_time,
            "end_time": end,
            "total_duration_sec": end - self.start_time,
            "total_distance_m": self.total_distance_m,
            "notes": str(self.get_parameter("experiment_notes").value),
        }
        row.update(self.counters)
        self.sinks["experiment_summary.csv"].write(row)

    def infer_goal_type(self) -> str:
        if "BATTERY" in self.current_mission_state:
            return "BATTERY_RETURN"
        if "TARGET" in self.current_mission_state:
            return "RADAR_TARGET"
        if "RETURN_TO_INTERRUPTED" in self.current_mission_state:
            return "RETURN_TO_INTERRUPTED_WAYPOINT"
        return "PATROL"

    def write(self, filename: str, row: dict[str, Any]) -> None:
        try:
            if "experiment_id" in self.sinks[filename].header and "experiment_id" not in row:
                row = dict(row)
                row["experiment_id"] = self.experiment_id
            self.sinks[filename].write(row)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"CSV write failed for {filename}: {exc}")

    def ts(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now(self) -> float:
        return self.ts()

    def _wall_time(self) -> float:
        return time.time()


def math_isfinite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def math_atan2(y: float, x: float) -> float:
    try:
        return math.atan2(float(y), float(x))
    except Exception:
        return 0.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExperimentDataLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

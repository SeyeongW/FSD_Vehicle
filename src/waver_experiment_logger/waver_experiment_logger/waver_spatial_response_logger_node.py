from __future__ import annotations

import csv
import json
import math
import os
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int32, String


def safe_float(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except Exception:
        return default


def stamp_sec(msg: Any) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_pose(pose: Any) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def dist_xy(ax: float, ay: float, bx: float, by: float) -> float:
    if any(math.isnan(v) for v in (ax, ay, bx, by)):
        return math.nan
    return math.hypot(ax - bx, ay - by)


def dist_3d(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    if any(math.isnan(v) for v in (ax, ay, az, bx, by, bz)):
        return math.nan
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


@dataclass
class CsvFile:
    path: Path
    fields: list[str]
    handle: Any
    writer: csv.DictWriter

    def row(self, data: dict[str, Any]) -> None:
        self.writer.writerow({field: data.get(field, "") for field in self.fields})
        self.handle.flush()


class WaverSpatialResponseLoggerNode(Node):
    """Observation-only spatial/latency logger for Gazebo bird patrol trials."""

    def __init__(self) -> None:
        super().__init__("waver_spatial_response_logger_node")
        self.declare_parameter(
            "output_root",
            str(Path.home() / "ros2_ws5/FSD_Vehicle/experiment_results/gazebo_bird_patrol"),
        )
        self.declare_parameter("trial_id", "spatial_response")
        self.declare_parameter("run_id", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("detector_mode", "lidar")
        self.declare_parameter("configured_offset_m", 2.0)
        self.declare_parameter("association_radius_m", 1.5)
        self.declare_parameter("sample_period_sec", 1.0)

        self.detector_mode = str(self.get_parameter("detector_mode").value).strip().lower()
        self.configured_offset_m = float(self.get_parameter("configured_offset_m").value)
        self.run_dir = self.make_run_dir()
        self.make_dirs()
        self.csvs: dict[str, CsvFile] = {}
        self.open_csvs()

        self.odom: Odometry | None = None
        self.mission_state = "UNKNOWN"
        self.lidar_state = "UNKNOWN"
        self.lidar_provenance = ""
        self.dynamic_lock = False
        self.cmd_vel_nonzero = False
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.sound_done = False
        self.sound_done_seen = False
        self.removal_state = "UNKNOWN"
        self.patrol_lap_count = 0
        self.latest_bird: dict[str, Any] = {}
        self.latest_birds: dict[str, dict[str, Any]] = {}
        self.latest_lidar_target: dict[str, float] = {}
        self.latest_object_goal: PoseStamped | None = None
        self.latest_object_goal_time = -1.0
        self.latest_object_goal_debug: dict[str, Any] = {}
        self.latest_active_goal: PoseStamped | None = None
        self.latest_active_goal_role = "UNKNOWN"
        self.latest_active_goal_meta: dict[str, Any] = {}
        self.latest_active_goal_meta_time = -1.0
        self.latest_sim_nav2_debug: dict[str, Any] = {}

        self.first_times: dict[str, float] = {}
        self.first_target_cmd_recorded = False
        self.target_goal_odom_start: tuple[float, float] | None = None
        self.path_prev: tuple[float, float] | None = None
        self.odom_path_length = 0.0
        self.odom_first: tuple[float, float] | None = None
        self.odom_last: tuple[float, float] | None = None
        self.nonzero_cmd_rows = 0
        self.nonzero_cmd_first: float | None = None
        self.nonzero_cmd_last: float | None = None
        self.linear_cmd_abs_max = 0.0
        self.angular_cmd_abs_max = 0.0

        self.state_pub = self.create_publisher(String, "/waver/spatial_response_logger_state", 10)

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(PoseStamped, "/bird/nearest_pose", self.bird_pose_callback, 10)
        self.create_subscription(String, "/waver/gazebo_bird_kinematics", self.bird_kinematics_callback, 10)
        self.create_subscription(String, "/waver/lidar_filter_response", self.lidar_filter_response_callback, 10)
        self.create_subscription(String, "/waver/lidar_tracking_state", self.lidar_state_callback, 10)
        self.create_subscription(PoseArray, "/waver/elevated_dynamic_targets", self.dynamic_targets_callback, 10)
        self.create_subscription(PoseStamped, "/waver/lidar_target_pose_odom", self.lidar_target_callback, 10)
        self.create_subscription(Bool, "/waver/dynamic_object_lock", self.dynamic_lock_callback, 10)
        self.create_subscription(String, "/waver/dynamic_object_lock_state", lambda m: None, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.object_goal_callback, 10)
        self.create_subscription(String, "/waver/object_mission_goal_debug", self.object_goal_debug_callback, 10)
        self.create_subscription(PoseStamped, "/waver/inspection_target_pose_map", self.inspection_target_callback, 10)
        self.create_subscription(PoseStamped, "/waver/active_nav_goal", self.active_goal_callback, 10)
        self.create_subscription(String, "/waver/active_nav_goal_meta", self.active_goal_meta_callback, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(String, "/waver/mission_event", self.mission_event_callback, 10)
        self.create_subscription(Int32, "/waver/patrol_lap_count", lambda m: setattr(self, "patrol_lap_count", int(m.data)), 10)
        self.create_subscription(String, "/waver/sim_nav2_debug", self.sim_nav2_debug_callback, 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.sound_done_callback, 10)
        self.create_subscription(String, "/waver/gazebo_bird_removal_state", self.removal_state_callback, 10)
        self.create_subscription(PointCloud2, "/mid360_PointCloud2", self.pointcloud_callback, qos_profile_sensor_data)

        self.create_timer(float(self.get_parameter("sample_period_sec").value), self.sample_timer)
        self.get_logger().info(f"spatial response logger writing to {self.run_dir}")

    def make_run_dir(self) -> Path:
        explicit = str(self.get_parameter("run_dir").value).strip()
        if explicit:
            return Path(os.path.expanduser(explicit)).resolve()
        run_id = str(self.get_parameter("run_id").value).strip()
        if not run_id:
            run_id = f"{str(self.get_parameter('trial_id').value)}_{time.strftime('%Y%m%d_%H%M%S')}"
        return (Path(os.path.expanduser(str(self.get_parameter("output_root").value))) / run_id).resolve()

    def make_dirs(self) -> None:
        (self.run_dir / "logs").mkdir(parents=True, exist_ok=True)
        (self.run_dir / "metrics").mkdir(parents=True, exist_ok=True)

    def open_csvs(self) -> None:
        specs = {
            "spatial_distance_timeseries": (
                "logs/spatial_distance_timeseries.csv",
                [
                    "time_sec",
                    "mission_state",
                    "waver_x_m",
                    "waver_y_m",
                    "waver_z_m",
                    "waver_yaw_rad",
                    "bird_name",
                    "bird_state",
                    "bird_active",
                    "bird_hidden",
                    "bird_fleeing",
                    "bird_x_m",
                    "bird_y_m",
                    "bird_z_m",
                    "bird_speed_xy_mps",
                    "bird_speed_3d_mps",
                    "lidar_target_x_m",
                    "lidar_target_y_m",
                    "lidar_target_z_m",
                    "lidar_target_valid",
                    "lidar_target_age_sec",
                    "lidar_track_id",
                    "object_goal_x_m",
                    "object_goal_y_m",
                    "active_nav_goal_x_m",
                    "active_nav_goal_y_m",
                    "active_goal_role",
                    "robot_to_bird_xy_m",
                    "robot_to_bird_3d_m",
                    "robot_to_lidar_target_xy_m",
                    "lidar_target_to_bird_xy_m",
                    "object_goal_to_bird_xy_m",
                    "active_goal_to_bird_xy_m",
                    "object_goal_to_lidar_target_xy_m",
                    "active_goal_to_lidar_target_xy_m",
                    "robot_to_active_goal_xy_m",
                    "configured_offset_m",
                    "active_goal_standoff_error_to_bird_m",
                    "active_goal_standoff_error_to_lidar_target_m",
                    "detector_mode",
                    "lidar_provenance",
                    "dynamic_lock",
                    "cmd_vel_nonzero",
                ],
            ),
            "goal_bird_distance_events": (
                "logs/goal_bird_distance_events.csv",
                [
                    "time_sec",
                    "event_type",
                    "goal_role",
                    "mission_state",
                    "is_target_related_goal",
                    "target_track_id",
                    "target_bird_name",
                    "association_method",
                    "association_distance_m",
                    "bird_name",
                    "bird_state",
                    "bird_x_m",
                    "bird_y_m",
                    "bird_z_m",
                    "bird_speed_xy_mps",
                    "bird_speed_3d_mps",
                    "goal_frame",
                    "goal_x_m",
                    "goal_y_m",
                    "goal_z_m",
                    "goal_yaw_rad",
                    "waver_x_m",
                    "waver_y_m",
                    "waver_yaw_rad",
                    "goal_to_bird_xy_m",
                    "goal_to_bird_3d_m",
                    "robot_to_goal_xy_m",
                    "robot_to_bird_xy_m",
                    "configured_offset_m",
                    "standoff_error_m",
                    "lidar_target_x_m",
                    "lidar_target_y_m",
                    "lidar_target_z_m",
                    "lidar_target_valid",
                    "lidar_target_age_sec",
                    "lidar_track_id",
                    "goal_to_lidar_target_xy_m",
                    "goal_to_lidar_target_3d_m",
                    "lidar_target_to_bird_xy_m",
                    "standoff_error_to_bird_m",
                    "standoff_error_to_lidar_target_m",
                ],
            ),
            "bird_kinematics": (
                "logs/bird_kinematics.csv",
                [
                    "time_sec",
                    "bird_name",
                    "released",
                    "active",
                    "hidden",
                    "state",
                    "removed",
                    "x_m",
                    "y_m",
                    "z_m",
                    "vx_mps",
                    "vy_mps",
                    "vz_mps",
                    "speed_xy_mps",
                    "speed_3d_mps",
                    "target_x_m",
                    "target_y_m",
                    "target_z_m",
                    "nearest",
                    "fleeing",
                    "inside_lidar_area",
                ],
            ),
            "lidar_filter_response": (
                "logs/lidar_filter_response.csv",
                [
                    "pc_stamp_sec",
                    "tracker_receive_time_sec",
                    "filter_done_time_sec",
                    "pc_age_at_receive_ms",
                    "filter_runtime_ms",
                    "filter_runtime_wall_ms",
                    "pc_stamp_to_filter_done_ms",
                    "pc_stamp_to_filter_done_sim_ms",
                    "raw_points",
                    "roi_points",
                    "cluster_points",
                    "roi_ratio",
                    "cluster_ratio",
                    "tf_ok",
                    "target_ok",
                    "target_lost",
                    "source",
                    "provenance",
                    "detector_mode",
                    "fallback_allowed",
                    "decision_allowed",
                    "moving",
                    "track_id",
                    "track_age_frames",
                    "missed_frames",
                    "estimated_target_speed_mps",
                    "lidar_target_x_m",
                    "lidar_target_y_m",
                    "lidar_target_z_m",
                    "lidar_target_valid",
                    "lidar_target_age_sec",
                ],
            ),
            "lidar_waver_latency_events": (
                "logs/lidar_waver_latency_events.csv",
                [
                    "trial_id",
                    "first_pointcloud_time_sec",
                    "first_lidar_filter_done_time_sec",
                    "first_lidar_target_ok_time_sec",
                    "first_dynamic_lock_true_time_sec",
                    "first_object_mission_goal_time_sec",
                    "first_active_nav_goal_time_sec",
                    "first_active_target_nav_goal_time_sec",
                    "first_approach_state_time_sec",
                    "first_nonzero_cmd_vel_time_sec",
                    "first_nonzero_cmd_after_target_goal_time_sec",
                    "first_odom_motion_after_target_goal_time_sec",
                    "pointcloud_to_lidar_target_ok_ms",
                    "lidar_target_ok_to_dynamic_lock_ms",
                    "dynamic_lock_to_object_mission_goal_ms",
                    "object_mission_goal_to_active_nav_goal_ms",
                    "object_mission_goal_to_active_target_nav_goal_ms",
                    "active_nav_goal_to_approach_state_ms",
                    "active_nav_goal_to_first_cmd_vel_ms",
                    "active_target_nav_goal_to_cmd_vel_ms",
                    "active_target_nav_goal_to_odom_motion_ms",
                    "lidar_target_ok_to_first_cmd_vel_ms",
                    "lidar_target_ok_to_first_target_cmd_vel_ms",
                    "pointcloud_to_object_mission_goal_ms",
                ],
            ),
            "cmd_vel_response": (
                "logs/cmd_vel_response.csv",
                [
                    "time_sec",
                    "mission_state",
                    "linear_x",
                    "angular_z",
                    "nonzero_cmd",
                    "time_since_first_lidar_target_ok_ms",
                    "time_since_active_nav_goal_ms",
                ],
            ),
        }
        for name, (rel, fields) in specs.items():
            path = self.run_dir / rel
            handle = path.open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            self.csvs[name] = CsvFile(path, fields, handle, writer)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def remember_first(self, name: str, value: float | None = None) -> None:
        if name not in self.first_times:
            self.first_times[name] = self.now_sec() if value is None else float(value)
            self.write_latency_row()

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.odom_first is None:
            self.odom_first = (x, y)
        if self.path_prev is not None:
            step = math.hypot(x - self.path_prev[0], y - self.path_prev[1])
            if math.isfinite(step) and step < 5.0:
                self.odom_path_length += step
        self.path_prev = (x, y)
        self.odom_last = (x, y)
        if (
            self.target_goal_odom_start is not None
            and "first_active_target_nav_goal_time_sec" in self.first_times
            and "first_odom_motion_after_target_goal_time_sec" not in self.first_times
        ):
            moved = math.hypot(x - self.target_goal_odom_start[0], y - self.target_goal_odom_start[1])
            if math.isfinite(moved) and moved >= 0.03:
                self.remember_first("first_odom_motion_after_target_goal_time_sec")

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        self.cmd_vel_nonzero = abs(self.cmd_linear_x) > 1e-4 or abs(self.cmd_angular_z) > 1e-4
        self.linear_cmd_abs_max = max(self.linear_cmd_abs_max, abs(self.cmd_linear_x))
        self.angular_cmd_abs_max = max(self.angular_cmd_abs_max, abs(self.cmd_angular_z))
        if self.cmd_vel_nonzero:
            now = self.now_sec()
            self.nonzero_cmd_rows += 1
            if self.nonzero_cmd_first is None:
                self.nonzero_cmd_first = now
            if (
                not self.first_target_cmd_recorded
                and "first_active_target_nav_goal_time_sec" in self.first_times
                and self.mission_state.split()[0] in {"APPROACH_TARGET_OFFSET", "TARGET_NAVIGATING"}
            ):
                self.first_target_cmd_recorded = True
                self.remember_first("first_nonzero_cmd_vel_time_sec", now)
                self.remember_first("first_nonzero_cmd_after_target_goal_time_sec", now)
            self.nonzero_cmd_last = now
        self.csvs["cmd_vel_response"].row(
            {
                "time_sec": self.now_sec(),
                "mission_state": self.mission_state,
                "linear_x": self.cmd_linear_x,
                "angular_z": self.cmd_angular_z,
                "nonzero_cmd": str(self.cmd_vel_nonzero).lower(),
                "time_since_first_lidar_target_ok_ms": self.elapsed_ms("first_lidar_target_ok_time_sec"),
                "time_since_active_nav_goal_ms": self.elapsed_ms("first_active_nav_goal_time_sec"),
            }
        )

    def bird_pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if abs(float(p.x)) < 1e-9 and abs(float(p.y)) < 1e-9 and abs(float(p.z)) < 1e-9 and self.latest_bird:
            return
        self.latest_bird.update({"name": self.latest_bird.get("name", "nearest"), "x": float(p.x), "y": float(p.y), "z": float(p.z)})

    def bird_kinematics_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        nearest_name = str(payload.get("nearest_name") or "")
        for bird in payload.get("birds", []):
            name = str(bird.get("name", ""))
            if not name:
                continue
            row = {
                "time_sec": safe_float(payload.get("time_sec"), self.now_sec()),
                "bird_name": name,
                "released": str(bool(bird.get("released"))).lower(),
                "active": str(bool(bird.get("active"))).lower(),
                "hidden": str(bool(bird.get("hidden"))).lower(),
                "state": str(bird.get("state", "")),
                "removed": str(bool(bird.get("removed"))).lower(),
                "x_m": safe_float(bird.get("x")),
                "y_m": safe_float(bird.get("y")),
                "z_m": safe_float(bird.get("z")),
                "vx_mps": safe_float(bird.get("vx"), 0.0),
                "vy_mps": safe_float(bird.get("vy"), 0.0),
                "vz_mps": safe_float(bird.get("vz"), 0.0),
                "speed_xy_mps": safe_float(bird.get("speed_xy"), 0.0),
                "speed_3d_mps": safe_float(bird.get("speed_3d"), 0.0),
                "target_x_m": safe_float(bird.get("target_x")),
                "target_y_m": safe_float(bird.get("target_y")),
                "target_z_m": safe_float(bird.get("target_z")),
                "nearest": str(name == nearest_name).lower(),
                "fleeing": str(bool(bird.get("fleeing"))).lower(),
                "inside_lidar_area": str(bool(bird.get("inside_lidar_area"))).lower(),
            }
            self.csvs["bird_kinematics"].row(row)
            self.latest_birds[name] = dict(bird)
            if name == nearest_name and bool(bird.get("active")) and not bool(bird.get("hidden")) and not bool(bird.get("removed")):
                self.latest_bird = {
                    "name": name,
                    "state": row["state"],
                    "active": True,
                    "hidden": False,
                    "fleeing": bool(bird.get("fleeing")),
                    "x": row["x_m"],
                    "y": row["y_m"],
                    "z": row["z_m"],
                    "speed_xy": row["speed_xy_mps"],
                    "speed_3d": row["speed_3d_mps"],
                }

    def lidar_filter_response_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.csvs["lidar_filter_response"].row(payload)
        self.lidar_provenance = str(payload.get("provenance", self.lidar_provenance))
        if payload.get("target_ok"):
            self.remember_first("first_lidar_filter_done_time_sec", safe_float(payload.get("filter_done_time_sec"), self.now_sec()))
            self.remember_first("first_lidar_target_ok_time_sec", safe_float(payload.get("filter_done_time_sec"), self.now_sec()))
            self.latest_lidar_target = {
                "x": safe_float(payload.get("lidar_target_x_m")),
                "y": safe_float(payload.get("lidar_target_y_m")),
                "z": safe_float(payload.get("lidar_target_z_m")),
                "valid": bool(payload.get("lidar_target_valid", payload.get("target_ok"))),
                "age": safe_float(payload.get("lidar_target_age_sec"), 0.0),
                "track_id": safe_float(payload.get("track_id"), -1),
            }
        else:
            self.latest_lidar_target.update({"valid": False, "age": safe_float(payload.get("lidar_target_age_sec"), math.nan)})

    def lidar_state_callback(self, msg: String) -> None:
        self.lidar_state = msg.data
        lower = msg.data.lower()
        if "ignored_for_decision=true" in lower:
            return
        if "provenance=gazebo_gt" in msg.data and self.detector_mode == "lidar":
            self.lidar_provenance = "gazebo_gt"
        elif "provenance=lidar" in msg.data:
            self.lidar_provenance = "lidar"

    def dynamic_targets_callback(self, msg: PoseArray) -> None:
        if msg.poses:
            p = msg.poses[0].position
            track_id = self.latest_lidar_target.get("track_id", -1)
            self.latest_lidar_target = {
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z),
                "valid": True,
                "age": 0.0,
                "track_id": track_id,
            }

    def lidar_target_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        track_id = self.latest_lidar_target.get("track_id", -1)
        self.latest_lidar_target = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "valid": True,
            "age": 0.0,
            "track_id": track_id,
        }

    def dynamic_lock_callback(self, msg: Bool) -> None:
        self.dynamic_lock = bool(msg.data)
        if self.dynamic_lock:
            self.remember_first("first_dynamic_lock_true_time_sec")

    def object_goal_callback(self, msg: PoseStamped) -> None:
        self.latest_object_goal = msg
        self.latest_object_goal_time = self.now_sec()
        if self.goal_role_for("object_mission_goal") == "TARGET_INSPECTION":
            self.remember_first("first_object_mission_goal_time_sec")
        self.write_goal_event("object_mission_goal", msg)

    def object_goal_debug_callback(self, msg: String) -> None:
        try:
            self.latest_object_goal_debug = json.loads(msg.data)
        except Exception:
            self.latest_object_goal_debug = {"raw": msg.data}

    def inspection_target_callback(self, msg: PoseStamped) -> None:
        pass

    def active_goal_callback(self, msg: PoseStamped) -> None:
        self.latest_active_goal = msg
        role = self.infer_active_goal_role(msg)
        self.latest_active_goal_role = role
        if "first_object_mission_goal_time_sec" in self.first_times:
            self.remember_first("first_active_nav_goal_time_sec")
        if role == "TARGET_INSPECTION" and "first_object_mission_goal_time_sec" in self.first_times:
            self.remember_first("first_active_target_nav_goal_time_sec")
            robot = self.current_robot()
            if math.isfinite(robot["x"]) and math.isfinite(robot["y"]):
                self.target_goal_odom_start = (robot["x"], robot["y"])
        self.write_goal_event("active_nav_goal", msg)

    def active_goal_meta_callback(self, msg: String) -> None:
        try:
            self.latest_active_goal_meta = json.loads(msg.data)
        except Exception:
            self.latest_active_goal_meta = {"raw": msg.data}
        self.latest_active_goal_meta_time = self.now_sec()

    def sim_nav2_debug_callback(self, msg: String) -> None:
        try:
            self.latest_sim_nav2_debug = json.loads(msg.data)
        except Exception:
            self.latest_sim_nav2_debug = {"raw": msg.data}

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = msg.data
        token = msg.data.strip().split()[0].upper() if msg.data.strip() else ""
        if token == "APPROACH_TARGET_OFFSET":
            self.remember_first("first_approach_state_time_sec")

    def mission_event_callback(self, msg: String) -> None:
        pass

    def sound_done_callback(self, msg: Bool) -> None:
        self.sound_done = bool(msg.data)
        if self.sound_done:
            self.sound_done_seen = True

    def removal_state_callback(self, msg: String) -> None:
        self.removal_state = msg.data

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.remember_first("first_pointcloud_time_sec", stamp_sec(msg) or self.now_sec())

    def current_robot(self) -> dict[str, float]:
        if self.odom is None:
            return {"x": math.nan, "y": math.nan, "z": math.nan, "yaw": math.nan}
        p = self.odom.pose.pose.position
        return {"x": float(p.x), "y": float(p.y), "z": float(p.z), "yaw": yaw_from_pose(self.odom.pose.pose)}

    def current_bird(self) -> dict[str, Any]:
        return {
            "name": self.latest_bird.get("name", ""),
            "state": self.latest_bird.get("state", ""),
            "active": bool(self.latest_bird.get("active", False)),
            "hidden": bool(self.latest_bird.get("hidden", False)),
            "fleeing": bool(self.latest_bird.get("fleeing", False)),
            "x": safe_float(self.latest_bird.get("x")),
            "y": safe_float(self.latest_bird.get("y")),
            "z": safe_float(self.latest_bird.get("z")),
            "speed_xy": safe_float(self.latest_bird.get("speed_xy"), 0.0),
            "speed_3d": safe_float(self.latest_bird.get("speed_3d"), 0.0),
        }

    def goal_role_for(self, event_type: str) -> str:
        if event_type == "object_mission_goal":
            return str(self.latest_object_goal_debug.get("goal_role", "TARGET_INSPECTION"))
        if event_type == "active_nav_goal":
            return self.latest_active_goal_role
        return "UNKNOWN"

    def infer_active_goal_role(self, goal: PoseStamped) -> str:
        state_token = self.mission_state.strip().split()[0].upper() if self.mission_state.strip() else "UNKNOWN"
        meta_role = str(self.latest_active_goal_meta.get("goal_role", "UNKNOWN")).upper()
        target_states = {"INSPECTION_GOAL_GENERATED", "APPROACH_TARGET_OFFSET", "TARGET_NAVIGATING"}
        terminal_target_states = {
            "TARGET_REACHED",
            "CAMERA_ALIGN_TO_TARGET",
            "CAMERA_ALIGN_DONE",
            "TARGET_CLASSIFICATION_WAIT",
        }
        if self.latest_object_goal is not None:
            gx, gy, _gz, _gyaw = self.goal_xy(goal)
            ox, oy, _oz, _oyaw = self.goal_xy(self.latest_object_goal)
            object_age = self.now_sec() - self.latest_object_goal_time
            recent_object_match = (
                math.isfinite(gx)
                and math.isfinite(gy)
                and math.isfinite(ox)
                and math.isfinite(oy)
                and object_age <= 2.0
                and dist_xy(gx, gy, ox, oy) <= 0.35
            )
            if recent_object_match:
                return "TARGET_INSPECTION"
            if (
                math.isfinite(gx)
                and math.isfinite(gy)
                and math.isfinite(ox)
                and math.isfinite(oy)
                and object_age <= 2.0
                and dist_xy(gx, gy, ox, oy) <= 0.35
                and (state_token in target_states or state_token in terminal_target_states)
            ):
                return "TARGET_INSPECTION"
        if meta_role == "TARGET_INSPECTION" and state_token in target_states.union(terminal_target_states):
            return "TARGET_INSPECTION"
        if (
            meta_role
            and meta_role not in {"UNKNOWN", "TARGET_INSPECTION"}
            and self.now_sec() - self.latest_active_goal_meta_time <= 1.0
        ):
            return meta_role
        if state_token.startswith("RETURN"):
            return "RETURN_TO_INTERRUPTED_WAYPOINT"
        if state_token.startswith("PATROL") or state_token == "RESUME_PATROL":
            return "PATROL"
        return meta_role or "UNKNOWN"

    def goal_xy(self, goal: PoseStamped | None) -> tuple[float, float, float, float]:
        if goal is None:
            return (math.nan, math.nan, math.nan, math.nan)
        p = goal.pose.position
        return (float(p.x), float(p.y), float(p.z), yaw_from_pose(goal.pose))

    def sample_timer(self) -> None:
        robot = self.current_robot()
        bird = self.current_bird()
        lidar = self.latest_lidar_target
        obj_x, obj_y, _obj_z, _obj_yaw = self.goal_xy(self.latest_object_goal)
        act_x, act_y, _act_z, _act_yaw = self.goal_xy(self.latest_active_goal)
        lidar_x = safe_float(lidar.get("x"))
        lidar_y = safe_float(lidar.get("y"))
        lidar_z = safe_float(lidar.get("z"))
        lidar_valid = bool(lidar.get("valid", bool(self.dynamic_lock)))
        lidar_age = safe_float(lidar.get("age"))
        lidar_track_id = safe_float(lidar.get("track_id"), -1)
        active_goal_role = self.goal_role_for("active_nav_goal")
        active_goal_to_bird = dist_xy(act_x, act_y, bird["x"], bird["y"])
        active_goal_to_lidar = dist_xy(act_x, act_y, lidar_x, lidar_y)
        self.csvs["spatial_distance_timeseries"].row(
            {
                "time_sec": self.now_sec(),
                "mission_state": self.mission_state,
                "waver_x_m": robot["x"],
                "waver_y_m": robot["y"],
                "waver_z_m": robot["z"],
                "waver_yaw_rad": robot["yaw"],
                "bird_name": bird["name"],
                "bird_state": bird["state"],
                "bird_active": str(bird["active"]).lower(),
                "bird_hidden": str(bird["hidden"]).lower(),
                "bird_fleeing": str(bird["fleeing"]).lower(),
                "bird_x_m": bird["x"],
                "bird_y_m": bird["y"],
                "bird_z_m": bird["z"],
                "bird_speed_xy_mps": bird["speed_xy"],
                "bird_speed_3d_mps": bird["speed_3d"],
                "lidar_target_x_m": lidar_x,
                "lidar_target_y_m": lidar_y,
                "lidar_target_z_m": lidar_z,
                "lidar_target_valid": str(lidar_valid).lower(),
                "lidar_target_age_sec": lidar_age,
                "lidar_track_id": lidar_track_id,
                "object_goal_x_m": obj_x,
                "object_goal_y_m": obj_y,
                "active_nav_goal_x_m": act_x,
                "active_nav_goal_y_m": act_y,
                "active_goal_role": active_goal_role,
                "robot_to_bird_xy_m": dist_xy(robot["x"], robot["y"], bird["x"], bird["y"]),
                "robot_to_bird_3d_m": dist_3d(robot["x"], robot["y"], robot["z"], bird["x"], bird["y"], bird["z"]),
                "robot_to_lidar_target_xy_m": dist_xy(robot["x"], robot["y"], lidar_x, lidar_y),
                "lidar_target_to_bird_xy_m": dist_xy(lidar_x, lidar_y, bird["x"], bird["y"]),
                "object_goal_to_bird_xy_m": dist_xy(obj_x, obj_y, bird["x"], bird["y"]),
                "active_goal_to_bird_xy_m": active_goal_to_bird,
                "object_goal_to_lidar_target_xy_m": dist_xy(obj_x, obj_y, lidar_x, lidar_y),
                "active_goal_to_lidar_target_xy_m": active_goal_to_lidar,
                "robot_to_active_goal_xy_m": dist_xy(robot["x"], robot["y"], act_x, act_y),
                "configured_offset_m": self.configured_offset_m,
                "active_goal_standoff_error_to_bird_m": "" if math.isnan(active_goal_to_bird) else active_goal_to_bird - self.configured_offset_m,
                "active_goal_standoff_error_to_lidar_target_m": "" if math.isnan(active_goal_to_lidar) else active_goal_to_lidar - self.configured_offset_m,
                "detector_mode": self.detector_mode,
                "lidar_provenance": self.lidar_provenance,
                "dynamic_lock": str(self.dynamic_lock).lower(),
                "cmd_vel_nonzero": str(self.cmd_vel_nonzero).lower(),
            }
        )
        self.write_metrics()
        self.write_latency_row()
        self.state_pub.publish(String(data=f"SPATIAL_RESPONSE_LOGGING run_dir={self.run_dir}"))

    def write_goal_event(self, event_type: str, goal: PoseStamped) -> None:
        robot = self.current_robot()
        bird = self.current_bird()
        goal_x, goal_y, goal_z, goal_yaw = self.goal_xy(goal)
        lidar = self.latest_lidar_target
        lidar_x = safe_float(lidar.get("x"))
        lidar_y = safe_float(lidar.get("y"))
        lidar_z = safe_float(lidar.get("z"))
        lidar_valid = bool(lidar.get("valid", bool(self.dynamic_lock)))
        lidar_age = safe_float(lidar.get("age"))
        lidar_track_id = safe_float(lidar.get("track_id"), -1)
        goal_role = self.goal_role_for(event_type)
        state_token = self.mission_state.strip().split()[0].upper() if self.mission_state.strip() else "UNKNOWN"
        active_target_states = {
            "INSPECTION_GOAL_GENERATED",
            "APPROACH_TARGET_OFFSET",
            "TARGET_NAVIGATING",
            "TARGET_REACHED",
            "CAMERA_ALIGN_TO_TARGET",
            "CAMERA_ALIGN_DONE",
            "TARGET_CLASSIFICATION_WAIT",
        }
        recent_object_match = False
        if event_type == "active_nav_goal" and self.latest_object_goal is not None:
            ox, oy, _oz, _oyaw = self.goal_xy(self.latest_object_goal)
            object_age = self.now_sec() - self.latest_object_goal_time
            recent_object_match = (
                math.isfinite(goal_x)
                and math.isfinite(goal_y)
                and math.isfinite(ox)
                and math.isfinite(oy)
                and object_age <= 2.0
                and dist_xy(goal_x, goal_y, ox, oy) <= 0.35
            )
        if (
            event_type == "active_nav_goal"
            and goal_role == "TARGET_INSPECTION"
            and state_token not in active_target_states
            and not recent_object_match
        ):
            goal_role = "PATROL" if state_token in {"RESUME_PATROL", "PATROL_DWELL", "PATROL_NAVIGATING"} else state_token
        target_related = goal_role == "TARGET_INSPECTION"
        goal_to_bird = dist_xy(goal_x, goal_y, bird["x"], bird["y"])
        goal_to_lidar = dist_xy(goal_x, goal_y, lidar_x, lidar_y)
        lidar_to_bird = dist_xy(lidar_x, lidar_y, bird["x"], bird["y"])
        self.csvs["goal_bird_distance_events"].row(
            {
                "time_sec": self.now_sec(),
                "event_type": event_type,
                "goal_role": goal_role,
                "mission_state": self.mission_state,
                "is_target_related_goal": str(target_related).lower(),
                "target_track_id": lidar_track_id if target_related else "",
                "target_bird_name": bird["name"] if target_related else "",
                "association_method": "nearest_active_gt_at_goal_generation" if target_related else "",
                "association_distance_m": lidar_to_bird if target_related else "",
                "bird_name": bird["name"],
                "bird_state": bird["state"],
                "bird_x_m": bird["x"],
                "bird_y_m": bird["y"],
                "bird_z_m": bird["z"],
                "bird_speed_xy_mps": bird["speed_xy"],
                "bird_speed_3d_mps": bird["speed_3d"],
                "goal_frame": goal.header.frame_id,
                "goal_x_m": goal_x,
                "goal_y_m": goal_y,
                "goal_z_m": goal_z,
                "goal_yaw_rad": goal_yaw,
                "waver_x_m": robot["x"],
                "waver_y_m": robot["y"],
                "waver_yaw_rad": robot["yaw"],
                "goal_to_bird_xy_m": goal_to_bird,
                "goal_to_bird_3d_m": dist_3d(goal_x, goal_y, goal_z, bird["x"], bird["y"], bird["z"]),
                "robot_to_goal_xy_m": dist_xy(robot["x"], robot["y"], goal_x, goal_y),
                "robot_to_bird_xy_m": dist_xy(robot["x"], robot["y"], bird["x"], bird["y"]),
                "configured_offset_m": self.configured_offset_m,
                "standoff_error_m": "" if math.isnan(goal_to_bird) else goal_to_bird - self.configured_offset_m,
                "lidar_target_x_m": lidar_x,
                "lidar_target_y_m": lidar_y,
                "lidar_target_z_m": lidar_z,
                "lidar_target_valid": str(lidar_valid).lower(),
                "lidar_target_age_sec": lidar_age,
                "lidar_track_id": lidar_track_id,
                "goal_to_lidar_target_xy_m": goal_to_lidar,
                "goal_to_lidar_target_3d_m": dist_3d(goal_x, goal_y, goal_z, lidar_x, lidar_y, lidar_z),
                "lidar_target_to_bird_xy_m": lidar_to_bird,
                "standoff_error_to_bird_m": "" if math.isnan(goal_to_bird) else goal_to_bird - self.configured_offset_m,
                "standoff_error_to_lidar_target_m": "" if math.isnan(goal_to_lidar) else goal_to_lidar - self.configured_offset_m,
            }
        )

    def elapsed_ms(self, key: str) -> float | str:
        if key not in self.first_times:
            return ""
        return (self.now_sec() - self.first_times[key]) * 1000.0

    def latency_delta(self, start: str, end: str) -> float | str:
        if start not in self.first_times or end not in self.first_times:
            return ""
        return (self.first_times[end] - self.first_times[start]) * 1000.0

    def write_latency_row(self) -> None:
        path = self.csvs["lidar_waver_latency_events"]
        path.handle.seek(0)
        path.handle.truncate()
        path.writer.writeheader()
        path.row(
            {
                "trial_id": str(self.get_parameter("trial_id").value),
                **{key: self.first_times.get(key, "") for key in path.fields if key.startswith("first_")},
                "pointcloud_to_lidar_target_ok_ms": self.latency_delta("first_pointcloud_time_sec", "first_lidar_target_ok_time_sec"),
                "lidar_target_ok_to_dynamic_lock_ms": self.latency_delta("first_lidar_target_ok_time_sec", "first_dynamic_lock_true_time_sec"),
                "dynamic_lock_to_object_mission_goal_ms": self.latency_delta("first_dynamic_lock_true_time_sec", "first_object_mission_goal_time_sec"),
                "object_mission_goal_to_active_nav_goal_ms": self.latency_delta("first_object_mission_goal_time_sec", "first_active_nav_goal_time_sec"),
                "object_mission_goal_to_active_target_nav_goal_ms": self.latency_delta("first_object_mission_goal_time_sec", "first_active_target_nav_goal_time_sec"),
                "active_nav_goal_to_approach_state_ms": self.latency_delta("first_active_nav_goal_time_sec", "first_approach_state_time_sec"),
                "active_nav_goal_to_first_cmd_vel_ms": self.latency_delta("first_active_nav_goal_time_sec", "first_nonzero_cmd_vel_time_sec"),
                "active_target_nav_goal_to_cmd_vel_ms": self.latency_delta("first_active_target_nav_goal_time_sec", "first_nonzero_cmd_after_target_goal_time_sec"),
                "active_target_nav_goal_to_odom_motion_ms": self.latency_delta("first_active_target_nav_goal_time_sec", "first_odom_motion_after_target_goal_time_sec"),
                "lidar_target_ok_to_first_cmd_vel_ms": self.latency_delta("first_lidar_target_ok_time_sec", "first_nonzero_cmd_vel_time_sec"),
                "lidar_target_ok_to_first_target_cmd_vel_ms": self.latency_delta("first_lidar_target_ok_time_sec", "first_nonzero_cmd_after_target_goal_time_sec"),
                "pointcloud_to_object_mission_goal_ms": self.latency_delta("first_pointcloud_time_sec", "first_object_mission_goal_time_sec"),
            }
        )

    def read_csv_rows(self, name: str) -> list[dict[str, str]]:
        path = self.csvs[name].path
        try:
            with path.open(newline="", encoding="utf-8") as f:
                return list(csv.DictReader(f))
        except Exception:
            return []

    def read_log_csv_rows(self, rel_path: str) -> list[dict[str, str]]:
        path = self.run_dir / rel_path
        if not path.exists():
            return []
        try:
            with path.open(newline="", encoding="utf-8", errors="replace") as f:
                return list(csv.DictReader(f))
        except Exception:
            return []

    @staticmethod
    def truth(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "pass", "ok"}

    @staticmethod
    def row_text(row: dict[str, Any]) -> str:
        return " ".join(str(v) for v in row.values())

    @staticmethod
    def first_time_after(rows: list[dict[str, str]], predicate: Any, after: float | None) -> float | None:
        for row in rows:
            if not predicate(row):
                continue
            for field in ("time_sec", "event_time_sec", "ros_time_sec"):
                value = safe_float(row.get(field))
                if math.isfinite(value) and (after is None or value >= after):
                    return value
        return None

    @classmethod
    def first_text_time_after(cls, rows: list[dict[str, str]], tokens: tuple[str, ...], after: float | None) -> float | None:
        upper_tokens = tuple(token.upper() for token in tokens)
        return cls.first_time_after(
            rows,
            lambda r: all(token in cls.row_text(r).upper() for token in upper_tokens),
            after,
        )

    def parse_cmd_vel_authority(self) -> tuple[int | str, bool]:
        path = self.run_dir / "logs/cmd_vel_topic_info.txt"
        if not path.exists():
            return "", False
        text = path.read_text(encoding="utf-8", errors="replace")
        publisher_count: int | None = None
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("Publisher count:"):
                try:
                    publisher_count = int(stripped.split(":", 1)[1].strip())
                except Exception:
                    publisher_count = None
        sole_mux = (
            publisher_count == 1
            and "safety_cmd_mux_node" in text
            and "simple_nav2_cmd_sim_node" not in text
            and "seo_cluster_tracker_node" not in text
        )
        return (publisher_count if publisher_count is not None else "", sole_mux)

    @staticmethod
    def numeric(rows: list[dict[str, str]], field: str) -> list[float]:
        vals = []
        for row in rows:
            v = safe_float(row.get(field))
            if math.isfinite(v):
                vals.append(v)
        return vals

    @staticmethod
    def numeric_any(rows: list[dict[str, str]], fields: list[str]) -> list[float]:
        vals = []
        for row in rows:
            for field in fields:
                v = safe_float(row.get(field))
                if math.isfinite(v):
                    vals.append(v)
                    break
        return vals

    @staticmethod
    def mean(vals: list[float]) -> float | str:
        return statistics.mean(vals) if vals else ""

    @staticmethod
    def std(vals: list[float]) -> float | str:
        return statistics.pstdev(vals) if len(vals) > 1 else ""

    @staticmethod
    def p95(vals: list[float]) -> float | str:
        if not vals:
            return ""
        vals = sorted(vals)
        index = min(len(vals) - 1, int(math.ceil(0.95 * len(vals))) - 1)
        return vals[index]

    def write_metrics(self) -> None:
        spatial_rows = self.read_csv_rows("spatial_distance_timeseries")
        goal_rows = self.read_csv_rows("goal_bird_distance_events")
        bird_rows = self.read_csv_rows("bird_kinematics")
        lidar_rows = self.read_csv_rows("lidar_filter_response")
        cmd_rows = self.read_csv_rows("cmd_vel_response")
        mission_events = self.read_log_csv_rows("logs/mission_events.csv")
        removal_events = self.read_log_csv_rows("logs/bird_removal_events.csv")
        target_goal_rows = [
            r
            for r in goal_rows
            if str(r.get("is_target_related_goal", "")).lower() == "true"
            or str(r.get("goal_role", "")).upper() == "TARGET_INSPECTION"
        ]
        object_target_goal_rows = [r for r in target_goal_rows if r.get("event_type") == "object_mission_goal"]
        target_spatial_rows = [
            r for r in spatial_rows if str(r.get("active_goal_role", "")).upper() == "TARGET_INSPECTION"
        ]
        valid_lidar_spatial_rows = [
            r
            for r in spatial_rows
            if self.truth(r.get("lidar_target_valid"))
            and safe_float(r.get("lidar_target_age_sec")) <= 0.5
            and self.truth(r.get("dynamic_lock"))
        ]
        active_bird_rows = [
            r
            for r in bird_rows
            if str(r.get("active", "")).lower() == "true"
            and str(r.get("hidden", "")).lower() != "true"
            and str(r.get("removed", "")).lower() != "true"
        ]
        active_flee_bird_rows = [r for r in active_bird_rows if str(r.get("fleeing", "")).lower() == "true"]
        active_normal_bird_rows = [r for r in active_bird_rows if str(r.get("fleeing", "")).lower() != "true"]
        removed = {
            str(r.get("bird_name", ""))
            for r in bird_rows
            if str(r.get("removed", "")).lower() == "true" and str(r.get("bird_name", ""))
        }
        gt_leakage = sum(
            1
            for r in lidar_rows
            if self.detector_mode == "lidar" and str(r.get("provenance", "")).lower() in {"gazebo_gt", "ground_truth"}
        )
        cmd_duration = 0.0
        if self.nonzero_cmd_first is not None and self.nonzero_cmd_last is not None:
            cmd_duration = max(0.0, self.nonzero_cmd_last - self.nonzero_cmd_first)
        net_disp = 0.0
        if self.odom_first is not None and self.odom_last is not None:
            net_disp = math.hypot(self.odom_last[0] - self.odom_first[0], self.odom_last[1] - self.odom_first[1])
        target_goal_to_bird = self.numeric(object_target_goal_rows, "goal_to_bird_xy_m")
        target_goal_to_lidar = self.numeric(object_target_goal_rows, "goal_to_lidar_target_xy_m")
        lidar_runtime_wall = self.numeric_any(lidar_rows, ["filter_runtime_wall_ms", "filter_runtime_ms"])
        cmd_vel_publisher_count, cmd_vel_sole_mux = self.parse_cmd_vel_authority()

        first_object_time = self.first_times.get("first_object_mission_goal_time_sec")
        first_active_target_time = self.first_times.get("first_active_target_nav_goal_time_sec")
        first_patrol_goal_time = self.first_text_time_after(
            mission_events,
            ("NAV2_DISABLED_SIMULATED_GOAL", "PATROL"),
            None,
        )
        if first_patrol_goal_time is None:
            first_patrol_goal_time = self.first_time_after(
                goal_rows,
                lambda r: r.get("event_type") == "active_nav_goal"
                and str(r.get("goal_role", "")).upper() not in {"TARGET_INSPECTION", "RADAR_TARGET"},
                None,
            )
        first_patrol_preempt_time = self.first_time_after(
            mission_events,
            lambda r: "TARGET_PREEMPTS_PATROL_IMMEDIATELY" in self.row_text(r).upper()
            or "INTERRUPT_PATROL_FOR_LIDAR_TARGET" in self.row_text(r).upper(),
            first_patrol_goal_time,
        )
        first_patrol_dwell_before_preempt = self.first_time_after(
            mission_events,
            lambda r: "PATROL_DWELL" in self.row_text(r).upper(),
            first_patrol_goal_time,
        )
        first_patrol_success_before_preempt = self.first_time_after(
            mission_events,
            lambda r: ("SIM_NAV_GOAL_ARRIVED" in self.row_text(r).upper() or "NAV2_GOAL_SUCCEEDED" in self.row_text(r).upper())
            and "PATROL" in self.row_text(r).upper(),
            first_patrol_goal_time,
        )
        first_target_cmd_after_preempt = self.first_time_after(
            cmd_rows,
            lambda r: str(r.get("nonzero_cmd", "")).lower() == "true"
            and (
                str(r.get("mission_state", "")).upper().startswith("APPROACH_TARGET_OFFSET")
                or str(r.get("mission_state", "")).upper().startswith("TARGET_NAVIGATING")
            ),
            first_patrol_preempt_time,
        )
        if first_target_cmd_after_preempt is None:
            first_target_cmd_after_preempt = self.first_times.get("first_nonzero_cmd_after_target_goal_time_sec")
        patrol_dwell_before_preempt = (
            first_patrol_dwell_before_preempt is not None
            and first_patrol_preempt_time is not None
            and first_patrol_dwell_before_preempt < first_patrol_preempt_time
        )
        patrol_success_before_preempt = (
            first_patrol_success_before_preempt is not None
            and first_patrol_preempt_time is not None
            and first_patrol_success_before_preempt < first_patrol_preempt_time
        )
        mid_patrol_preempt_success = (
            first_patrol_goal_time is not None
            and first_patrol_preempt_time is not None
            and first_active_target_time is not None
            and first_target_cmd_after_preempt is not None
            and first_patrol_goal_time <= first_patrol_preempt_time <= first_active_target_time
            and not patrol_dwell_before_preempt
            and not patrol_success_before_preempt
        )
        removed_times = [
            safe_float(row.get("time_sec") or row.get("event_time_sec"))
            for row in removal_events
            if str(row.get("event", row.get("state", ""))).upper().startswith("REMOVED")
        ]
        removed_times = [value for value in removed_times if math.isfinite(value)]
        if not removed_times:
            removed_times = [
                safe_float(row.get("time_sec"))
                for row in bird_rows
                if self.truth(row.get("removed")) and math.isfinite(safe_float(row.get("time_sec")))
            ]
        last_removed_time = max(removed_times) if removed_times else None
        first_return_time = self.first_time_after(
            spatial_rows,
            lambda r: "RETURN" in str(r.get("mission_state", "")).upper(),
            last_removed_time,
        ) or self.first_time_after(
            mission_events,
            lambda r: "RETURN" in self.row_text(r).upper(),
            last_removed_time,
        )
        first_resume_time = self.first_time_after(
            spatial_rows,
            lambda r: "RESUME" in str(r.get("mission_state", "")).upper(),
            first_return_time,
        ) or self.first_time_after(
            mission_events,
            lambda r: "RESUME" in self.row_text(r).upper(),
            first_return_time,
        )
        final_patrol_time = self.first_time_after(
            spatial_rows,
            lambda r: str(r.get("mission_state", "")).upper().startswith("PATROL"),
            first_resume_time,
        ) or self.first_time_after(
            mission_events,
            lambda r: "PATROL" in self.row_text(r).upper(),
            first_resume_time,
        )
        return_resume_sequence = (
            last_removed_time is not None
            and first_return_time is not None
            and first_resume_time is not None
            and final_patrol_time is not None
            and last_removed_time <= first_return_time <= first_resume_time <= final_patrol_time
        )
        has_return_resume = False
        target_goal_time = self.first_times.get("first_active_target_nav_goal_time_sec")
        if target_goal_time is not None:
            for row in spatial_rows:
                row_time = safe_float(row.get("time_sec"))
                state = str(row.get("mission_state", "")).upper()
                if math.isfinite(row_time) and row_time >= target_goal_time and (
                    "RETURN" in state or "RESUME" in state or state.startswith("PATROL")
                ):
                    has_return_resume = True
                    break
        metrics = {
            "mechanism.detector_mode": self.detector_mode,
            "mechanism.gt_leakage_in_lidar_mode_count": gt_leakage,
            "mechanism.lidar_only_decision_clean": self.detector_mode != "lidar" or gt_leakage == 0,
            "mechanism.has_lidar_target_ok": "first_lidar_target_ok_time_sec" in self.first_times,
            "mechanism.has_dynamic_lock": "first_dynamic_lock_true_time_sec" in self.first_times,
            "mechanism.has_object_mission_goal": "first_object_mission_goal_time_sec" in self.first_times,
            "mechanism.has_active_nav_goal": "first_active_nav_goal_time_sec" in self.first_times,
            "mechanism.has_active_target_nav_goal": "first_active_target_nav_goal_time_sec" in self.first_times,
            "mechanism.has_patrol_preempt_to_target_goal": first_patrol_preempt_time is not None and first_active_target_time is not None,
            "mechanism.mid_patrol_preempt_success": mid_patrol_preempt_success,
            "mechanism.preempt_before_first_patrol_dwell": first_patrol_preempt_time is not None and not patrol_dwell_before_preempt,
            "mechanism.preempt_before_first_patrol_success": first_patrol_preempt_time is not None and not patrol_success_before_preempt,
            "mechanism.has_nonzero_cmd_vel": "first_nonzero_cmd_vel_time_sec" in self.first_times,
            "mechanism.has_nonzero_cmd_vel_after_target_goal": "first_nonzero_cmd_after_target_goal_time_sec" in self.first_times,
            "mechanism.has_target_cmd_vel": "first_nonzero_cmd_after_target_goal_time_sec" in self.first_times,
            "mechanism.has_odom_motion_after_target_goal": "first_odom_motion_after_target_goal_time_sec" in self.first_times,
            "mechanism.has_return_resume": has_return_resume or return_resume_sequence,
            "mechanism.return_resume_sequence_success": return_resume_sequence,
            "mechanism.has_sound_task": self.sound_done_seen,
            "mechanism.removed_bird_count": len(removed),
            "mechanism.two_bird_removal_success": len(removed) >= 2,
            "mechanism.four_bird_removal_success": len(removed) >= 4,
            "mechanism.five_bird_removal_success": len(removed) >= 5,
            "mechanism.patrol_lap_count": int(self.patrol_lap_count),
            "mechanism.two_patrol_laps_success": int(self.patrol_lap_count) >= 2,
            "mechanism.validation_success_5_birds_2_laps": len(removed) >= 5 and int(self.patrol_lap_count) >= 2,
            "spatial.robot_to_bird_xy_mean_m": self.mean(self.numeric(spatial_rows, "robot_to_bird_xy_m")),
            "spatial.robot_to_bird_xy_min_m": min(self.numeric(spatial_rows, "robot_to_bird_xy_m") or [math.nan]),
            "spatial.object_goal_to_bird_xy_first_m": self.first_numeric(target_goal_rows, "goal_to_bird_xy_m", "object_mission_goal"),
            "spatial.active_goal_to_bird_xy_first_m": self.first_numeric(target_goal_rows, "goal_to_bird_xy_m", "active_nav_goal"),
            "spatial.target_goal_to_bird_xy_first_m": self.first_numeric(object_target_goal_rows, "goal_to_bird_xy_m", "object_mission_goal"),
            "spatial.target_goal_to_bird_xy_mean_m": self.mean(target_goal_to_bird),
            "spatial.target_goal_to_bird_xy_std_m": self.std(target_goal_to_bird),
            "spatial.target_goal_to_bird_xy_min_m": min(target_goal_to_bird or [math.nan]),
            "spatial.target_goal_to_bird_xy_max_m": max(target_goal_to_bird or [math.nan]),
            "spatial.target_goal_to_lidar_target_xy_first_m": self.first_numeric(object_target_goal_rows, "goal_to_lidar_target_xy_m", "object_mission_goal"),
            "spatial.target_goal_to_lidar_target_xy_mean_m": self.mean(target_goal_to_lidar),
            "spatial.target_goal_to_lidar_target_xy_std_m": self.std(target_goal_to_lidar),
            "spatial.goal_to_bird_xy_mean_m": self.mean(target_goal_to_bird),
            "spatial.standoff_error_mean_abs_m": self.mean([abs(v) for v in self.numeric(target_spatial_rows, "active_goal_standoff_error_to_bird_m")]),
            "spatial.target_goal_standoff_error_to_bird_mean_abs_m": self.mean(
                [abs(v) for v in self.numeric(object_target_goal_rows, "standoff_error_to_bird_m")]
            ),
            "spatial.target_goal_standoff_error_to_lidar_target_mean_abs_m": self.mean(
                [abs(v) for v in self.numeric(object_target_goal_rows, "standoff_error_to_lidar_target_m")]
            ),
            "spatial.lidar_target_to_bird_xy_mean_m": self.mean(self.numeric(spatial_rows, "lidar_target_to_bird_xy_m")),
            "spatial.lidar_target_to_bird_xy_p95_m": self.p95(self.numeric(spatial_rows, "lidar_target_to_bird_xy_m")),
            "spatial.valid_lidar_target_sample_count": len(valid_lidar_spatial_rows),
            "spatial.valid_lidar_target_to_bird_xy_mean_m": self.mean(self.numeric(valid_lidar_spatial_rows, "lidar_target_to_bird_xy_m")),
            "spatial.valid_lidar_target_to_bird_xy_p95_m": self.p95(self.numeric(valid_lidar_spatial_rows, "lidar_target_to_bird_xy_m")),
            "spatial.valid_lidar_target_to_bird_xy_max_m": max(self.numeric(valid_lidar_spatial_rows, "lidar_target_to_bird_xy_m") or [math.nan]),
            "bird.speed_xy_mean_mps": self.mean(self.numeric(active_bird_rows, "speed_xy_mps")),
            "bird.speed_xy_max_mps": max(self.numeric(active_bird_rows, "speed_xy_mps") or [math.nan]),
            "bird.speed_xy_normal_mean_mps": self.mean(self.numeric(active_normal_bird_rows, "speed_xy_mps")),
            "bird.speed_xy_flee_mean_mps": self.mean(self.numeric(active_flee_bird_rows, "speed_xy_mps")),
            "bird.motion_sample_count": len(active_bird_rows),
            "lidar.filter_runtime_mean_ms": self.mean(lidar_runtime_wall),
            "lidar.filter_runtime_p95_ms": self.p95(lidar_runtime_wall),
            "lidar.filter_runtime_wall_mean_ms": self.mean(lidar_runtime_wall),
            "lidar.filter_runtime_wall_p95_ms": self.p95(lidar_runtime_wall),
            "lidar.pc_stamp_to_filter_done_mean_ms": self.mean(self.numeric(lidar_rows, "pc_stamp_to_filter_done_ms")),
            "lidar.pc_stamp_to_filter_done_sim_mean_ms": self.mean(self.numeric(lidar_rows, "pc_stamp_to_filter_done_sim_ms")),
            "lidar.raw_points_mean": self.mean(self.numeric(lidar_rows, "raw_points")),
            "lidar.roi_points_mean": self.mean(self.numeric(lidar_rows, "roi_points")),
            "lidar.cluster_points_mean": self.mean(self.numeric(lidar_rows, "cluster_points")),
            "lidar.target_ok_count": sum(1 for r in lidar_rows if str(r.get("target_ok", "")).lower() == "true"),
            "latency.lidar_target_ok_to_first_cmd_vel_ms": self.latency_delta("first_lidar_target_ok_time_sec", "first_nonzero_cmd_vel_time_sec"),
            "latency.lidar_target_ok_to_first_target_cmd_vel_ms": self.latency_delta(
                "first_lidar_target_ok_time_sec", "first_nonzero_cmd_after_target_goal_time_sec"
            ),
            "latency.pointcloud_to_object_mission_goal_ms": self.latency_delta("first_pointcloud_time_sec", "first_object_mission_goal_time_sec"),
            "latency.object_mission_goal_to_active_target_nav_goal_ms": self.latency_delta(
                "first_object_mission_goal_time_sec", "first_active_target_nav_goal_time_sec"
            ),
            "latency.active_target_nav_goal_to_cmd_vel_ms": self.latency_delta(
                "first_active_target_nav_goal_time_sec", "first_nonzero_cmd_after_target_goal_time_sec"
            ),
            "latency.patrol_goal_to_target_preempt_ms": (
                ""
                if first_patrol_goal_time is None or first_patrol_preempt_time is None
                else (first_patrol_preempt_time - first_patrol_goal_time) * 1000.0
            ),
            "latency.target_preempt_to_active_target_nav_goal_ms": (
                ""
                if first_patrol_preempt_time is None or first_active_target_time is None
                else (first_active_target_time - first_patrol_preempt_time) * 1000.0
            ),
            "latency.target_preempt_to_target_cmd_vel_ms": (
                ""
                if first_patrol_preempt_time is None or first_target_cmd_after_preempt is None
                else (first_target_cmd_after_preempt - first_patrol_preempt_time) * 1000.0
            ),
            "latency.active_target_nav_goal_to_odom_motion_ms": self.latency_delta(
                "first_active_target_nav_goal_time_sec", "first_odom_motion_after_target_goal_time_sec"
            ),
            "navigation.odom_path_length_m": self.odom_path_length,
            "navigation.net_displacement_m": net_disp,
            "navigation.cmd_vel_nonzero_rows": self.nonzero_cmd_rows,
            "navigation.cmd_vel_active_duration_sec": cmd_duration,
            "navigation.linear_cmd_max_abs": self.linear_cmd_abs_max,
            "navigation.angular_cmd_max_abs": self.angular_cmd_abs_max,
            "safety.cmd_vel_publisher_count": cmd_vel_publisher_count,
            "safety.cmd_vel_safety_mux_sole_publisher": cmd_vel_sole_mux,
            "safety.cmd_vel_authority_pass": cmd_vel_sole_mux,
        }
        (self.run_dir / "metrics/spatial_response_metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        self.write_metric_csv("spatial_response_metrics.csv", metrics)
        self.write_paper_tables(metrics)

    @staticmethod
    def first_numeric(rows: list[dict[str, str]], field: str, event_type: str) -> float | str:
        for row in rows:
            if row.get("event_type") == event_type:
                value = safe_float(row.get(field))
                if math.isfinite(value):
                    return value
        return ""

    def write_metric_csv(self, name: str, metrics: dict[str, Any]) -> None:
        with (self.run_dir / "metrics" / name).open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["metric", "value"])
            writer.writeheader()
            for key, value in metrics.items():
                writer.writerow({"metric": key, "value": value})

    def write_paper_tables(self, metrics: dict[str, Any]) -> None:
        spatial = [
            ("target_goal_to_bird_xy_first_m", metrics.get("spatial.target_goal_to_bird_xy_first_m", ""), "m", "First target-inspection nav goal to bird XY distance"),
            ("target_goal_to_bird_xy_mean_m", metrics.get("spatial.target_goal_to_bird_xy_mean_m", ""), "m", "Mean target-inspection goal to bird XY distance"),
            ("target_goal_to_lidar_target_xy_mean_m", metrics.get("spatial.target_goal_to_lidar_target_xy_mean_m", ""), "m", "Mean target-inspection goal to LiDAR target XY distance"),
            ("target_goal_standoff_error_to_bird_mean_abs_m", metrics.get("spatial.target_goal_standoff_error_to_bird_mean_abs_m", ""), "m", "Mean absolute target-goal standoff error to bird"),
            ("target_goal_standoff_error_to_lidar_target_mean_abs_m", metrics.get("spatial.target_goal_standoff_error_to_lidar_target_mean_abs_m", ""), "m", "Mean absolute target-goal standoff error to LiDAR target"),
            ("robot_to_bird_xy_min_m", metrics.get("spatial.robot_to_bird_xy_min_m", ""), "m", "Minimum Waver to bird XY distance"),
            ("lidar_target_to_bird_xy_mean_m", metrics.get("spatial.lidar_target_to_bird_xy_mean_m", ""), "m", "Mean LiDAR target to bird GT XY distance"),
            ("bird_speed_xy_mean_mps", metrics.get("bird.speed_xy_mean_mps", ""), "m/s", "Mean bird XY speed"),
            ("odom_path_length_m", metrics.get("navigation.odom_path_length_m", ""), "m", "Waver odometry path length"),
        ]
        latency = [
            ("filter_runtime_wall_mean_ms", metrics.get("lidar.filter_runtime_wall_mean_ms", ""), "ms", "Mean LiDAR ROI/cluster filtering wall runtime"),
            ("pc_stamp_to_filter_done_mean_ms", metrics.get("lidar.pc_stamp_to_filter_done_mean_ms", ""), "ms", "PointCloud stamp to filter done latency"),
            ("lidar_target_ok_to_first_target_cmd_vel_ms", metrics.get("latency.lidar_target_ok_to_first_target_cmd_vel_ms", ""), "ms", "LiDAR target OK to first target-approach Waver command"),
            ("pointcloud_to_object_mission_goal_ms", metrics.get("latency.pointcloud_to_object_mission_goal_ms", ""), "ms", "PointCloud to object mission goal"),
            ("active_target_nav_goal_to_odom_motion_ms", metrics.get("latency.active_target_nav_goal_to_odom_motion_ms", ""), "ms", "Active target nav goal to odom motion"),
        ]
        for rel, rows in [("paper_spatial_table.csv", spatial), ("paper_latency_table.csv", latency)]:
            with (self.run_dir / "metrics" / rel).open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["metric", "value", "unit", "meaning"])
                writer.writeheader()
                for metric, value, unit, meaning in rows:
                    writer.writerow({"metric": metric, "value": value, "unit": unit, "meaning": meaning})

    def destroy_node(self) -> bool:
        try:
            self.write_metrics()
            self.write_latency_row()
        except Exception:
            pass
        for csv_file in getattr(self, "csvs", {}).values():
            try:
                csv_file.handle.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaverSpatialResponseLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

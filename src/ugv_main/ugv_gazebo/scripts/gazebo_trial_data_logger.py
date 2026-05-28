#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def local_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_degrees(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return value.strip("_") or "trial"


class GazeboTrialDataLogger(Node):
    """CSV logger for bird-detection autonomous patrol experiments."""

    FIELDNAMES = [
        "session_id",
        "sample_index",
        "wall_time_iso",
        "ros_time_sec",
        "elapsed_s",
        "tracking_start_iso",
        "map_side_m",
        "patrol_side_m",
        "patrol_state",
        "patrol_completed",
        "waypoint_arrival_count",
        "robot_x_m",
        "robot_y_m",
        "robot_yaw_rad",
        "robot_speed_mps",
        "robot_path_length_m",
        "cmd_linear_mps",
        "cmd_angular_radps",
        "current_waypoint_x_m",
        "current_waypoint_y_m",
        "obstacle_count",
        "obstacle_id",
        "obstacle_x_m",
        "obstacle_y_m",
        "obstacle_z_m",
        "obstacle_dx_1s_m",
        "obstacle_dy_1s_m",
        "obstacle_dz_1s_m",
        "obstacle_speed_mps",
        "obstacle_direction_deg",
        "robot_to_obstacle_xy_m",
        "robot_to_obstacle_3d_m",
        "robot_to_obstacle_bearing_deg",
        "min_robot_obstacle_xy_m",
        "min_clearance_xy_m",
        "min_vertical_separation_m",
        "overhead_pass_flag",
        "collision_flag",
        "scan_front_min_m",
        "scan_valid_points",
        "bird_confirmed",
        "target_class",
        "target_confidence",
        "first_detection_elapsed_s",
        "dynamic_obstacle_state",
    ]

    def __init__(self) -> None:
        super().__init__("gazebo_trial_data_logger")
        self.declare_parameter("output_dir", "~/ros2_ws/bird_patrol_data")
        self.declare_parameter("session_name", "bird_patrol_trial")
        self.declare_parameter("sample_period_s", 1.0)
        self.declare_parameter("map_side_m", 15.0)
        self.declare_parameter("patrol_side_m", 10.0)
        self.declare_parameter("robot_collision_radius_m", 0.35)
        self.declare_parameter("obstacle_collision_radius_m", 0.15)
        self.declare_parameter("collision_vertical_threshold_m", 1.0)
        self.declare_parameter("dynamic_targets_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("dynamic_obstacle_state_topic", "/waver/dynamic_obstacle_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("patrol_state_topic", "/waver/patrol_state")
        self.declare_parameter("current_waypoint_topic", "/waver/current_waypoint")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")

        self.session_name = slugify(str(self.get_parameter("session_name").value))
        self.session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = f"{self.session_name}_{self.session_stamp}"
        output_dir = Path(os.path.expanduser(str(self.get_parameter("output_dir").value)))
        output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_dir / f"{self.session_id}.csv"
        self.summary_path = output_dir / f"{self.session_id}_summary.json"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()

        self.sample_period_s = float(self.get_parameter("sample_period_s").value)
        self.map_side_m = float(self.get_parameter("map_side_m").value)
        self.patrol_side_m = float(self.get_parameter("patrol_side_m").value)
        self.robot_collision_radius_m = float(
            self.get_parameter("robot_collision_radius_m").value
        )
        self.obstacle_collision_radius_m = float(
            self.get_parameter("obstacle_collision_radius_m").value
        )
        self.collision_vertical_threshold_m = float(
            self.get_parameter("collision_vertical_threshold_m").value
        )

        self.start_wall_time = time.time()
        self.start_mono = time.monotonic()
        self.tracking_start_iso = ""
        self.sample_index = 0
        self.rows_written = 0
        self.samples_with_obstacle = 0
        self.samples_with_bird_confirmed = 0
        self.first_detection_elapsed: Optional[float] = None
        self.max_obstacle_speed = 0.0
        self.min_robot_obstacle_xy = math.inf
        self.min_clearance_xy = math.inf
        self.collision_count = 0
        self.patrol_completed = False
        self.waypoint_arrival_count = 0
        self.reached_waypoints: list[str] = []

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0
        self.robot_path_length = 0.0
        self.last_robot_path_point: Optional[tuple[float, float]] = None
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.current_waypoint: Optional[tuple[float, float]] = None
        self.targets: list[tuple[float, float, float]] = []
        self.previous_target_samples: dict[str, tuple[float, float, float, float]] = {}
        self.patrol_state = "waiting"
        self.dynamic_obstacle_state = "waiting"
        self.target_class = "unknown"
        self.target_confidence = 0.0
        self.bird_confirmed = False
        self.scan_front_min = math.inf
        self.scan_valid_points = 0

        self.create_subscription(
            PoseArray,
            str(self.get_parameter("dynamic_targets_topic").value),
            self.targets_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("dynamic_obstacle_state_topic").value),
            self.dynamic_state_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_callback,
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("patrol_state_topic").value),
            self.patrol_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("current_waypoint_topic").value),
            self.current_waypoint_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_class_topic").value),
            self.target_class_callback,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("target_confidence_topic").value),
            self.target_confidence_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("bird_confirmed_topic").value),
            self.bird_confirmed_callback,
            10,
        )
        self.timer = self.create_timer(max(self.sample_period_s, 0.1), self.write_sample)
        self.get_logger().info(f"trial CSV logger started: {self.csv_path}")

    def targets_callback(self, msg: PoseArray) -> None:
        self.targets = [
            (float(p.position.x), float(p.position.y), float(p.position.z))
            for p in msg.poses
        ]
        if self.targets and not self.tracking_start_iso:
            self.tracking_start_iso = local_iso()

    def dynamic_state_callback(self, msg: String) -> None:
        self.dynamic_obstacle_state = msg.data

    def odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw_from_quaternion(
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vz = float(msg.twist.twist.linear.z)
        self.robot_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if self.last_robot_path_point is not None:
            px, py = self.last_robot_path_point
            step = math.hypot(x - px, y - py)
            if step < 1.0:
                self.robot_path_length += step
        self.last_robot_path_point = (x, y)

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    def scan_callback(self, msg: LaserScan) -> None:
        valid = 0
        front_values: list[float] = []
        angle = float(msg.angle_min)
        for value in msg.ranges:
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                valid += 1
                if abs(angle) <= math.radians(30.0):
                    front_values.append(float(value))
            angle += float(msg.angle_increment)
        self.scan_valid_points = valid
        self.scan_front_min = min(front_values) if front_values else math.inf

    def patrol_state_callback(self, msg: String) -> None:
        self.patrol_state = msg.data
        if msg.data.startswith("ARRIVED"):
            self.waypoint_arrival_count += 1
            if ":" in msg.data:
                self.reached_waypoints.append(msg.data.split(":", 1)[1].strip())
        if "COMPLETE" in msg.data:
            self.patrol_completed = True

    def current_waypoint_callback(self, msg: PoseStamped) -> None:
        self.current_waypoint = (float(msg.pose.position.x), float(msg.pose.position.y))

    def target_class_callback(self, msg: String) -> None:
        self.target_class = msg.data

    def target_confidence_callback(self, msg: Float32) -> None:
        self.target_confidence = float(msg.data)

    def bird_confirmed_callback(self, msg: Bool) -> None:
        self.bird_confirmed = bool(msg.data)
        if self.bird_confirmed and self.first_detection_elapsed is None:
            self.first_detection_elapsed = time.monotonic() - self.start_mono

    def obstacle_name(self, index: int, total: int) -> str:
        if total == 1 and "bird" in self.target_class.lower():
            return "bird_single"
        return f"dynamic_{index + 1}"

    def target_metrics(
        self,
        obstacle_id: str,
        target: tuple[float, float, float],
        now_mono: float,
    ) -> tuple[float, float, float, float, float]:
        x, y, z = target
        previous = self.previous_target_samples.get(obstacle_id)
        if previous is None:
            dx = dy = dz = 0.0
            speed = 0.0
            direction = 0.0
        else:
            prev_t, prev_x, prev_y, prev_z = previous
            dt = max(1e-6, now_mono - prev_t)
            dx = x - prev_x
            dy = y - prev_y
            dz = z - prev_z
            speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            direction = math.degrees(math.atan2(dy, dx)) if math.hypot(dx, dy) > 1e-6 else 0.0
        self.previous_target_samples[obstacle_id] = (now_mono, x, y, z)
        return dx, dy, dz, speed, direction

    def base_row(self, now_mono: float) -> dict[str, object]:
        elapsed = now_mono - self.start_mono
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        waypoint_x = self.current_waypoint[0] if self.current_waypoint else ""
        waypoint_y = self.current_waypoint[1] if self.current_waypoint else ""
        first_detection = (
            f"{self.first_detection_elapsed:.3f}"
            if self.first_detection_elapsed is not None
            else ""
        )
        return {
            "session_id": self.session_id,
            "sample_index": self.sample_index,
            "wall_time_iso": local_iso(),
            "ros_time_sec": f"{now_ros:.6f}",
            "elapsed_s": f"{elapsed:.3f}",
            "tracking_start_iso": self.tracking_start_iso,
            "map_side_m": f"{self.map_side_m:.2f}",
            "patrol_side_m": f"{self.patrol_side_m:.2f}",
            "patrol_state": self.patrol_state,
            "patrol_completed": self.patrol_completed,
            "waypoint_arrival_count": self.waypoint_arrival_count,
            "robot_x_m": f"{self.robot_x:.4f}",
            "robot_y_m": f"{self.robot_y:.4f}",
            "robot_yaw_rad": f"{self.robot_yaw:.4f}",
            "robot_speed_mps": f"{self.robot_speed:.4f}",
            "robot_path_length_m": f"{self.robot_path_length:.4f}",
            "cmd_linear_mps": f"{self.cmd_linear:.4f}",
            "cmd_angular_radps": f"{self.cmd_angular:.4f}",
            "current_waypoint_x_m": waypoint_x,
            "current_waypoint_y_m": waypoint_y,
            "obstacle_count": len(self.targets),
            "scan_front_min_m": "" if math.isinf(self.scan_front_min) else f"{self.scan_front_min:.4f}",
            "scan_valid_points": self.scan_valid_points,
            "bird_confirmed": self.bird_confirmed,
            "target_class": self.target_class,
            "target_confidence": f"{self.target_confidence:.4f}",
            "first_detection_elapsed_s": first_detection,
            "dynamic_obstacle_state": self.dynamic_obstacle_state,
        }

    def write_sample(self) -> None:
        now_mono = time.monotonic()
        self.sample_index += 1
        targets = list(self.targets)
        if targets:
            self.samples_with_obstacle += 1
        if self.bird_confirmed:
            self.samples_with_bird_confirmed += 1

        xy_ranges = [
            math.hypot(x - self.robot_x, y - self.robot_y)
            for x, y, _z in targets
        ]
        vertical_separations = [abs(z) for _x, _y, z in targets]
        min_xy = min(xy_ranges) if xy_ranges else math.inf
        min_vertical = min(vertical_separations) if vertical_separations else math.inf
        if min_xy < self.min_robot_obstacle_xy:
            self.min_robot_obstacle_xy = min_xy
        clearance = min_xy - self.robot_collision_radius_m - self.obstacle_collision_radius_m
        if not math.isinf(clearance) and clearance < self.min_clearance_xy:
            self.min_clearance_xy = clearance
        near_same_height = (
            not math.isinf(min_vertical)
            and min_vertical <= self.collision_vertical_threshold_m
        )
        overhead_pass_flag = bool(
            not math.isinf(clearance)
            and clearance <= 0.0
            and not near_same_height
        )
        collision_flag = bool(
            not math.isinf(clearance)
            and clearance <= 0.0
            and near_same_height
        )
        if collision_flag:
            self.collision_count += 1

        if not targets:
            row = self.base_row(now_mono)
            row.update(
                {
                    "obstacle_id": "",
                    "obstacle_x_m": "",
                    "obstacle_y_m": "",
                    "obstacle_z_m": "",
                    "obstacle_dx_1s_m": "",
                    "obstacle_dy_1s_m": "",
                    "obstacle_dz_1s_m": "",
                    "obstacle_speed_mps": "",
                    "obstacle_direction_deg": "",
                    "robot_to_obstacle_xy_m": "",
                    "robot_to_obstacle_3d_m": "",
                    "robot_to_obstacle_bearing_deg": "",
                    "min_robot_obstacle_xy_m": "",
                    "min_clearance_xy_m": "",
                    "min_vertical_separation_m": "",
                    "overhead_pass_flag": False,
                    "collision_flag": collision_flag,
                }
            )
            self.writer.writerow(row)
            self.rows_written += 1
            self.csv_file.flush()
            return

        for index, (x, y, z) in enumerate(targets):
            obstacle_id = self.obstacle_name(index, len(targets))
            dx, dy, dz, speed, direction = self.target_metrics(obstacle_id, (x, y, z), now_mono)
            self.max_obstacle_speed = max(self.max_obstacle_speed, speed)
            xy = math.hypot(x - self.robot_x, y - self.robot_y)
            dist3d = math.sqrt((x - self.robot_x) ** 2 + (y - self.robot_y) ** 2 + z * z)
            bearing = normalize_degrees(
                math.degrees(math.atan2(y - self.robot_y, x - self.robot_x) - self.robot_yaw)
            )
            row = self.base_row(now_mono)
            row.update(
                {
                    "obstacle_id": obstacle_id,
                    "obstacle_x_m": f"{x:.4f}",
                    "obstacle_y_m": f"{y:.4f}",
                    "obstacle_z_m": f"{z:.4f}",
                    "obstacle_dx_1s_m": f"{dx:.4f}",
                    "obstacle_dy_1s_m": f"{dy:.4f}",
                    "obstacle_dz_1s_m": f"{dz:.4f}",
                    "obstacle_speed_mps": f"{speed:.4f}",
                    "obstacle_direction_deg": f"{direction:.2f}",
                    "robot_to_obstacle_xy_m": f"{xy:.4f}",
                    "robot_to_obstacle_3d_m": f"{dist3d:.4f}",
                    "robot_to_obstacle_bearing_deg": f"{bearing:.2f}",
                    "min_robot_obstacle_xy_m": "" if math.isinf(min_xy) else f"{min_xy:.4f}",
                    "min_clearance_xy_m": "" if math.isinf(clearance) else f"{clearance:.4f}",
                    "min_vertical_separation_m": ""
                    if math.isinf(min_vertical)
                    else f"{min_vertical:.4f}",
                    "overhead_pass_flag": overhead_pass_flag,
                    "collision_flag": collision_flag,
                }
            )
            self.writer.writerow(row)
            self.rows_written += 1
        self.csv_file.flush()

    def write_summary(self) -> None:
        duration = time.monotonic() - self.start_mono
        summary = {
            "session_id": self.session_id,
            "csv_path": str(self.csv_path),
            "summary_path": str(self.summary_path),
            "start_time_iso": datetime.fromtimestamp(self.start_wall_time).astimezone().isoformat(timespec="milliseconds"),
            "end_time_iso": local_iso(),
            "duration_s": duration,
            "rows_written": self.rows_written,
            "sample_count": self.sample_index,
            "samples_with_obstacle": self.samples_with_obstacle,
            "samples_with_bird_confirmed": self.samples_with_bird_confirmed,
            "first_detection_elapsed_s": self.first_detection_elapsed,
            "patrol_completed": self.patrol_completed,
            "waypoint_arrival_count": self.waypoint_arrival_count,
            "reached_waypoints": self.reached_waypoints,
            "robot_path_length_m": self.robot_path_length,
            "max_obstacle_speed_mps": self.max_obstacle_speed,
            "min_robot_obstacle_xy_m": None
            if math.isinf(self.min_robot_obstacle_xy)
            else self.min_robot_obstacle_xy,
            "min_clearance_xy_m": None
            if math.isinf(self.min_clearance_xy)
            else self.min_clearance_xy,
            "collision_count": self.collision_count,
            "schema": self.FIELDNAMES,
            "paper_metric_groups": {
                "bird_detection": [
                    "target_class",
                    "target_confidence",
                    "bird_confirmed",
                    "first_detection_elapsed_s",
                ],
                "dynamic_obstacle_tracking": [
                    "obstacle_x_m",
                    "obstacle_y_m",
                    "obstacle_z_m",
                    "obstacle_dx_1s_m",
                    "obstacle_dy_1s_m",
                    "obstacle_dz_1s_m",
                    "obstacle_speed_mps",
                    "obstacle_direction_deg",
                ],
                "autonomous_patrol": [
                    "patrol_state",
                    "patrol_completed",
                    "waypoint_arrival_count",
                    "robot_path_length_m",
                    "robot_speed_mps",
                    "cmd_linear_mps",
                    "cmd_angular_radps",
                    "min_clearance_xy_m",
                    "min_vertical_separation_m",
                    "overhead_pass_flag",
                    "collision_flag",
                ],
            },
        }
        self.summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def close_files(self) -> None:
        if getattr(self, "csv_file", None) is None:
            return
        try:
            self.write_summary()
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            self.csv_file = None

    def destroy_node(self) -> bool:
        self.close_files()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboTrialDataLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Validate Waver operator-panel-visible topics and command safety.

This script is intentionally separate from the Tk UI.  It runs during Gazebo
or saved-map tests, watches the same topics the operator panel renders, and
writes a paper/test friendly CSV under ``~/ugv_ws/FSD_Vehicle/experiments_result``.
"""

from __future__ import annotations

import argparse
import csv
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Bool, String


FIELDS = [
    "time_sec",
    "trial_id",
    "map_received",
    "map_mode",
    "slam_live",
    "map_fixed",
    "robot_pose_visible",
    "robot_yaw_visible",
    "global_path_visible",
    "local_path_visible",
    "waypoint_visible",
    "active_goal_visible",
    "object_goal_visible",
    "elevated_target_visible",
    "mission_state_visible",
    "safety_state_visible",
    "camera_state_visible",
    "sound_state_visible",
    "ui_command_panel_alive",
    "ui_direct_cmd_vel_disabled",
    "manual_cmd_seen",
    "mission_command_seen",
    "operator_command_seen",
    "emergency_stop_seen",
    "cmd_vel_publishers",
    "overall_ui_success",
]


class UiVisualizationCheck(Node):
    def __init__(self, args: argparse.Namespace, run_dir: Path) -> None:
        super().__init__("waver_ui_visualization_check")
        self.args = args
        self.run_dir = run_dir
        self.start = time.monotonic()
        self.map_received = False
        self.robot_pose_visible = False
        self.robot_yaw_visible = False
        self.global_path_visible = False
        self.local_path_visible = False
        self.waypoint_visible = False
        self.active_goal_visible = False
        self.object_goal_visible = False
        self.elevated_target_visible = False
        self.mission_state_visible = False
        self.safety_state_visible = False
        self.camera_state_visible = False
        self.sound_state_visible = False
        self.manual_cmd_seen = False
        self.mission_command_seen = False
        self.operator_command_seen = False
        self.emergency_stop_seen = False

        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, args.amcl_pose_topic, self.amcl_cb, 10)
        self.create_subscription(NavPath, args.global_path_topic, self.global_path_cb, 10)
        self.create_subscription(NavPath, args.local_path_topic, self.local_path_cb, 10)
        self.create_subscription(PoseStamped, args.current_waypoint_topic, self.waypoint_cb, 10)
        self.create_subscription(PoseStamped, args.active_goal_topic, self.active_goal_cb, 10)
        self.create_subscription(PoseStamped, args.object_goal_topic, self.object_goal_cb, 10)
        self.create_subscription(PoseArray, args.elevated_target_topic, self.elevated_target_cb, 10)
        self.create_subscription(String, args.mission_state_topic, self.mission_state_cb, 10)
        self.create_subscription(String, args.safety_state_topic, self.safety_state_cb, 10)
        self.create_subscription(String, args.camera_state_topic, self.camera_state_cb, 10)
        self.create_subscription(String, args.sound_state_topic, self.sound_state_cb, 10)
        self.create_subscription(Twist, args.manual_cmd_topic, self.manual_cmd_cb, 10)
        self.create_subscription(String, args.mission_command_topic, self.mission_command_cb, 10)
        self.create_subscription(String, args.operator_command_topic, self.operator_command_cb, 10)
        self.create_subscription(Bool, args.emergency_stop_topic, self.estop_cb, 10)

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map_received = msg.info.width > 0 and msg.info.height > 0

    def odom_cb(self, _msg: Odometry) -> None:
        self.robot_pose_visible = True
        self.robot_yaw_visible = True

    def amcl_cb(self, _msg: PoseWithCovarianceStamped) -> None:
        self.robot_pose_visible = True
        self.robot_yaw_visible = True

    def global_path_cb(self, msg: NavPath) -> None:
        self.global_path_visible = self.global_path_visible or bool(msg.poses)

    def local_path_cb(self, msg: NavPath) -> None:
        self.local_path_visible = self.local_path_visible or bool(msg.poses)

    def waypoint_cb(self, _msg: PoseStamped) -> None:
        self.waypoint_visible = True

    def active_goal_cb(self, _msg: PoseStamped) -> None:
        self.active_goal_visible = True

    def object_goal_cb(self, _msg: PoseStamped) -> None:
        self.object_goal_visible = True

    def elevated_target_cb(self, msg: PoseArray) -> None:
        self.elevated_target_visible = self.elevated_target_visible or bool(msg.poses)

    def mission_state_cb(self, _msg: String) -> None:
        self.mission_state_visible = True

    def safety_state_cb(self, _msg: String) -> None:
        self.safety_state_visible = True

    def camera_state_cb(self, _msg: String) -> None:
        self.camera_state_visible = True

    def sound_state_cb(self, _msg: String) -> None:
        self.sound_state_visible = True

    def manual_cmd_cb(self, msg: Twist) -> None:
        self.manual_cmd_seen = self.manual_cmd_seen or abs(msg.linear.x) > 1e-5 or abs(msg.angular.z) > 1e-5

    def mission_command_cb(self, msg: String) -> None:
        self.mission_command_seen = self.mission_command_seen or bool(msg.data)

    def operator_command_cb(self, msg: String) -> None:
        self.operator_command_seen = self.operator_command_seen or bool(msg.data)

    def estop_cb(self, msg: Bool) -> None:
        self.emergency_stop_seen = self.emergency_stop_seen or bool(msg.data)

    def cmd_vel_publishers(self) -> list[str]:
        return sorted({info.node_name for info in self.get_publishers_info_by_topic("/cmd_vel")})

    def row(self) -> dict[str, object]:
        publishers = self.cmd_vel_publishers()
        direct_disabled = "waver_remote_panel" not in publishers
        map_mode = str(self.args.map_mode)
        slam_live = map_mode == "slam_live" and self.map_received
        map_fixed = map_mode in {"map_fixed", "auto"} and self.map_received
        command_ok = self.mission_command_seen or self.operator_command_seen or self.manual_cmd_seen
        # local path may be absent in rover-only Gazebo; global path + map + pose are the hard UI gate.
        overall = all(
            [
                self.map_received,
                self.robot_pose_visible,
                self.robot_yaw_visible,
                self.global_path_visible,
                self.mission_state_visible,
                self.safety_state_visible,
                direct_disabled,
            ]
        ) and (not self.args.require_command or command_ok)
        return {
            "time_sec": round(time.monotonic() - self.start, 3),
            "trial_id": self.args.trial_id,
            "map_received": self.map_received,
            "map_mode": map_mode,
            "slam_live": slam_live,
            "map_fixed": map_fixed,
            "robot_pose_visible": self.robot_pose_visible,
            "robot_yaw_visible": self.robot_yaw_visible,
            "global_path_visible": self.global_path_visible,
            "local_path_visible": self.local_path_visible,
            "waypoint_visible": self.waypoint_visible,
            "active_goal_visible": self.active_goal_visible,
            "object_goal_visible": self.object_goal_visible,
            "elevated_target_visible": self.elevated_target_visible,
            "mission_state_visible": self.mission_state_visible,
            "safety_state_visible": self.safety_state_visible,
            "camera_state_visible": self.camera_state_visible,
            "sound_state_visible": self.sound_state_visible,
            "ui_command_panel_alive": self.mission_command_seen or self.operator_command_seen or self.manual_cmd_seen,
            "ui_direct_cmd_vel_disabled": direct_disabled,
            "manual_cmd_seen": self.manual_cmd_seen,
            "mission_command_seen": self.mission_command_seen,
            "operator_command_seen": self.operator_command_seen,
            "emergency_stop_seen": self.emergency_stop_seen,
            "cmd_vel_publishers": ";".join(publishers),
            "overall_ui_success": overall,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write Waver operator UI visualization validation CSV")
    parser.add_argument("--output-root", default="~/ugv_ws/FSD_Vehicle/experiments_result")
    parser.add_argument("--experiment-name", default="")
    parser.add_argument("--duration-sec", type=float, default=12.0)
    parser.add_argument("--trial-id", default="U3")
    parser.add_argument("--map-mode", default="map_fixed", choices=["auto", "slam_live", "map_fixed"])
    parser.add_argument("--require-command", action="store_true")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--amcl-pose-topic", default="/amcl_pose")
    parser.add_argument("--global-path-topic", default="/plan")
    parser.add_argument("--local-path-topic", default="/local_plan")
    parser.add_argument("--current-waypoint-topic", default="/waver/current_waypoint")
    parser.add_argument("--active-goal-topic", default="/waver/active_nav_goal")
    parser.add_argument("--object-goal-topic", default="/waver/object_mission_goal")
    parser.add_argument("--elevated-target-topic", default="/waver/elevated_dynamic_targets")
    parser.add_argument("--mission-state-topic", default="/waver/mission_state")
    parser.add_argument("--safety-state-topic", default="/waver/safety_state")
    parser.add_argument("--camera-state-topic", default="/waver/classification_state")
    parser.add_argument("--sound-state-topic", default="/waver/sound_mission_status")
    parser.add_argument("--manual-cmd-topic", default="/waver/manual_cmd_vel")
    parser.add_argument("--mission-command-topic", default="/waver/mission_command")
    parser.add_argument("--operator-command-topic", default="/waver/operator_command")
    parser.add_argument("--emergency-stop-topic", default="/waver/emergency_stop")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = args.experiment_name or f"ui_validation_{stamp}"
    run_dir = Path(os.path.expanduser(args.output_root)) / name
    csv_dir = run_dir / "csv"
    reports_dir = run_dir / "reports"
    csv_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = UiVisualizationCheck(args, run_dir)
    deadline = time.monotonic() + max(args.duration_sec, 1.0)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    row = node.row()
    csv_path = csv_dir / "ui_visualization_check.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(row)
    summary_path = run_dir / "experiment_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(row)
    report_path = reports_dir / "ui_visualization_check.md"
    with report_path.open("w", encoding="utf-8") as stream:
        stream.write("# Waver Operator UI Validation\n\n")
        stream.write(f"- overall_ui_success: {row['overall_ui_success']}\n")
        stream.write(f"- map_received: {row['map_received']}\n")
        stream.write(f"- robot_pose_visible: {row['robot_pose_visible']}\n")
        stream.write(f"- global_path_visible: {row['global_path_visible']}\n")
        stream.write(f"- local_path_visible: {row['local_path_visible']}\n")
        stream.write(f"- ui_direct_cmd_vel_disabled: {row['ui_direct_cmd_vel_disabled']}\n")
        stream.write(f"- cmd_vel_publishers: {row['cmd_vel_publishers']}\n")
    print(f"UI_VISUALIZATION_OUTPUT {run_dir}")
    print(f"UI_VISUALIZATION_SUCCESS {row['overall_ui_success']}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if row["overall_ui_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

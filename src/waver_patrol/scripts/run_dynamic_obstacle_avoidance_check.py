#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import String


FIELDS = [
    "time_sec",
    "trial_id",
    "mission_command_sent",
    "dynamic_obstacle_seen",
    "avoidance_state_seen",
    "path_received",
    "path_detour_detected",
    "cmd_vel_nonzero",
    "odom_changed",
    "safety_state_seen",
    "safety_state_last",
    "overall_success",
    "failure_reason",
]


class DynamicObstacleAvoidanceCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("waver_dynamic_obstacle_avoidance_check")
        self.args = args
        self.start = time.monotonic()
        self.command_sent = False
        self.dynamic_obstacle_seen = False
        self.avoidance_state_seen = False
        self.path_received = False
        self.path_detour_detected = False
        self.cmd_vel_nonzero = False
        self.safety_state_seen = False
        self.safety_state_last = ""
        self.first_odom: tuple[float, float] | None = None
        self.last_odom: tuple[float, float] | None = None
        self.command_pub = self.create_publisher(String, args.mission_command_topic, 10)
        self.create_subscription(String, args.obstacle_state_topic, self.obstacle_cb, 10)
        self.create_subscription(String, args.sim_nav2_state_topic, self.sim_state_cb, 10)
        self.create_subscription(NavPath, args.global_path_topic, self.path_cb, 10)
        self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.create_subscription(String, args.safety_state_topic, self.safety_cb, 10)

    def maybe_start(self) -> None:
        if not self.command_sent and time.monotonic() - self.start >= self.args.start_after_sec:
            self.command_pub.publish(String(data="START_PATROL"))
            self.command_sent = True

    def obstacle_cb(self, msg: String) -> None:
        self.dynamic_obstacle_seen = self.dynamic_obstacle_seen or bool(msg.data)

    def sim_state_cb(self, msg: String) -> None:
        self.avoidance_state_seen = self.avoidance_state_seen or "AVOIDING" in msg.data

    def path_cb(self, msg: NavPath) -> None:
        self.path_received = self.path_received or bool(msg.poses)
        if len(msg.poses) < 3:
            return
        sx = msg.poses[0].pose.position.x
        sy = msg.poses[0].pose.position.y
        gx = msg.poses[-1].pose.position.x
        gy = msg.poses[-1].pose.position.y
        vx = gx - sx
        vy = gy - sy
        denom = max((vx * vx + vy * vy) ** 0.5, 1e-6)
        max_lateral = 0.0
        for pose in msg.poses[1:-1]:
            px = pose.pose.position.x - sx
            py = pose.pose.position.y - sy
            max_lateral = max(max_lateral, abs(vx * py - vy * px) / denom)
        self.path_detour_detected = self.path_detour_detected or max_lateral >= self.args.min_detour_lateral_m

    def cmd_cb(self, msg: Twist) -> None:
        self.cmd_vel_nonzero = self.cmd_vel_nonzero or abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4

    def odom_cb(self, msg: Odometry) -> None:
        pos = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if self.first_odom is None:
            self.first_odom = pos
        self.last_odom = pos

    def safety_cb(self, msg: String) -> None:
        self.safety_state_seen = True
        self.safety_state_last = msg.data

    def row(self) -> dict[str, object]:
        odom_changed = False
        if self.first_odom is not None and self.last_odom is not None:
            dx = self.last_odom[0] - self.first_odom[0]
            dy = self.last_odom[1] - self.first_odom[1]
            odom_changed = (dx * dx + dy * dy) ** 0.5 >= self.args.min_odom_delta_m
        overall = all(
            [
                self.command_sent,
                self.dynamic_obstacle_seen,
                self.avoidance_state_seen,
                self.path_received,
                self.path_detour_detected,
                self.cmd_vel_nonzero,
                odom_changed,
                self.safety_state_seen,
            ]
        )
        failures: list[str] = []
        for key, ok in [
            ("mission_command_not_sent", self.command_sent),
            ("dynamic_obstacle_missing", self.dynamic_obstacle_seen),
            ("avoidance_state_missing", self.avoidance_state_seen),
            ("path_missing", self.path_received),
            ("path_detour_missing", self.path_detour_detected),
            ("cmd_vel_nonzero_missing", self.cmd_vel_nonzero),
            ("odom_not_changed", odom_changed),
            ("safety_state_missing", self.safety_state_seen),
        ]:
            if not ok:
                failures.append(key)
        return {
            "time_sec": round(time.monotonic() - self.start, 3),
            "trial_id": self.args.trial_id,
            "mission_command_sent": self.command_sent,
            "dynamic_obstacle_seen": self.dynamic_obstacle_seen,
            "avoidance_state_seen": self.avoidance_state_seen,
            "path_received": self.path_received,
            "path_detour_detected": self.path_detour_detected,
            "cmd_vel_nonzero": self.cmd_vel_nonzero,
            "odom_changed": odom_changed,
            "safety_state_seen": self.safety_state_seen,
            "safety_state_last": self.safety_state_last,
            "overall_success": overall,
            "failure_reason": ";".join(failures),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate Gazebo dynamic obstacle detour smoke test")
    parser.add_argument("--output-root", default="~/ros2_ws5/FSD_Vehicle/experiments_result")
    parser.add_argument("--experiment-name", default="")
    parser.add_argument("--duration-sec", type=float, default=18.0)
    parser.add_argument("--start-after-sec", type=float, default=2.0)
    parser.add_argument("--trial-id", default="D1")
    parser.add_argument("--mission-command-topic", default="/waver/mission_command")
    parser.add_argument("--obstacle-state-topic", default="/waver/gazebo_dynamic_obstacle_state")
    parser.add_argument("--sim-nav2-state-topic", default="/waver/sim_nav2_state")
    parser.add_argument("--global-path-topic", default="/plan")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--safety-state-topic", default="/waver/safety_state")
    parser.add_argument("--min-detour-lateral-m", type=float, default=0.20)
    parser.add_argument("--min-odom-delta-m", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = args.experiment_name or f"dynamic_obstacle_avoidance_{stamp}"
    run_dir = Path(os.path.expanduser(args.output_root)) / name
    csv_dir = run_dir / "csv"
    reports_dir = run_dir / "reports"
    csv_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = DynamicObstacleAvoidanceCheck(args)
    deadline = time.monotonic() + max(args.duration_sec, 1.0)
    while rclpy.ok() and time.monotonic() < deadline:
        node.maybe_start()
        rclpy.spin_once(node, timeout_sec=0.1)
    row = node.row()
    for path in (csv_dir / "dynamic_obstacle_avoidance.csv", run_dir / "experiment_summary.csv"):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerow(row)
    with (reports_dir / "dynamic_obstacle_avoidance.md").open("w", encoding="utf-8") as stream:
        stream.write("# Dynamic Obstacle Avoidance Smoke Test\n\n")
        for key in FIELDS:
            stream.write(f"- {key}: {row[key]}\n")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(f"DYNAMIC_OBSTACLE_OUTPUT {run_dir}")
    print(f"DYNAMIC_OBSTACLE_SUCCESS {row['overall_success']}")
    return 0 if row["overall_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


COUNT_RE = re.compile(r"removed_count=(\d+)")


@dataclass
class TrialState:
    removed_count: int = 0
    lap_count: int = 0
    dynamic_target_seen: bool = False
    lock_seen: bool = False
    sound_done_seen: bool = False
    return_or_resume_seen: bool = False
    removal_goal_seen: bool = False
    last_removal_state: str = ""
    last_mission_state: str = ""


class BirdPatrolTrialValidator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("waver_bird_patrol_trial_validator")
        self.args = args
        self.state = TrialState()
        self.start_wall = self.get_clock().now().nanoseconds * 1e-9
        self.command_sent = False
        self.done = False
        self.success = False

        self.command_pub = self.create_publisher(String, "/waver/mission_command", 10)
        self.mode_pub = self.create_publisher(String, "/waver/mode_cmd", 10)
        self.create_subscription(String, "/waver/gazebo_bird_removal_state", self.removal_callback, 10)
        self.create_subscription(Int32, "/waver/patrol_lap_count", self.lap_callback, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_callback, 10)
        self.create_subscription(Bool, "/waver/dynamic_object_lock", self.lock_callback, 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.sound_callback, 10)
        self.create_subscription(String, "/waver/lidar_tracking_state", self.lidar_callback, 10)
        self.create_subscription(String, "/waver/dynamic_obstacle_state", self.lidar_callback, 10)
        self.create_timer(0.2, self.tick)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def removal_callback(self, msg: String) -> None:
        text = msg.data.strip()
        self.state.last_removal_state = text
        match = COUNT_RE.search(text)
        if match:
            self.state.removed_count = max(self.state.removed_count, int(match.group(1)))
        if text.startswith("REMOVAL_GOAL_REACHED"):
            self.state.removal_goal_seen = True

    def lap_callback(self, msg: Int32) -> None:
        self.state.lap_count = max(self.state.lap_count, int(msg.data))

    def mission_callback(self, msg: String) -> None:
        self.state.last_mission_state = msg.data
        upper = msg.data.upper()
        if "RETURN" in upper or "RESUME" in upper or "PATROL_NAVIGATING" in upper:
            self.state.return_or_resume_seen = True

    def lock_callback(self, msg: Bool) -> None:
        self.state.lock_seen = self.state.lock_seen or bool(msg.data)

    def sound_callback(self, msg: Bool) -> None:
        self.state.sound_done_seen = self.state.sound_done_seen or bool(msg.data)

    def lidar_callback(self, msg: String) -> None:
        upper = msg.data.upper()
        if "TARGET" in upper or "DYNAMIC" in upper or "LOCK" in upper:
            self.state.dynamic_target_seen = True

    def tick(self) -> None:
        elapsed = self.now_sec() - self.start_wall
        if not self.command_sent and elapsed >= self.args.start_delay_sec:
            self.mode_pub.publish(String(data="AUTO"))
            self.command_pub.publish(String(data="START_PATROL"))
            self.command_sent = True
            self.get_logger().info("sent AUTO + START_PATROL")

        required_ok = (
            self.state.removed_count >= self.args.required_removed
            and self.state.lap_count >= self.args.required_laps
            and self.state.dynamic_target_seen
            and self.state.lock_seen
            and self.state.sound_done_seen
            and self.state.return_or_resume_seen
        )
        if required_ok:
            self.success = True
            self.done = True
            self.get_logger().info(
                "PASS removed=%d laps=%d last_removal='%s' mission='%s'"
                % (
                    self.state.removed_count,
                    self.state.lap_count,
                    self.state.last_removal_state,
                    self.state.last_mission_state,
                )
            )
            return
        if elapsed >= self.args.timeout_sec:
            self.success = False
            self.done = True
            self.get_logger().error(
                "FAIL timeout removed=%d/%d laps=%d/%d dynamic=%s lock=%s sound=%s return=%s last_removal='%s' mission='%s'"
                % (
                    self.state.removed_count,
                    self.args.required_removed,
                    self.state.lap_count,
                    self.args.required_laps,
                    self.state.dynamic_target_seen,
                    self.state.lock_seen,
                    self.state.sound_done_seen,
                    self.state.return_or_resume_seen,
                    self.state.last_removal_state,
                    self.state.last_mission_state,
                )
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--required-removed", type=int, default=5)
    parser.add_argument("--required-laps", type=int, default=2)
    parser.add_argument("--timeout-sec", type=float, default=420.0)
    parser.add_argument("--start-delay-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BirdPatrolTrialValidator(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

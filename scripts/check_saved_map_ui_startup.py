#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def run(command: list[str], timeout: float = 8.0) -> str:
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        return str(stdout) + str(stderr) + "\nTIMEOUT\n"
    return (result.stdout or "") + (result.stderr or "")


def topic_publishers(topic: str) -> tuple[int, list[str]]:
    text = run(["ros2", "topic", "info", "-v", topic], timeout=8.0)
    count_match = re.search(r"Publisher count:\s*(\d+)", text)
    count = int(count_match.group(1)) if count_match else 0
    nodes = [
        match.group(1).strip()
        for match in re.finditer(
            r"Node name:\s*([^\n]+)(?:(?!\nNode name:).)*?Endpoint type:\s*PUBLISHER",
            text,
            flags=re.DOTALL,
        )
    ]
    return count, nodes


class SavedMapUiStartupCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("saved_map_ui_startup_check")
        self.args = args
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_msg: OccupancyGrid | None = None
        self.current_map_source = ""
        self.map_apply_state = ""
        self.mapping_state = ""
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, qos)
        self.create_subscription(String, args.current_map_source_topic, self.source_callback, qos)
        self.create_subscription(String, args.map_apply_state_topic, self.apply_callback, qos)
        self.create_subscription(String, args.mapping_state_topic, self.mapping_callback, qos)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def source_callback(self, msg: String) -> None:
        self.current_map_source = msg.data.strip()

    def apply_callback(self, msg: String) -> None:
        self.map_apply_state = msg.data.strip()

    def mapping_callback(self, msg: String) -> None:
        self.mapping_state = msg.data.strip()

    def spin_until_ready(self) -> bool:
        deadline = time.monotonic() + self.args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ready():
                return True
        return self.ready()

    def map_ok(self) -> bool:
        if self.map_msg is None:
            return False
        return int(self.map_msg.info.width) >= self.args.min_width and int(self.map_msg.info.height) >= self.args.min_height

    def source_ok(self) -> bool:
        return self.current_map_source.upper() in {
            "STATIC_MAP",
            "FIXED_MAP_READY",
            "SAVED_SLAM_MAP",
            "MAP_FIXED",
        }

    def ready(self) -> bool:
        return (
            self.map_ok()
            and self.source_ok()
            and "MAP_FIXED_READY" in self.map_apply_state.upper()
            and self.mapping_state.upper().startswith("STATIC_MAP_READY")
        )

    def map_stats(self) -> str:
        if self.map_msg is None:
            return "none"
        data = list(self.map_msg.data)
        occupied = sum(1 for value in data if int(value) > 50)
        free = sum(1 for value in data if 0 <= int(value) <= 50)
        unknown = sum(1 for value in data if int(value) < 0)
        total = max(1, len(data))
        known_ratio = (occupied + free) / float(total)
        return (
            f"width={int(self.map_msg.info.width)} height={int(self.map_msg.info.height)} "
            f"occupied={occupied} free={free} unknown={unknown} known_ratio={known_ratio:.4f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--current-map-source-topic", default="/waver/current_map_source")
    parser.add_argument("--map-apply-state-topic", default="/waver/map_apply_state")
    parser.add_argument("--mapping-state-topic", default="/waver/mapping_state")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--min-width", type=int, default=10)
    parser.add_argument("--min-height", type=int, default=10)
    parser.add_argument("--require-map-publisher-count", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = SavedMapUiStartupCheck(args)
    try:
        ready = node.spin_until_ready()
        map_count, map_nodes = topic_publishers(args.map_topic)
        remote_count, remote_nodes = topic_publishers("/waver/manual_cmd_vel")

        print("SAVED_MAP_UI_STARTUP_CHECK")
        print(f"map_publishers={map_count} nodes={map_nodes}")
        print(f"manual_cmd_publishers={remote_count} nodes={remote_nodes}")
        print(f"current_map_source={node.current_map_source or 'NONE'}")
        print(f"map_apply_state={node.map_apply_state or 'NONE'}")
        print(f"mapping_state={node.mapping_state or 'NONE'}")
        print(f"map_stats={node.map_stats()}")
        print(f"RESULT={'PASS' if ready and map_count == args.require_map_publisher_count else 'FAIL'}")

        if map_count != args.require_map_publisher_count:
            return 1
        if not ready:
            return 1
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

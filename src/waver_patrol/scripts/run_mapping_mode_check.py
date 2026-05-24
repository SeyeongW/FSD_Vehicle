#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


FIELDS = [
    "time_sec",
    "trial_id",
    "map_messages",
    "first_known_ratio",
    "last_known_ratio",
    "map_changed",
    "save_command_sent",
    "map_saved",
    "map_applied",
    "saved_map_yaml",
    "overall_success",
    "failure_reason",
]


class MappingModeCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("waver_mapping_mode_check")
        self.args = args
        self.start = time.monotonic()
        self.known_ratios: list[float] = []
        self.saved_path = ""
        self.map_saved = False
        self.map_applied = False
        self.save_sent = False
        self.command_pub = self.create_publisher(String, args.mission_command_topic, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, 10)
        self.create_subscription(String, args.map_apply_state_topic, self.apply_cb, 10)
        self.create_subscription(String, args.map_saved_path_topic, self.saved_path_cb, 10)

    def map_cb(self, msg: OccupancyGrid) -> None:
        total = max(len(msg.data), 1)
        known = sum(1 for value in msg.data if value >= 0)
        self.known_ratios.append(known / total)

    def apply_cb(self, msg: String) -> None:
        text = msg.data
        self.map_saved = self.map_saved or "MAP_SAVED" in text
        self.map_applied = self.map_applied or "MAP_APPLIED" in text

    def saved_path_cb(self, msg: String) -> None:
        if msg.data:
            self.saved_path = msg.data
            self.map_saved = True

    def maybe_send_commands(self) -> None:
        elapsed = time.monotonic() - self.start
        if elapsed < 0.5:
            self.command_pub.publish(String(data="START_MAPPING"))
        if (not self.save_sent) and elapsed >= self.args.save_after_sec:
            self.command_pub.publish(String(data="SAVE_MAP"))
            self.save_sent = True

    def row(self) -> dict[str, object]:
        first = self.known_ratios[0] if self.known_ratios else 0.0
        last = self.known_ratios[-1] if self.known_ratios else 0.0
        changed = len(self.known_ratios) >= 2 and max(self.known_ratios) - min(self.known_ratios) >= 0.05
        saved_file_ok = bool(self.saved_path) and Path(os.path.expanduser(self.saved_path)).is_file()
        overall = len(self.known_ratios) >= 2 and changed and self.save_sent and self.map_saved and self.map_applied and saved_file_ok
        failures: list[str] = []
        if len(self.known_ratios) < 2:
            failures.append("map_messages_insufficient")
        if not changed:
            failures.append("map_not_changed")
        if not self.save_sent:
            failures.append("save_command_not_sent")
        if not self.map_saved:
            failures.append("map_saved_state_missing")
        if not self.map_applied:
            failures.append("map_applied_state_missing")
        if not saved_file_ok:
            failures.append("saved_map_yaml_missing")
        return {
            "time_sec": round(time.monotonic() - self.start, 3),
            "trial_id": self.args.trial_id,
            "map_messages": len(self.known_ratios),
            "first_known_ratio": round(first, 4),
            "last_known_ratio": round(last, 4),
            "map_changed": changed,
            "save_command_sent": self.save_sent,
            "map_saved": self.map_saved,
            "map_applied": self.map_applied,
            "saved_map_yaml": self.saved_path,
            "overall_success": overall,
            "failure_reason": ";".join(failures),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate Gazebo mapping mode /map update and save/apply workflow")
    parser.add_argument("--output-root", default="~/ros2_ws/experiments_result")
    parser.add_argument("--experiment-name", default="")
    parser.add_argument("--duration-sec", type=float, default=14.0)
    parser.add_argument("--save-after-sec", type=float, default=7.0)
    parser.add_argument("--trial-id", default="M1")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--mission-command-topic", default="/waver/mission_command")
    parser.add_argument("--map-apply-state-topic", default="/waver/map_apply_state")
    parser.add_argument("--map-saved-path-topic", default="/waver/map_saved_path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = args.experiment_name or f"mapping_mode_{stamp}"
    run_dir = Path(os.path.expanduser(args.output_root)) / name
    csv_dir = run_dir / "csv"
    reports_dir = run_dir / "reports"
    csv_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MappingModeCheck(args)
    deadline = time.monotonic() + max(args.duration_sec, 1.0)
    while rclpy.ok() and time.monotonic() < deadline:
        node.maybe_send_commands()
        rclpy.spin_once(node, timeout_sec=0.1)
    row = node.row()
    for path in (csv_dir / "mapping_mode_check.csv", run_dir / "experiment_summary.csv"):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerow(row)
    with (reports_dir / "mapping_mode_check.md").open("w", encoding="utf-8") as stream:
        stream.write("# Gazebo Mapping Mode Check\n\n")
        for key in FIELDS:
            stream.write(f"- {key}: {row[key]}\n")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(f"MAPPING_MODE_OUTPUT {run_dir}")
    print(f"MAPPING_MODE_SUCCESS {row['overall_success']}")
    return 0 if row["overall_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

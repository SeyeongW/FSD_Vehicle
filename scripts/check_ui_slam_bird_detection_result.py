#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import pathlib
import sys
from typing import Any


REQUIRED_TRUE_FIELDS = [
    "current_map_source_slam_live",
    "bird_topics_visible",
    "bird_topics_fresh",
    "bird_detector_state_fresh",
    "bird_fusion_state_fresh",
    "mapping_path_visible",
    "no_patrol_conflict",
    "no_patrol_emergency_stop",
    "no_target_approach_without_arm",
    "no_sound_without_arm",
    "map_quality_pass",
]


def load_report(path: pathlib.Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"report not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report",
        default="reports/ui_slam_bird_detection/latest.json",
        help="Combined UI SLAM + bird detection smoke JSON report.",
    )
    parser.add_argument("--require-map-publishers", type=int, default=1)
    parser.add_argument("--require-cmd-vel-publishers", type=int, default=1)
    parser.add_argument("--require-mode-publishers", type=int, default=1)
    args = parser.parse_args()

    report_path = pathlib.Path(args.report).expanduser()
    report = load_report(report_path)

    failures: list[str] = []
    for field in REQUIRED_TRUE_FIELDS:
        if not bool(report.get(field, False)):
            failures.append(field)

    publisher_expectations = [
        ("map_publisher_count", args.require_map_publishers),
        ("cmd_vel_publisher_count", args.require_cmd_vel_publishers),
        ("mode_publisher_count", args.require_mode_publishers),
    ]
    for field, expected in publisher_expectations:
        value = int(report.get(field, -1))
        if value != expected:
            failures.append(f"{field}_expected_{expected}_got_{value}")

    result = "PASS" if not failures else "FAIL"
    print("UI_SLAM_BIRD_DETECTION_CHECK")
    print(f"report={report_path}")
    for field in REQUIRED_TRUE_FIELDS:
        print(f"{field}={report.get(field)}")
    for field, _expected in publisher_expectations:
        print(f"{field}={report.get(field)} nodes={report.get(field.replace('_count', '_nodes'), [])}")
    print(f"RESULT={result}" + ("" if not failures else " failures=" + ",".join(failures)))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())

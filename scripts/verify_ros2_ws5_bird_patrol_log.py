#!/usr/bin/env python3
"""Verify the ros2_ws5 Gazebo bird patrol mechanism event log.

This checker is intentionally log-based so a completed Gazebo run can be audited
without relying on memory or screenshots. It validates the high-level mechanism
required by CODEX_FINAL_PROMPT_ROS2_WS5_BIRD_PATROL.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re
import sys


REQUIRED_SEQUENCE = [
    ("mission_event", "MISSION_COMMAND reason=START_PATROL"),
    ("mission_state", "PATROL_NAVIGATING"),
    ("dynamic_target", "count="),
    ("dynamic_lock", "True"),
    ("inspection_goal", "x="),
    ("mission_state", "APPROACH_TARGET_OFFSET"),
    ("mission_state", "TARGET_REACHED"),
    ("mission_state", "CAMERA_ALIGN_DONE"),
    ("classification", ("GAZEBO_FAKE_YOLO11S_CLASSIFICATION", "class=bird")),
    ("bird_confirmed", "True"),
    ("mission_state", "SOUND_TASK_DONE"),
    ("mission_state", "RETURN_TO_INTERRUPTED_WAYPOINT"),
    ("mission_state", "RESUME_PATROL"),
    ("mission_state", "PATROL_NAVIGATING"),
]

REMOVAL_SEQUENCE = [
    ("mission_event", "MISSION_COMMAND reason=START_PATROL"),
    ("mission_state", "PATROL_NAVIGATING"),
    ("dynamic_target", "count="),
    ("dynamic_lock", "True"),
    ("inspection_goal", "x="),
    ("mission_state", "APPROACH_TARGET_OFFSET"),
    ("classification", ("GAZEBO_FAKE_YOLO11S_CLASSIFICATION", "class=bird")),
    ("bird_confirmed", "True"),
    ("bird_removed", "removed_count=1"),
    ("bird_removed", "removed_count=2"),
    ("mission_state", "RETURN_TO_INTERRUPTED_WAYPOINT"),
    ("mission_state", "RESUME_PATROL"),
    ("mission_state", "PATROL_NAVIGATING"),
]


def row_text(row: dict[str, str]) -> str:
    return " ".join(row.get(k, "") for k in ("event", "detail", "mission_state", "mode"))


def token_matches(text: str, token: str | tuple[str, ...]) -> bool:
    if isinstance(token, tuple):
        return all(part in text for part in token)
    return token in text


def token_label(token: str | tuple[str, ...]) -> str:
    if isinstance(token, tuple):
        return "&".join(token)
    return token


def find_in_order(rows: list[dict[str, str]], required_sequence: list[tuple[str, str | tuple[str, ...]]]) -> tuple[bool, list[str]]:
    missing: list[str] = []
    cursor = 0
    for event, token in required_sequence:
        found_at = None
        for idx in range(cursor, len(rows)):
            row = rows[idx]
            if row.get("event") == event and token_matches(row_text(row), token):
                found_at = idx
                break
        if found_at is None:
            missing.append(f"{event}:{token_label(token)}")
        else:
            cursor = found_at + 1
    return not missing, missing


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="mechanism_events.csv path")
    parser.add_argument("--min-removed-birds", type=int, default=0)
    args = parser.parse_args()

    if not args.log.exists():
        print(f"MECHANISM_LOG_VERIFY=FAIL reason=missing_file path={args.log}")
        return 2

    with args.log.open(newline="", encoding="utf-8", errors="replace") as f:
        rows = list(csv.DictReader(f))

    required_sequence = REMOVAL_SEQUENCE if args.min_removed_birds > 0 else REQUIRED_SEQUENCE
    ok_sequence, missing = find_in_order(rows, required_sequence)
    event_counts: dict[str, int] = {}
    removed_birds: set[str] = set()
    for row in rows:
        event_counts[row.get("event", "")] = event_counts.get(row.get("event", ""), 0) + 1
        if row.get("event") == "bird_removed":
            match = re.search(r"bird=([^\s]+)", row.get("detail", ""))
            if match:
                removed_birds.add(match.group(1))

    # Extra sanity checks for command authority evidence captured in the log.
    nonzero_cmd_rows = [
        row for row in rows
        if abs(float(row.get("cmd_linear") or 0.0)) > 1e-6
        or abs(float(row.get("cmd_angular") or 0.0)) > 1e-6
    ]
    odom_progress = False
    odom_x_values = []
    odom_y_values = []
    for row in rows:
        try:
            odom_x_values.append(float(row.get("odom_x") or 0.0))
            odom_y_values.append(float(row.get("odom_y") or 0.0))
        except ValueError:
            pass
    if odom_x_values and odom_y_values:
        odom_progress = (max(odom_x_values) - min(odom_x_values)) > 0.5 or (max(odom_y_values) - min(odom_y_values)) > 0.5

    removal_ok = len(removed_birds) >= max(0, args.min_removed_birds)
    pass_all = ok_sequence and bool(nonzero_cmd_rows) and odom_progress and removal_ok
    if pass_all:
        print(
            "MECHANISM_LOG_VERIFY=PASS "
            f"rows={len(rows)} "
            f"nonzero_cmd_rows={len(nonzero_cmd_rows)} "
            f"odom_dx={max(odom_x_values)-min(odom_x_values):.3f} "
            f"odom_dy={max(odom_y_values)-min(odom_y_values):.3f} "
            f"removed_birds={len(removed_birds)}"
        )
        for event in sorted(event_counts):
            if event:
                print(f"EVENT_COUNT {event}={event_counts[event]}")
        return 0

    print(
        "MECHANISM_LOG_VERIFY=FAIL "
        f"missing={missing} "
        f"nonzero_cmd_rows={len(nonzero_cmd_rows)} "
        f"odom_progress={odom_progress} "
        f"removed_birds={len(removed_birds)} "
        f"min_removed_birds={args.min_removed_birds}"
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())

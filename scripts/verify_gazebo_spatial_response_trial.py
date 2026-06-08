#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def number(value: Any) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else math.nan
    except Exception:
        return math.nan


def truth(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass", "ok"}


def require(condition: bool, name: str, failures: list[str], details: str = "") -> None:
    if condition:
        print(f"PASS {name}{(' ' + details) if details else ''}")
    else:
        failures.append(name if not details else f"{name}: {details}")
        print(f"FAIL {name}{(' ' + details) if details else ''}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--min-removed-birds", type=int, default=2)
    parser.add_argument("--max-gt-leakage", type=int, default=0)
    parser.add_argument("--require-odom-motion", action="store_true", default=True)
    args = parser.parse_args()

    run_dir = args.run_dir.expanduser().resolve()
    metrics_path = run_dir / "metrics" / "spatial_response_metrics.json"
    metrics = load_json(metrics_path)
    logs = run_dir / "logs"
    lidar = rows(logs / "lidar_filter_response.csv")
    goals = rows(logs / "goal_bird_distance_events.csv")
    birds = rows(logs / "bird_kinematics.csv")
    latency = rows(logs / "lidar_waver_latency_events.csv")

    failures: list[str] = []
    require(metrics_path.exists(), "metrics_json_exists", failures, str(metrics_path))
    require(truth(metrics.get("mechanism.lidar_only_decision_clean")), "lidar_only_decision_clean", failures)
    require(int(number(metrics.get("mechanism.gt_leakage_in_lidar_mode_count")) or 0) <= args.max_gt_leakage, "gt_leakage_zero", failures)
    require(int(number(metrics.get("mechanism.removed_bird_count")) or 0) >= args.min_removed_birds, "removed_bird_count", failures)
    require(truth(metrics.get("mechanism.has_lidar_target_ok")), "has_lidar_target_ok", failures)
    require(truth(metrics.get("mechanism.has_dynamic_lock")), "has_dynamic_lock", failures)
    require(truth(metrics.get("mechanism.has_object_mission_goal")), "has_object_mission_goal", failures)
    require(truth(metrics.get("mechanism.has_active_target_nav_goal")), "has_active_target_nav_goal", failures)
    require(truth(metrics.get("mechanism.has_nonzero_cmd_vel_after_target_goal")), "has_target_cmd_vel", failures)
    if args.require_odom_motion:
        require(truth(metrics.get("mechanism.has_odom_motion_after_target_goal")), "has_odom_motion_after_target_goal", failures)
    require(truth(metrics.get("mechanism.has_return_resume")), "has_return_resume", failures)

    runtime = number(metrics.get("lidar.filter_runtime_wall_mean_ms"))
    require(math.isfinite(runtime) and runtime > 0.0, "lidar_wall_runtime_positive", failures, f"{runtime:.3f} ms" if math.isfinite(runtime) else "")
    target_goal_mean = number(metrics.get("spatial.target_goal_to_bird_xy_mean_m"))
    require(math.isfinite(target_goal_mean) and target_goal_mean > 0.1, "target_goal_distance_metric_present", failures)

    target_goal_rows = [
        row
        for row in goals
        if str(row.get("is_target_related_goal", "")).lower() == "true"
        or str(row.get("goal_role", "")).upper() == "TARGET_INSPECTION"
    ]
    require(bool(target_goal_rows), "target_only_goal_rows_present", failures)
    require(any(str(row.get("target_ok", "")).lower() == "true" for row in lidar), "lidar_target_ok_rows_present", failures)
    require(any(str(row.get("removed", "")).lower() == "true" for row in birds), "bird_removed_rows_present", failures)
    require(bool(latency), "latency_csv_present", failures)

    print(f"RUN_DIR={run_dir}")
    if failures:
        print("VERIFY_GAZEBO_SPATIAL_RESPONSE=FAIL")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

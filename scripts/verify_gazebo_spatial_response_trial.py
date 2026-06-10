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


def finite_metric(metrics: dict[str, Any], key: str) -> bool:
    value = number(metrics.get(key))
    return math.isfinite(value)


def metric_int(metrics: dict[str, Any], key: str, default: int = 0) -> int:
    value = number(metrics.get(key))
    return int(value) if math.isfinite(value) else default


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--mode", choices=["spatial", "full"], default="spatial")
    parser.add_argument("--min-removed-birds", type=int, default=2)
    parser.add_argument("--max-gt-leakage", type=int, default=0)
    parser.add_argument("--require-odom-motion", action="store_true", default=True)
    parser.add_argument("--require-return-resume", action="store_true", default=False)
    args = parser.parse_args()
    if args.mode == "full":
        args.require_return_resume = True

    run_dir = args.run_dir.expanduser().resolve()
    metrics_path = run_dir / "metrics" / "spatial_response_metrics.json"
    metrics = load_json(metrics_path)
    logs = run_dir / "logs"
    spatial = rows(logs / "spatial_distance_timeseries.csv")
    lidar = rows(logs / "lidar_filter_response.csv")
    goals = rows(logs / "goal_bird_distance_events.csv")
    birds = rows(logs / "bird_kinematics.csv")
    latency = rows(logs / "lidar_waver_latency_events.csv")
    mechanism = rows(logs / "mechanism_events.csv")

    failures: list[str] = []
    require(metrics_path.exists(), "metrics_json_exists", failures, str(metrics_path))
    for rel in [
        "logs/spatial_distance_timeseries.csv",
        "logs/goal_bird_distance_events.csv",
        "logs/bird_kinematics.csv",
        "logs/lidar_filter_response.csv",
        "logs/lidar_waver_latency_events.csv",
        "logs/mechanism_events.csv",
    ]:
        require((run_dir / rel).exists(), f"{rel}_exists", failures)
    require(str(metrics.get("mechanism.detector_mode", "")).lower() == "lidar", "detector_mode_lidar", failures)
    require(truth(metrics.get("mechanism.lidar_only_decision_clean")), "lidar_only_decision_clean", failures)
    require(metric_int(metrics, "mechanism.gt_leakage_in_lidar_mode_count") <= args.max_gt_leakage, "gt_leakage_zero", failures)
    require(metric_int(metrics, "mechanism.removed_bird_count") >= args.min_removed_birds, "removed_bird_count", failures)
    require(truth(metrics.get("mechanism.has_lidar_target_ok")), "has_lidar_target_ok", failures)
    require(truth(metrics.get("mechanism.has_dynamic_lock")), "has_dynamic_lock", failures)
    require(truth(metrics.get("mechanism.has_object_mission_goal")), "has_object_mission_goal", failures)
    require(truth(metrics.get("mechanism.has_active_target_nav_goal")), "has_active_target_nav_goal", failures)
    require(truth(metrics.get("mechanism.has_nonzero_cmd_vel_after_target_goal")), "has_target_cmd_vel", failures)
    if args.mode == "full":
        require(truth(metrics.get("mechanism.has_patrol_preempt_to_target_goal")), "patrol_preempt_to_target_goal", failures)
    if args.require_odom_motion:
        require(truth(metrics.get("mechanism.has_odom_motion_after_target_goal")), "has_odom_motion_after_target_goal", failures)
    if args.require_return_resume:
        require(truth(metrics.get("mechanism.return_resume_sequence_success")), "return_resume_sequence_success", failures)

    runtime = number(metrics.get("lidar.filter_runtime_wall_mean_ms"))
    require(math.isfinite(runtime) and runtime > 0.0, "lidar_wall_runtime_positive", failures, f"{runtime:.3f} ms" if math.isfinite(runtime) else "")
    require(finite_metric(metrics, "spatial.target_goal_to_bird_xy_first_m"), "target_goal_to_bird_xy_first_finite", failures)
    require(finite_metric(metrics, "spatial.target_goal_to_lidar_target_xy_first_m"), "target_goal_to_lidar_target_xy_first_finite", failures)
    valid_count = metric_int(metrics, "spatial.valid_lidar_target_sample_count")
    require(valid_count > 0, "valid_lidar_target_sample_count_positive", failures, str(valid_count))
    require(finite_metric(metrics, "spatial.valid_lidar_target_to_bird_xy_mean_m"), "valid_lidar_target_to_bird_xy_mean_finite", failures)
    require(number(metrics.get("navigation.odom_path_length_m")) > 0.5, "odom_path_length_gt_0p5m", failures)
    require(truth(metrics.get("safety.cmd_vel_safety_mux_sole_publisher")), "cmd_vel_safety_mux_sole_publisher", failures)
    require((logs / "cmd_vel_topic_info.txt").exists(), "cmd_vel_topic_info_exists", failures)
    require(finite_metric(metrics, "latency.active_target_nav_goal_to_cmd_vel_ms"), "active_target_nav_goal_to_cmd_vel_finite", failures)
    require(finite_metric(metrics, "latency.active_target_nav_goal_to_odom_motion_ms"), "active_target_nav_goal_to_odom_motion_finite", failures)

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
    require(bool(spatial), "spatial_timeseries_rows_present", failures)
    require(bool(mechanism), "mechanism_events_rows_present", failures)

    print(f"RUN_DIR={run_dir}")
    if failures:
        print("VERIFY_GAZEBO_SPATIAL_RESPONSE=FAIL")
        print(f"missing_or_failed={failures}")
        return 1
    print("VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

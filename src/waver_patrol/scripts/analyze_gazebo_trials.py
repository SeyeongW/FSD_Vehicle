#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path


def read_latest_summaries(input_dir: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    patterns = [
        "gazebo_trial_*/experiment_summary.csv",
        "pre_real_height_*/experiment_summary.csv",
        "pre_real_gazebo_trial_*/experiment_summary.csv",
    ]
    summaries = []
    for pattern in patterns:
        summaries.extend(input_dir.glob(pattern))
    for summary in sorted(set(summaries)):
        with summary.open(newline="", encoding="utf-8") as f:
            data = list(csv.DictReader(f))
        if data:
            row = data[-1]
            row["summary_path"] = str(summary)
            rows.append(row)
    return rows


def as_float(value: str, default: float = math.nan) -> float:
    try:
        return float(value)
    except Exception:
        return default


def write_csv(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze Waver Gazebo moving-object trial CSV output")
    parser.add_argument("--input_dir", default="~/ros2_ws/experiments_result")
    parser.add_argument("--output_dir", default="~/ros2_ws/experiments_result/results")
    args = parser.parse_args()
    input_dir = Path(os.path.expanduser(args.input_dir))
    output_dir = Path(os.path.expanduser(args.output_dir))
    rows = read_latest_summaries(input_dir)
    normalized: list[dict[str, object]] = []
    for row in rows:
        dynamic_motion_m = row.get("dynamic_motion_m", row.get("target_motion_distance_m", ""))
        normalized.append(
            {
                "trial_id": row.get("trial_id", ""),
                "scenario": row.get("scenario", ""),
                "overall_success": str(row.get("overall_success", "")).lower() == "true",
                "dynamic_motion_m": as_float(dynamic_motion_m),
                "target_min_height_m": as_float(row.get("target_min_height_m", "")),
                "target_object_height_m": as_float(row.get("target_object_height_m", "")),
                "z_valid": row.get("z_valid", ""),
                "height_filter_pass": row.get("height_filter_pass", ""),
                "dynamic_filter_pass": row.get("dynamic_filter_pass", ""),
                "elevated_dynamic_target_valid": row.get("elevated_dynamic_target_valid", ""),
                "classification": row.get("classification", ""),
                "compensated_motion_m": as_float(row.get("compensated_motion_m", dynamic_motion_m)),
                "compensated_velocity_mps": as_float(row.get("compensated_velocity_mps", "")),
                "total_duration_sec": as_float(row.get("total_duration_sec", "")),
                "target_detected": row.get("target_detected", ""),
                "moving_target_valid": row.get("moving_target_valid", ""),
                "map_transform_success": row.get("map_transform_success", ""),
                "target_goal_success": row.get("target_goal_success", ""),
                "yaw_alignment_success": row.get("yaw_alignment_success", ""),
                "camera_detection_success": row.get("camera_detection_success", ""),
                "sound_mission_success": row.get("sound_mission_success", ""),
                "patrol_resume_success": row.get("patrol_resume_success", ""),
                "safety_gate_pass": row.get("safety_gate_pass", ""),
                "failure_reason": row.get("failure_reason", ""),
                "summary_path": row.get("summary_path", ""),
            }
        )
    total = len(normalized)
    success = sum(1 for row in normalized if row["overall_success"])
    success_rate = success / total if total else 0.0
    write_csv(
        output_dir / "gazebo_trial_summary.csv",
        [
            "trial_id", "scenario", "overall_success", "dynamic_motion_m",
            "target_min_height_m", "target_object_height_m", "z_valid",
            "height_filter_pass", "dynamic_filter_pass", "elevated_dynamic_target_valid",
            "classification", "compensated_motion_m", "compensated_velocity_mps",
            "total_duration_sec",
            "target_detected", "moving_target_valid", "map_transform_success", "target_goal_success",
            "yaw_alignment_success", "camera_detection_success", "sound_mission_success",
            "patrol_resume_success", "safety_gate_pass", "failure_reason", "summary_path",
        ],
        normalized,
    )
    write_csv(
        output_dir / "gazebo_success_rate.csv",
        ["total_trials", "successful_trials", "mission_success_rate"],
        [{"total_trials": total, "successful_trials": success, "mission_success_rate": success_rate}],
    )
    print(f"Gazebo trial analysis: {success}/{total} success, rate={success_rate:.3f}")
    print(f"Wrote {output_dir / 'gazebo_trial_summary.csv'}")
    print(f"Wrote {output_dir / 'gazebo_success_rate.csv'}")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())

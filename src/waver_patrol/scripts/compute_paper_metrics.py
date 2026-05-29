#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from statistics import mean


GT_NA = "N/A - no ground truth"


def read_rows(root: Path, name: str) -> list[dict[str, str]]:
    path = root / name
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def floats(rows: list[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        try:
            text = row.get(key, "")
            if text in ("", "nan", "None"):
                continue
            value = float(text)
            if math.isfinite(value):
                values.append(value)
        except (TypeError, ValueError):
            continue
    return values


def count_contains(rows: list[dict[str, str]], key: str, token: str) -> int:
    token = token.lower()
    return sum(1 for row in rows if token in str(row.get(key, "")).lower())


def count_nonempty(rows: list[dict[str, str]], key: str) -> int:
    return sum(1 for row in rows if str(row.get(key, "")).strip())


def pct(numer: float, denom: float) -> float | str:
    if denom <= 0:
        return "N/A"
    return float(numer) / float(denom)


def compute(root: Path) -> dict[str, object]:
    mission = read_rows(root, "mission_events.csv")
    waypoint = read_rows(root, "waypoint_progress.csv")
    lidar = read_rows(root, "lidar_targets.csv")
    goals = read_rows(root, "inspection_goals.csv")
    approach = read_rows(root, "target_approach.csv")
    camera = read_rows(root, "camera_alignment.csv")
    cls = read_rows(root, "camera_classification.csv")
    sound = read_rows(root, "sound_events.csv")
    ret = read_rows(root, "return_to_patrol.csv")
    safety = read_rows(root, "safety_state.csv")
    cmd = read_rows(root, "cmd_vel.csv")
    pose = read_rows(root, "robot_pose.csv")

    accepted_lidar = [
        row for row in lidar
        if str(row.get("accepted_as_aerial_target", "")).lower() in {"true", "1", "yes"}
    ]
    sound_requests = [
        row for row in sound
        if str(row.get("sound_alert_request", "")).lower() in {"true", "1", "yes"}
    ]
    sound_done = [
        row for row in sound
        if str(row.get("sound_task_done", "")).lower() in {"true", "1", "yes"}
    ]
    class_counts = {
        name: sum(1 for row in cls if row.get("target_class", "").lower() == name)
        for name in ("bird", "drone", "unknown", "irrelevant")
    }

    total_distance = floats(pose, "distance_accumulated_m")
    metrics: dict[str, object] = {
        "navigation.waypoint_success_rate": pct(count_contains(waypoint, "arrival_success", "true"), len(waypoint)),
        "navigation.average_waypoint_travel_time_sec": mean(floats(waypoint, "arrival_time_sec")) if floats(waypoint, "arrival_time_sec") else "N/A",
        "navigation.total_distance_m": max(total_distance) if total_distance else "N/A",
        "navigation.path_efficiency": "N/A - planned path length not logged",
        "navigation.nav_goal_success_rate": pct(count_contains(mission, "reason", "NAV2_GOAL_SUCCEEDED"), count_contains(mission, "reason", "goal_type")),
        "navigation.recovery_count": count_contains(mission, "reason", "NAV2_RECOVERY"),
        "navigation.safety_stop_count": count_contains(safety, "safety_state", "STOP"),
        "navigation.cmd_vel_timeout_count": count_contains(safety, "safety_state", "TIMEOUT"),
        "lidar.target_detection_count": len(lidar),
        "lidar.accepted_aerial_target_count": len(accepted_lidar),
        "lidar.rejected_low_height_count": count_contains(lidar, "reject_reason", "height_low") + count_contains(goals, "goal_reject_reason", "height_low"),
        "lidar.rejected_static_count": count_contains(lidar, "reject_reason", "static") + count_contains(goals, "goal_reject_reason", "static"),
        "lidar.rejected_no_tf_count": count_contains(lidar, "reject_reason", "no_tf") + count_contains(goals, "goal_reject_reason", "no_tf"),
        "lidar.average_target_height_m": mean(floats(lidar, "height_m")) if floats(lidar, "height_m") else "N/A",
        "lidar.average_target_range_m": mean(floats(lidar, "range_m")) if floats(lidar, "range_m") else "N/A",
        "lidar.average_target_velocity_mps": mean(floats(lidar, "velocity_mps")) if floats(lidar, "velocity_mps") else "N/A",
        "lidar.lidar_detection_to_goal_latency_ms": mean(floats(goals, "candidate_to_goal_latency_ms")) if floats(goals, "candidate_to_goal_latency_ms") else "N/A",
        "lidar.dynamic_validation_latency_ms": "N/A - validation timestamps not paired",
        "lidar.target_track_duration_sec": "N/A - track start/end not logged",
        "lidar.target_lost_count": count_nonempty(lidar, "target_lost_reason"),
        "inspection.inspection_goal_success_rate": pct(count_contains(goals, "goal_publish_success", "true"), len(goals)),
        "inspection.approach_success_rate": pct(count_contains(approach, "arrival_success", "true"), len(approach)),
        "inspection.average_approach_duration_sec": mean(floats(approach, "approach_duration_sec")) if floats(approach, "approach_duration_sec") else "N/A",
        "inspection.final_standoff_error_m": mean(floats(approach, "final_distance_to_target_m")) if floats(approach, "final_distance_to_target_m") else "N/A",
        "inspection.target_reacquisition_rate": "N/A - reacquisition labels not logged",
        "inspection.target_arrival_distance_m": mean(floats(approach, "final_distance_to_target_m")) if floats(approach, "final_distance_to_target_m") else "N/A",
        "camera.alignment_success_rate": pct(count_contains(camera, "centered", "true"), len(camera)),
        "camera.average_alignment_latency_ms": mean(floats(camera, "alignment_latency_ms")) if floats(camera, "alignment_latency_ms") else "N/A",
        "camera.pointing_error_mean_rad": mean(floats(camera, "pointing_error_rad")) if floats(camera, "pointing_error_rad") else "N/A",
        "camera.pointing_error_std_rad": "N/A - not enough paired pointing-error samples",
        "camera.camera_centered_rate": pct(count_contains(camera, "centered", "true"), len(camera)),
        "camera.fallback_body_alignment_count": count_contains(camera, "fallback_robot_body_alignment_used", "true"),
        "classification.class_counts": class_counts,
        "classification.bird_precision": GT_NA,
        "classification.bird_recall": GT_NA,
        "classification.bird_f1": GT_NA,
        "classification.drone_precision": GT_NA,
        "classification.drone_recall": GT_NA,
        "classification.drone_f1": GT_NA,
        "classification.confusion_matrix": GT_NA,
        "classification.average_classification_latency_ms": mean(floats(cls, "classification_latency_ms")) if floats(cls, "classification_latency_ms") else "N/A",
        "classification.inference_fps_mean": mean(floats(cls, "inference_fps")) if floats(cls, "inference_fps") else "N/A",
        "classification.mAP@0.5": GT_NA,
        "classification.mAP@0.5:0.95": GT_NA,
        "sound.sound_request_count": len(sound_requests),
        "sound.sound_accept_count": count_contains(sound, "sound_request_accepted", "true"),
        "sound.sound_blocked_by_class_count": count_contains(sound, "sound_block_reason", "BY_CLASS"),
        "sound.sound_task_completion_rate": pct(len(sound_done), len(sound_requests)),
        "sound.average_sound_duration_sec": mean(floats(sound, "sound_duration_sec")) if floats(sound, "sound_duration_sec") else "N/A",
        "sound.bird_confirmed_to_sound_latency_sec": "N/A - paired timestamps not logged",
        "sound.sound_to_resume_patrol_latency_sec": "N/A - paired timestamps not logged",
        "return.return_to_patrol_success_rate": pct(count_contains(ret, "resumed_patrol", "true"), len(ret)),
        "return.average_return_duration_sec": mean(floats(ret, "return_duration_sec")) if floats(ret, "return_duration_sec") else "N/A",
        "return.resume_patrol_success_rate": pct(count_contains(mission, "next_state", "RESUME_PATROL"), count_contains(mission, "event_type", "STATE_CHANGE")),
        "return.post_target_resume_cooldown_count": count_contains(mission, "reason", "POST_TARGET_RESUME_COOLDOWN"),
        "safety.estop_count": count_contains(safety, "estop", "true"),
        "safety.external_stop_count": count_contains(safety, "external_stop", "true"),
        "safety.scan_stale_stop_count": count_contains(safety, "safety_state", "SCAN_STALE"),
        "safety.odom_stale_stop_count": count_contains(safety, "safety_state", "ODOM_STALE"),
        "safety.battery_stale_count": count_contains(safety, "battery_state", "STALE"),
        "safety.battery_critical_count": count_contains(safety, "battery_state", "CRITICAL"),
        "safety.min_obstacle_distance_m": min(floats(safety, "min_scan_range_m")) if floats(safety, "min_scan_range_m") else "N/A",
        "safety.safety_stop_duration_sec": "N/A - contiguous stop windows not reconstructed",
        "slam.map_known_ratio": "N/A - no occupancy-grid summary",
        "slam.occupied_cell_count": "N/A - no occupancy-grid summary",
        "slam.free_cell_count": "N/A - no occupancy-grid summary",
        "slam.scan_matching_failed_count": "N/A - backend-specific",
        "slam.ATE": GT_NA,
        "slam.RPE": GT_NA,
        "slam.odom_drift_estimate": "N/A - no reference trajectory",
        "cmd.sample_count": len(cmd),
    }
    return metrics


def write_outputs(root: Path, metrics: dict[str, object]) -> None:
    json_path = root / "paper_metrics_summary.json"
    csv_path = root / "paper_metrics_summary.csv"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        for key in sorted(metrics):
            value = metrics[key]
            writer.writerow([key, json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value])


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: compute_paper_metrics.py <experiment_result_dir>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).expanduser().resolve()
    if not root.exists():
        print(f"experiment_result_dir not found: {root}", file=sys.stderr)
        return 2
    metrics = compute(root)
    write_outputs(root, metrics)
    print(f"wrote {root / 'paper_metrics_summary.csv'}")
    print(f"wrote {root / 'paper_metrics_summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

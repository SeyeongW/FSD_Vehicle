#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def safe_float(value: Any) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else math.nan
    except Exception:
        return math.nan


def nums(table: list[dict[str, str]], field: str) -> list[float]:
    out = []
    for row in table:
        value = safe_float(row.get(field))
        if math.isfinite(value):
            out.append(value)
    return out


def nums_any(table: list[dict[str, str]], fields: list[str]) -> list[float]:
    out = []
    for row in table:
        for field in fields:
            value = safe_float(row.get(field))
            if math.isfinite(value):
                out.append(value)
                break
    return out


def truth(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass", "ok"}


def mean(values: list[float]) -> float | str:
    return statistics.mean(values) if values else ""


def p95(values: list[float]) -> float | str:
    if not values:
        return ""
    values = sorted(values)
    idx = min(len(values) - 1, int(math.ceil(0.95 * len(values))) - 1)
    return values[idx]


def first_event(table: list[dict[str, str]], event_type: str, field: str) -> float | str:
    for row in table:
        if row.get("event_type") == event_type:
            value = safe_float(row.get(field))
            if math.isfinite(value):
                return value
    return ""


def first_event_time(table: list[dict[str, str]], event_type: str) -> float | None:
    for row in table:
        if row.get("event_type") == event_type:
            value = safe_float(row.get("event_time_sec") or row.get("time_sec"))
            if math.isfinite(value):
                return value
    return None


def first_event_time_after(table: list[dict[str, str]], predicate, after: float | None) -> float | None:
    for row in table:
        if not predicate(row):
            continue
        value = safe_float(row.get("event_time_sec") or row.get("time_sec"))
        if math.isfinite(value) and (after is None or value >= after):
            return value
    return None


def target_goal_row(row: dict[str, str]) -> bool:
    return (
        str(row.get("is_target_related_goal", "")).lower() == "true"
        or str(row.get("goal_role", "")).upper() == "TARGET_INSPECTION"
    )


def parse_cmd_vel_authority(path: Path) -> tuple[int | str, bool]:
    if not path.exists():
        return "", False
    text = path.read_text(encoding="utf-8", errors="replace")
    publisher_count: int | None = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("Publisher count:"):
            try:
                publisher_count = int(stripped.split(":", 1)[1].strip())
            except Exception:
                publisher_count = None
    sole_mux = (
        publisher_count == 1
        and "safety_cmd_mux_node" in text
        and "simple_nav2_cmd_sim_node" not in text
        and "seo_cluster_tracker_node" not in text
    )
    return (publisher_count if publisher_count is not None else "", sole_mux)


def first_time(table: list[dict[str, str]], predicate) -> float | None:
    for row in table:
        if predicate(row):
            value = safe_float(row.get("time_sec"))
            if math.isfinite(value):
                return value
    return None


def first_field_time(table: list[dict[str, str]], predicate, field: str) -> float | None:
    for row in table:
        if predicate(row):
            value = safe_float(row.get(field))
            if math.isfinite(value):
                return value
    return None


def first_time_after(table: list[dict[str, str]], predicate, after: float | None) -> float | None:
    for row in table:
        if not predicate(row):
            continue
        value = safe_float(row.get("time_sec"))
        if math.isfinite(value) and (after is None or value >= after):
            return value
    return None


def delta_ms(a: float | None, b: float | None) -> float | str:
    if a is None or b is None:
        return ""
    return (b - a) * 1000.0


def write_metric_csv(path: Path, metrics: dict[str, Any]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "value"])
        writer.writeheader()
        for key, value in metrics.items():
            writer.writerow({"metric": key, "value": value})


def write_paper_table(path: Path, rows_in: list[tuple[str, Any, str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "value", "unit", "meaning"])
        writer.writeheader()
        for metric, value, unit, meaning in rows_in:
            writer.writerow({"metric": metric, "value": value, "unit": unit, "meaning": meaning})


def sanitize(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [sanitize(v) for v in value]
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    logs = run_dir / "logs"
    metrics_dir = run_dir / "metrics"
    metrics_dir.mkdir(parents=True, exist_ok=True)

    spatial = rows(logs / "spatial_distance_timeseries.csv")
    goals = rows(logs / "goal_bird_distance_events.csv")
    birds = rows(logs / "bird_kinematics.csv")
    lidar = rows(logs / "lidar_filter_response.csv")
    cmd = rows(logs / "cmd_vel_response.csv")
    latency = rows(logs / "lidar_waver_latency_events.csv")
    mission_events = rows(logs / "mission_events.csv")
    removal_events = rows(logs / "bird_removal_events.csv")
    cmd_vel_publisher_count, cmd_vel_sole_mux = parse_cmd_vel_authority(logs / "cmd_vel_topic_info.txt")

    detector_mode = ""
    for row in spatial:
        detector_mode = row.get("detector_mode", "")
        if detector_mode:
            break
    removed = {
        row.get("bird_name", "")
        for row in birds
        if row.get("bird_name", "") and str(row.get("removed", "")).lower() == "true"
    }
    gt_leakage = sum(
        1
        for row in lidar
        if detector_mode == "lidar" and str(row.get("provenance", "")).lower() in {"gazebo_gt", "ground_truth"}
    )
    first_pc = None
    if latency:
        first_pc = safe_float(latency[-1].get("first_pointcloud_time_sec"))
        first_pc = first_pc if math.isfinite(first_pc) else None
    first_lidar_ok = first_field_time(
        lidar,
        lambda r: str(r.get("target_ok", "")).lower() == "true",
        "filter_done_time_sec",
    )
    first_lock = first_time_after(
        spatial,
        lambda r: str(r.get("dynamic_lock", "")).lower() == "true",
        first_lidar_ok,
    )
    raw_target_goals = [row for row in goals if target_goal_row(row)]
    raw_object_target_goals = [
        row
        for row in raw_target_goals
        if row.get("event_type") == "object_mission_goal"
        and str(row.get("target_bird_name", row.get("bird_name", ""))).strip()
    ]

    def matches_recent_object_goal(row: dict[str, str]) -> bool:
        if row.get("event_type") != "active_nav_goal":
            return False
        row_time = safe_float(row.get("time_sec"))
        gx = safe_float(row.get("goal_x_m"))
        gy = safe_float(row.get("goal_y_m"))
        if not (math.isfinite(row_time) and math.isfinite(gx) and math.isfinite(gy)):
            return False
        for obj in reversed(raw_object_target_goals):
            obj_time = safe_float(obj.get("time_sec"))
            ox = safe_float(obj.get("goal_x_m"))
            oy = safe_float(obj.get("goal_y_m"))
            if not (math.isfinite(obj_time) and math.isfinite(ox) and math.isfinite(oy)):
                continue
            if 0.0 <= row_time - obj_time <= 2.0 and math.hypot(gx - ox, gy - oy) <= 0.35:
                return True
        return False

    target_goals = [row for row in goals if target_goal_row(row) or matches_recent_object_goal(row)]
    object_target_goals = [
        row
        for row in target_goals
        if row.get("event_type") == "object_mission_goal"
        and str(row.get("target_bird_name", row.get("bird_name", ""))).strip()
    ]
    target_spatial = [row for row in spatial if str(row.get("active_goal_role", "")).upper() == "TARGET_INSPECTION"]
    valid_lidar_spatial = [
        row
        for row in spatial
        if truth(row.get("lidar_target_valid"))
        and safe_float(row.get("lidar_target_age_sec")) <= 0.5
        and truth(row.get("dynamic_lock"))
    ]
    active_birds = [
        row
        for row in birds
        if str(row.get("active", "")).lower() == "true"
        and str(row.get("hidden", "")).lower() != "true"
        and str(row.get("removed", "")).lower() != "true"
    ]
    active_flee_birds = [row for row in active_birds if truth(row.get("fleeing"))]
    active_normal_birds = [row for row in active_birds if not truth(row.get("fleeing"))]
    # Object/active target goals can be generated immediately when the mission
    # manager preempts a patrol waypoint. The 1 Hz spatial sample may mark
    # dynamic_lock slightly later, so use the higher-rate LiDAR filter OK time as
    # the lower bound for first target-goal timing.
    first_object = first_event_time_after(
        target_goals,
        lambda r: r.get("event_type") == "object_mission_goal",
        first_lidar_ok,
    )
    first_active = first_event_time_after(goals, lambda r: r.get("event_type") == "active_nav_goal", first_object)
    first_active_target = first_event_time_after(
        target_goals,
        lambda r: r.get("event_type") == "active_nav_goal",
        first_object,
    )
    patrol_object_goal_time = None
    patrol_active_target_goal_time = None
    for obj in object_target_goals:
        obj_time = safe_float(obj.get("time_sec"))
        mission_state = str(obj.get("mission_state", "")).upper()
        if not math.isfinite(obj_time) or "PATROL" not in mission_state:
            continue
        obj_x = safe_float(obj.get("goal_x_m"))
        obj_y = safe_float(obj.get("goal_y_m"))
        for active in target_goals:
            if active.get("event_type") != "active_nav_goal":
                continue
            if str(active.get("goal_role", "")).upper() != "TARGET_INSPECTION":
                continue
            active_time = safe_float(active.get("time_sec"))
            if not (math.isfinite(active_time) and 0.0 <= active_time - obj_time <= 2.0):
                continue
            active_x = safe_float(active.get("goal_x_m"))
            active_y = safe_float(active.get("goal_y_m"))
            same_goal = True
            if all(math.isfinite(v) for v in (obj_x, obj_y, active_x, active_y)):
                same_goal = math.hypot(active_x - obj_x, active_y - obj_y) <= 0.35
            if not same_goal:
                continue
            patrol_object_goal_time = obj_time
            patrol_active_target_goal_time = active_time
            break
        if patrol_object_goal_time is not None:
            break
    first_cmd = first_time_after(
        cmd,
        lambda r: str(r.get("nonzero_cmd", "")).lower() == "true",
        first_lidar_ok,
    )
    first_target_cmd = first_time_after(
        cmd,
        lambda r: str(r.get("nonzero_cmd", "")).lower() == "true"
        and (
            str(r.get("mission_state", "")).upper().startswith("APPROACH_TARGET_OFFSET")
            or str(r.get("mission_state", "")).upper().startswith("TARGET_NAVIGATING")
        ),
        first_active_target,
    )
    if first_target_cmd is None:
        first_target_cmd = first_time_after(cmd, lambda r: str(r.get("nonzero_cmd", "")).lower() == "true", first_active_target)
    odom_start = None
    for row in spatial:
        t = safe_float(row.get("time_sec"))
        if first_active_target is not None and math.isfinite(t) and t >= first_active_target:
            x = safe_float(row.get("waver_x_m"))
            y = safe_float(row.get("waver_y_m"))
            if math.isfinite(x) and math.isfinite(y):
                odom_start = (x, y)
                break
    first_odom_motion = None
    if odom_start is not None and first_active_target is not None:
        for row in spatial:
            t = safe_float(row.get("time_sec"))
            x = safe_float(row.get("waver_x_m"))
            y = safe_float(row.get("waver_y_m"))
            if (
                math.isfinite(t)
                and t >= first_active_target
                and math.isfinite(x)
                and math.isfinite(y)
                and math.hypot(x - odom_start[0], y - odom_start[1]) >= 0.03
            ):
                first_odom_motion = t
                break
    removed_times = [
        safe_float(row.get("time_sec"))
        for row in removal_events
        if str(row.get("event", row.get("state", ""))).upper().startswith("REMOVED")
        and math.isfinite(safe_float(row.get("time_sec")))
    ]
    if not removed_times:
        removed_times = [
            safe_float(row.get("time_sec"))
            for row in birds
            if truth(row.get("removed")) and math.isfinite(safe_float(row.get("time_sec")))
        ]
    last_removed_time = max(removed_times) if removed_times else None
    first_return = first_time_after(
        spatial,
        lambda r: "RETURN" in str(r.get("mission_state", "")).upper(),
        last_removed_time,
    )
    first_resume = first_time_after(
        spatial,
        lambda r: "RESUME" in str(r.get("mission_state", "")).upper(),
        first_return,
    )
    final_patrol = first_time_after(
        spatial,
        lambda r: str(r.get("mission_state", "")).upper().startswith("PATROL"),
        first_resume,
    )
    if first_return is None and mission_events:
        first_return = first_event_time_after(
            mission_events,
            lambda r: "RETURN" in " ".join(str(v).upper() for v in r.values()),
            last_removed_time,
        )
    if first_resume is None and mission_events:
        first_resume = first_event_time_after(
            mission_events,
            lambda r: "RESUME" in " ".join(str(v).upper() for v in r.values()),
            first_return,
        )
    if first_resume is not None and (final_patrol is None or final_patrol < first_resume):
        final_patrol = first_time_after(
            spatial,
            lambda r: str(r.get("mission_state", "")).upper().startswith("PATROL"),
            first_resume,
        )
    if (final_patrol is None or (first_resume is not None and final_patrol < first_resume)) and mission_events:
        final_patrol = first_event_time_after(
            mission_events,
            lambda r: "PATROL" in " ".join(str(v).upper() for v in r.values()),
            first_resume,
        )
    return_resume_sequence = (
        last_removed_time is not None
        and first_return is not None
        and first_resume is not None
        and final_patrol is not None
        and last_removed_time <= first_return <= first_resume <= final_patrol
    )

    target_goal_to_bird = nums(object_target_goals, "goal_to_bird_xy_m")
    target_goal_to_lidar = nums(object_target_goals, "goal_to_lidar_target_xy_m")
    lidar_runtime_wall = nums_any(lidar, ["filter_runtime_wall_ms", "filter_runtime_ms"])
    odom_path_length = 0.0
    prev_xy: tuple[float, float] | None = None
    first_xy: tuple[float, float] | None = None
    last_xy: tuple[float, float] | None = None
    for row in spatial:
        x = safe_float(row.get("waver_x_m"))
        y = safe_float(row.get("waver_y_m"))
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if first_xy is None:
            first_xy = (x, y)
        if prev_xy is not None:
            step = math.hypot(x - prev_xy[0], y - prev_xy[1])
            if math.isfinite(step) and step < 5.0:
                odom_path_length += step
        prev_xy = (x, y)
        last_xy = (x, y)
    net_displacement = math.hypot(last_xy[0] - first_xy[0], last_xy[1] - first_xy[1]) if first_xy and last_xy else math.nan
    nonzero_cmd_rows = [row for row in cmd if truth(row.get("nonzero_cmd"))]
    cmd_times = [safe_float(row.get("time_sec")) for row in nonzero_cmd_rows if math.isfinite(safe_float(row.get("time_sec")))]
    cmd_active_duration = max(cmd_times) - min(cmd_times) if len(cmd_times) >= 2 else 0.0

    metrics = {
        "mechanism.detector_mode": detector_mode,
        "mechanism.gt_leakage_in_lidar_mode_count": gt_leakage,
        "mechanism.lidar_only_decision_clean": detector_mode != "lidar" or gt_leakage == 0,
        "mechanism.has_lidar_target_ok": first_lidar_ok is not None,
        "mechanism.has_dynamic_lock": first_lock is not None,
        "mechanism.has_object_mission_goal": first_object is not None,
        "mechanism.has_active_nav_goal": first_active is not None,
        "mechanism.has_active_target_nav_goal": first_active_target is not None,
        "mechanism.has_nonzero_cmd_vel": first_cmd is not None,
        "mechanism.has_nonzero_cmd_vel_after_target_goal": first_target_cmd is not None,
        "mechanism.has_target_cmd_vel": first_target_cmd is not None,
        "mechanism.has_odom_motion_after_target_goal": first_odom_motion is not None,
        "mechanism.has_patrol_preempt_to_target_goal": patrol_active_target_goal_time is not None,
        "mechanism.has_return_to_interrupted_waypoint": first_return is not None,
        "mechanism.has_resume_patrol": first_resume is not None,
        "mechanism.has_final_patrol_after_resume": final_patrol is not None,
        "mechanism.has_return_resume": return_resume_sequence,
        "mechanism.return_resume_sequence_success": return_resume_sequence,
        "mechanism.removed_bird_count": len(removed),
        "mechanism.two_bird_removal_success": len(removed) >= 2,
        "mechanism.four_bird_removal_success": len(removed) >= 4,
        "spatial.robot_to_bird_xy_mean_m": mean(nums(spatial, "robot_to_bird_xy_m")),
        "spatial.robot_to_bird_xy_min_m": min(nums(spatial, "robot_to_bird_xy_m") or [math.nan]),
        "spatial.object_goal_to_bird_xy_first_m": first_event(target_goals, "object_mission_goal", "goal_to_bird_xy_m"),
        "spatial.active_goal_to_bird_xy_first_m": first_event(target_goals, "active_nav_goal", "goal_to_bird_xy_m"),
        "spatial.target_goal_to_bird_xy_first_m": first_event(object_target_goals, "object_mission_goal", "goal_to_bird_xy_m"),
        "spatial.target_goal_to_bird_xy_mean_m": mean(target_goal_to_bird),
        "spatial.target_goal_to_bird_xy_std_m": statistics.pstdev(target_goal_to_bird) if len(target_goal_to_bird) > 1 else "",
        "spatial.target_goal_to_bird_xy_min_m": min(target_goal_to_bird or [math.nan]),
        "spatial.target_goal_to_bird_xy_max_m": max(target_goal_to_bird or [math.nan]),
        "spatial.target_goal_to_lidar_target_xy_first_m": first_event(object_target_goals, "object_mission_goal", "goal_to_lidar_target_xy_m"),
        "spatial.target_goal_to_lidar_target_xy_mean_m": mean(target_goal_to_lidar),
        "spatial.target_goal_to_lidar_target_xy_std_m": statistics.pstdev(target_goal_to_lidar) if len(target_goal_to_lidar) > 1 else "",
        "spatial.goal_to_bird_xy_mean_m": mean(target_goal_to_bird),
        "spatial.goal_to_bird_xy_std_m": statistics.pstdev(target_goal_to_bird) if len(target_goal_to_bird) > 1 else "",
        "spatial.standoff_error_mean_abs_m": mean([abs(v) for v in nums(target_spatial, "active_goal_standoff_error_to_bird_m")]),
        "spatial.target_goal_standoff_error_to_bird_mean_abs_m": mean([abs(v) for v in nums(object_target_goals, "standoff_error_to_bird_m")]),
        "spatial.target_goal_standoff_error_to_lidar_target_mean_abs_m": mean(
            [abs(v) for v in nums(object_target_goals, "standoff_error_to_lidar_target_m")]
        ),
        "spatial.lidar_target_to_bird_xy_mean_m": mean(nums(spatial, "lidar_target_to_bird_xy_m")),
        "spatial.lidar_target_to_bird_xy_p95_m": p95(nums(spatial, "lidar_target_to_bird_xy_m")),
        "spatial.valid_lidar_target_sample_count": len(valid_lidar_spatial),
        "spatial.valid_lidar_target_to_bird_xy_mean_m": mean(nums(valid_lidar_spatial, "lidar_target_to_bird_xy_m")),
        "spatial.valid_lidar_target_to_bird_xy_p95_m": p95(nums(valid_lidar_spatial, "lidar_target_to_bird_xy_m")),
        "spatial.valid_lidar_target_to_bird_xy_max_m": max(nums(valid_lidar_spatial, "lidar_target_to_bird_xy_m") or [math.nan]),
        "bird.active_speed_xy_mean_mps": mean(nums(active_normal_birds, "speed_xy_mps")),
        "bird.active_speed_xy_max_mps": max(nums(active_normal_birds, "speed_xy_mps") or [math.nan]),
        "bird.flee_speed_xy_mean_mps": mean(nums(active_flee_birds, "speed_xy_mps")),
        "bird.flee_speed_xy_max_mps": max(nums(active_flee_birds, "speed_xy_mps") or [math.nan]),
        "bird.speed_xy_mean_mps": mean(nums(active_birds, "speed_xy_mps")),
        "bird.speed_xy_max_mps": max(nums(active_birds, "speed_xy_mps") or [math.nan]),
        "bird.speed_xy_normal_mean_mps": mean(nums(active_normal_birds, "speed_xy_mps")),
        "bird.speed_xy_flee_mean_mps": mean(nums(active_flee_birds, "speed_xy_mps")),
        "bird.speed_3d_mean_mps": mean(nums(active_birds, "speed_3d_mps")),
        "bird.speed_3d_max_mps": max(nums(active_birds, "speed_3d_mps") or [math.nan]),
        "bird.motion_sample_count": len(active_birds),
        "lidar.filter_runtime_mean_ms": mean(lidar_runtime_wall),
        "lidar.filter_runtime_p95_ms": p95(lidar_runtime_wall),
        "lidar.filter_runtime_wall_mean_ms": mean(lidar_runtime_wall),
        "lidar.filter_runtime_wall_p95_ms": p95(lidar_runtime_wall),
        "lidar.pc_stamp_to_filter_done_mean_ms": mean(nums(lidar, "pc_stamp_to_filter_done_ms")),
        "lidar.pc_stamp_to_filter_done_sim_mean_ms": mean(nums(lidar, "pc_stamp_to_filter_done_sim_ms")),
        "lidar.pc_stamp_to_filter_done_p95_ms": p95(nums(lidar, "pc_stamp_to_filter_done_ms")),
        "lidar.raw_points_mean": mean(nums(lidar, "raw_points")),
        "lidar.roi_points_mean": mean(nums(lidar, "roi_points")),
        "lidar.cluster_points_mean": mean(nums(lidar, "cluster_points")),
        "lidar.roi_ratio_mean": mean(nums(lidar, "roi_ratio")),
        "lidar.cluster_ratio_mean": mean(nums(lidar, "cluster_ratio")),
        "lidar.target_ok_count": sum(1 for row in lidar if str(row.get("target_ok", "")).lower() == "true"),
        "lidar.provenance_gt_count": gt_leakage,
        "latency.pointcloud_to_lidar_target_ok_ms": delta_ms(first_pc, first_lidar_ok),
        "latency.lidar_target_ok_to_dynamic_lock_ms": delta_ms(first_lidar_ok, first_lock),
        "latency.dynamic_lock_to_object_mission_goal_ms": delta_ms(first_lock, first_object),
        "latency.object_mission_goal_to_active_nav_goal_ms": delta_ms(first_object, first_active),
        "latency.object_mission_goal_to_active_target_nav_goal_ms": delta_ms(first_object, first_active_target),
        "latency.patrol_object_goal_to_active_target_nav_goal_ms": delta_ms(
            patrol_object_goal_time,
            patrol_active_target_goal_time,
        ),
        "latency.lidar_target_ok_to_first_cmd_vel_ms": delta_ms(first_lidar_ok, first_cmd),
        "latency.lidar_target_ok_to_first_target_cmd_vel_ms": delta_ms(first_lidar_ok, first_target_cmd),
        "latency.active_target_nav_goal_to_cmd_vel_ms": delta_ms(first_active_target, first_target_cmd),
        "latency.active_target_nav_goal_to_odom_motion_ms": delta_ms(first_active_target, first_odom_motion),
        "navigation.odom_path_length_m": odom_path_length,
        "navigation.net_displacement_m": net_displacement,
        "navigation.cmd_vel_nonzero_rows": len(nonzero_cmd_rows),
        "navigation.cmd_vel_active_duration_sec": cmd_active_duration,
        "safety.cmd_vel_publisher_count": cmd_vel_publisher_count,
        "safety.cmd_vel_safety_mux_sole_publisher": cmd_vel_sole_mux,
        "safety.cmd_vel_authority_pass": cmd_vel_sole_mux,
    }
    metrics = sanitize(metrics)
    (metrics_dir / "spatial_response_metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False, default=str), encoding="utf-8")
    write_metric_csv(metrics_dir / "spatial_response_metrics.csv", metrics)
    write_paper_table(
        metrics_dir / "paper_spatial_table.csv",
        [
            ("target_goal_to_bird_xy_first_m", metrics["spatial.target_goal_to_bird_xy_first_m"], "m", "First target-inspection nav goal to bird XY distance"),
            ("target_goal_to_bird_xy_mean_m", metrics["spatial.target_goal_to_bird_xy_mean_m"], "m", "Mean target-inspection goal to bird XY distance"),
            ("target_goal_to_lidar_target_xy_mean_m", metrics["spatial.target_goal_to_lidar_target_xy_mean_m"], "m", "Mean target-inspection goal to LiDAR target XY distance"),
            ("target_goal_standoff_error_to_bird_mean_abs_m", metrics["spatial.target_goal_standoff_error_to_bird_mean_abs_m"], "m", "Mean absolute target-goal standoff error to bird"),
            ("target_goal_standoff_error_to_lidar_target_mean_abs_m", metrics["spatial.target_goal_standoff_error_to_lidar_target_mean_abs_m"], "m", "Mean absolute target-goal standoff error to LiDAR target"),
            ("valid_lidar_target_to_bird_xy_mean_m", metrics["spatial.valid_lidar_target_to_bird_xy_mean_m"], "m", "Mean valid LiDAR target to associated bird GT XY distance"),
            ("robot_to_bird_xy_min_m", metrics["spatial.robot_to_bird_xy_min_m"], "m", "Minimum Waver to bird XY distance"),
            ("active_bird_speed_xy_mean_mps", metrics["bird.active_speed_xy_mean_mps"], "m/s", "Mean active non-flee bird XY speed"),
            ("flee_bird_speed_xy_mean_mps", metrics["bird.flee_speed_xy_mean_mps"], "m/s", "Mean fleeing bird XY speed"),
            ("odom_path_length_m", metrics["navigation.odom_path_length_m"], "m", "Waver odometry path length"),
        ],
    )
    write_paper_table(
        metrics_dir / "paper_latency_table.csv",
        [
            ("filter_runtime_wall_mean_ms", metrics["lidar.filter_runtime_wall_mean_ms"], "ms", "Mean LiDAR filtering wall runtime"),
            ("filter_runtime_wall_p95_ms", metrics["lidar.filter_runtime_wall_p95_ms"], "ms", "P95 LiDAR filtering wall runtime"),
            ("pc_stamp_to_filter_done_mean_ms", metrics["lidar.pc_stamp_to_filter_done_mean_ms"], "ms", "PointCloud stamp to filter completion latency"),
            ("pointcloud_to_lidar_target_ok_ms", metrics["latency.pointcloud_to_lidar_target_ok_ms"], "ms", "PointCloud to first LiDAR target OK"),
            ("lidar_target_ok_to_dynamic_lock_ms", metrics["latency.lidar_target_ok_to_dynamic_lock_ms"], "ms", "LiDAR target to dynamic lock"),
            ("dynamic_lock_to_object_mission_goal_ms", metrics["latency.dynamic_lock_to_object_mission_goal_ms"], "ms", "Dynamic lock to object mission goal"),
            ("object_mission_goal_to_active_target_nav_goal_ms", metrics["latency.object_mission_goal_to_active_target_nav_goal_ms"], "ms", "Object mission goal to active target nav goal"),
            ("patrol_object_goal_to_active_target_nav_goal_ms", metrics["latency.patrol_object_goal_to_active_target_nav_goal_ms"], "ms", "Patrol-time object goal to active target nav goal"),
            ("active_target_nav_goal_to_cmd_vel_ms", metrics["latency.active_target_nav_goal_to_cmd_vel_ms"], "ms", "Active target nav goal to command velocity"),
            ("active_target_nav_goal_to_odom_motion_ms", metrics["latency.active_target_nav_goal_to_odom_motion_ms"], "ms", "Active target nav goal to odom motion"),
        ],
    )
    print(f"SPATIAL_RESPONSE_METRICS=OK {metrics_dir / 'spatial_response_metrics.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

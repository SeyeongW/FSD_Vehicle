#!/usr/bin/env python3
from __future__ import annotations

import csv
import sys
from collections import Counter
from pathlib import Path


def rows(root: Path, name: str) -> list[dict[str, str]]:
    path = root / name
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, "") or default)
    except ValueError:
        return default


def save_or_note(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: plot_paper_metrics.py <experiment_result_dir>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).expanduser().resolve()
    plot_dir = root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        (plot_dir / "PLOTS_NOT_GENERATED.txt").write_text(
            f"matplotlib unavailable: {exc}\n",
            encoding="utf-8",
        )
        return 0

    mission = rows(root, "mission_events.csv")
    pose = rows(root, "robot_pose.csv")
    lidar = rows(root, "lidar_targets.csv")
    goals = rows(root, "inspection_goals.csv")
    cls = rows(root, "camera_classification.csv")
    sound = rows(root, "sound_events.csv")
    safety = rows(root, "safety_state.csv")
    camera = rows(root, "camera_alignment.csv")
    ret = rows(root, "return_to_patrol.csv")

    fig, ax = plt.subplots(figsize=(12, 3))
    states = [r.get("mission_state", "") for r in mission if r.get("mission_state")]
    ax.plot(range(len(states)), [hash(s) % 100 for s in states], marker=".", linewidth=0.8)
    ax.set_title("Mission state timeline")
    ax.set_xlabel("sample")
    ax.set_ylabel("state index")
    save_or_note(fig, plot_dir / "01_mission_state_timeline.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 7))
    xs = [f(r, "x") for r in pose if r.get("x")]
    ys = [f(r, "y") for r in pose if r.get("y")]
    ax.plot(xs, ys, label="robot")
    ax.scatter([f(r, "target_x") for r in goals if r.get("target_x")], [f(r, "target_y") for r in goals if r.get("target_y")], label="LiDAR target")
    ax.scatter([f(r, "goal_x") for r in goals if r.get("goal_x")], [f(r, "goal_y") for r in goals if r.get("goal_y")], label="inspection goal")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Robot trajectory and inspection points")
    ax.legend()
    save_or_note(fig, plot_dir / "02_robot_xy_trajectory.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot([f(r, "height_m") for r in lidar], label="height")
    ax.plot([f(r, "range_m") for r in lidar], label="range")
    ax.plot([f(r, "velocity_mps") for r in lidar], label="velocity")
    ax.set_title("LiDAR target height/range/velocity")
    ax.legend()
    save_or_note(fig, plot_dir / "03_lidar_height_range_velocity.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    samples = [f(r, "candidate_to_goal_latency_ms") for r in goals if r.get("candidate_to_goal_latency_ms")]
    approach = [f(r, "approach_duration_sec") * 1000.0 for r in rows(root, "target_approach.csv") if r.get("approach_duration_sec")]
    ax.boxplot([samples or [0.0], approach or [0.0]], labels=["detect->goal ms", "approach ms"])
    ax.set_title("Detection-to-goal and approach latency")
    save_or_note(fig, plot_dir / "04_latency_boxplot.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot([f(r, "pan_cmd_rad") for r in camera], label="pan")
    ax.plot([f(r, "tilt_cmd_rad") for r in camera], label="tilt")
    ax.plot([f(r, "pointing_error_rad") for r in camera], label="pointing error")
    ax.set_title("Camera alignment")
    ax.legend()
    save_or_note(fig, plot_dir / "05_camera_alignment.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    conf = [f(r, "confidence") for r in cls if r.get("confidence")]
    ax.hist(conf or [0.0], bins=20)
    ax.set_title("Classification confidence histogram")
    save_or_note(fig, plot_dir / "06_classification_confidence_histogram.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    counts = Counter(r.get("target_class", "unknown") or "unknown" for r in cls)
    ax.bar(list(counts.keys()), list(counts.values()))
    ax.set_title("Class distribution")
    save_or_note(fig, plot_dir / "07_class_distribution.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot([1 if str(r.get("sound_alert_request", "")).lower() == "true" else 0 for r in sound], label="request")
    ax.plot([1 if str(r.get("sound_task_done", "")).lower() == "true" else 0 for r in sound], label="done")
    ax.set_title("Sound request/done timeline")
    ax.legend()
    save_or_note(fig, plot_dir / "08_sound_timeline.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    safety_counts = Counter(r.get("safety_state", "UNKNOWN") for r in safety)
    ax.bar(list(safety_counts.keys()), list(safety_counts.values()))
    ax.tick_params(axis="x", rotation=35)
    ax.set_title("Safety state samples")
    save_or_note(fig, plot_dir / "09_safety_state_samples.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    values = [f(r, "return_duration_sec") for r in ret if r.get("return_duration_sec")]
    ax.boxplot(values or [0.0])
    ax.set_title("Return-to-patrol duration")
    save_or_note(fig, plot_dir / "10_return_to_patrol_duration.png")
    plt.close(fig)

    print(f"wrote plots under {plot_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

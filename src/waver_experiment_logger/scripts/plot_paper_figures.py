#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def f(v: str) -> float:
    try:
        return float(v)
    except Exception:
        return 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    plot_dir = run_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    gt = rows(run_dir / "annotations/ground_truth_3d.csv")
    if gt:
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.plot([f(r["x"]) for r in gt], [f(r["y"]) for r in gt], label="bird GT")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / "trajectory_gt_xy.png", dpi=160)
        plt.close(fig)

    assoc = rows(run_dir / "annotations/lidar_gt_association.csv")
    if assoc:
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot([f(r["time_sec"]) for r in assoc], [f(r.get("nearest_dist_m", "0")) for r in assoc])
        ax.set_xlabel("time [s]")
        ax.set_ylabel("LiDAR-GT nearest distance [m]")
        fig.tight_layout()
        fig.savefig(plot_dir / "lidar_gt_error_timeline.png", dpi=160)
        plt.close(fig)

    mission = rows(run_dir / "logs/mission_events.csv")
    if mission:
        states = {}
        ys = []
        xs = []
        for r in mission:
            text = r.get("mission_state", "")
            if text not in states:
                states[text] = len(states)
            xs.append(f(r.get("time_sec", "0")))
            ys.append(states[text])
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.scatter(xs, ys, s=8)
        ax.set_yticks(list(states.values()))
        ax.set_yticklabels(list(states.keys()), fontsize=7)
        ax.set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(plot_dir / "mission_timeline.png", dpi=160)
        plt.close(fig)

    print(f"PAPER_PLOTS=OK {plot_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

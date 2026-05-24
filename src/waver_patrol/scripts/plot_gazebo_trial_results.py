#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Create simple paper-ready plots from Waver Gazebo trial summaries")
    parser.add_argument("--results_dir", default="~/ros2_ws/experiments_result/results")
    args = parser.parse_args()
    results_dir = Path(os.path.expanduser(args.results_dir))
    summary = results_dir / "gazebo_trial_summary.csv"
    if not summary.exists():
        print(f"missing {summary}; run analyze_gazebo_trials.py first")
        return 1
    with summary.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib unavailable, skipping PNG plots: {exc}")
        return 0

    trial_ids = [str(row.get("trial_id", "")) for row in rows]
    heights = [float(row.get("target_object_height_m") or 0.0) for row in rows]
    successes = [1.0 if str(row.get("overall_success", "")).lower() == "true" else 0.0 for row in rows]

    plt.figure(figsize=(7, 4))
    plt.bar(trial_ids, heights)
    plt.axhline(3.0, color="red", linestyle="--", label="3m height threshold")
    plt.xlabel("Trial")
    plt.ylabel("Object height (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "target_height_plot.png")
    plt.close()

    plt.figure(figsize=(7, 4))
    plt.bar(trial_ids, successes)
    plt.ylim(0, 1.1)
    plt.xlabel("Trial")
    plt.ylabel("Overall success")
    plt.tight_layout()
    plt.savefig(results_dir / "mission_success_plot.png")
    plt.close()
    print(f"Wrote plots under {results_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

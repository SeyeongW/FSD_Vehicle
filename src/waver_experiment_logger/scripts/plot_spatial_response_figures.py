#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def f(row: dict[str, str], field: str) -> float:
    try:
        value = float(row.get(field, ""))
        return value if math.isfinite(value) else math.nan
    except Exception:
        return math.nan


def series(table: list[dict[str, str]], x_field: str, y_field: str) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for row in table:
        x = f(row, x_field)
        y = f(row, y_field)
        if math.isfinite(x) and math.isfinite(y):
            xs.append(x)
            ys.append(y)
    return xs, ys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    logs = run_dir / "logs"
    figures = run_dir / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    spatial = rows(logs / "spatial_distance_timeseries.csv")
    bird = rows(logs / "bird_kinematics.csv")
    lidar = rows(logs / "lidar_filter_response.csv")
    goals = rows(logs / "goal_bird_distance_events.csv")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        summary = {
            "plot_status": "SKIPPED_MATPLOTLIB_UNAVAILABLE",
            "reason": str(exc),
            "spatial_rows": len(spatial),
            "bird_rows": len(bird),
            "lidar_rows": len(lidar),
            "goal_rows": len(goals),
        }
        (figures / "plot_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"SPATIAL_RESPONSE_PLOTS=SKIPPED {figures / 'plot_summary.json'}")
        return 0

    if spatial:
        rx, ry = series(spatial, "waver_x_m", "waver_y_m")
        bx, by = series(spatial, "bird_x_m", "bird_y_m")
        gx, gy = series(spatial, "active_nav_goal_x_m", "active_nav_goal_y_m")
        lx, ly = series(spatial, "lidar_target_x_m", "lidar_target_y_m")
        plt.figure(figsize=(7, 7))
        if rx:
            plt.plot(rx, ry, label="Waver odom", linewidth=2)
        if bx:
            plt.scatter(bx, by, label="Bird", s=12)
        if lx:
            plt.scatter(lx, ly, label="LiDAR target", s=12)
        if gx:
            plt.scatter(gx, gy, label="Active goal", s=24, marker="x")
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.tight_layout()
        plt.savefig(figures / "spatial_xy_trajectory.png", dpi=180)
        plt.close()

        tx, dist = series(spatial, "time_sec", "robot_to_bird_xy_m")
        tx2, ldist = series(spatial, "time_sec", "lidar_target_to_bird_xy_m")
        if tx or tx2:
            plt.figure(figsize=(9, 4))
            if tx:
                t0 = tx[0]
                plt.plot([v - t0 for v in tx], dist, label="Waver to bird")
            if tx2:
                t02 = tx2[0]
                plt.plot([v - t02 for v in tx2], ldist, label="LiDAR target to bird")
            plt.grid(True)
            plt.xlabel("time [s]")
            plt.ylabel("distance [m]")
            plt.legend()
            plt.tight_layout()
            plt.savefig(figures / "distance_timeseries.png", dpi=180)
            plt.close()

    if lidar:
        tx, runtime = series(lidar, "filter_done_time_sec", "filter_runtime_wall_ms")
        if tx and runtime:
            t0 = tx[0]
            plt.figure(figsize=(9, 4))
            plt.plot([v - t0 for v in tx], runtime)
            plt.grid(True)
            plt.xlabel("time [s]")
            plt.ylabel("LiDAR filter wall runtime [ms]")
            plt.tight_layout()
            plt.savefig(figures / "lidar_filter_wall_runtime.png", dpi=180)
            plt.close()

    summary = {
        "plot_status": "OK",
        "spatial_rows": len(spatial),
        "bird_rows": len(bird),
        "lidar_rows": len(lidar),
        "goal_rows": len(goals),
    }
    (figures / "plot_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"SPATIAL_RESPONSE_PLOTS=OK {figures}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

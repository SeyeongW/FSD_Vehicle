#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    frame_rows = rows(run_dir / "annotations/frame_index.csv")
    gt_rows = rows(run_dir / "annotations/ground_truth_3d.csv")
    assoc_rows = rows(run_dir / "annotations/lidar_gt_association.csv")
    mission_rows = rows(run_dir / "logs/mission_events.csv")
    classifier_rows = rows(run_dir / "logs/classifier_events.csv")
    sound_rows = rows(run_dir / "logs/sound_events.csv")
    removal_rows = rows(run_dir / "logs/bird_removal_events.csv")

    valid_frames = [r for r in frame_rows if str(r.get("gt_valid", "")).lower() == "true"]
    associated = [r for r in assoc_rows if str(r.get("associated", "")).lower() == "true"]
    bird_confirmed = [r for r in classifier_rows if str(r.get("bird_confirmed", "")).lower() == "true"]
    sound_done = [r for r in sound_rows if str(r.get("done", "")).lower() == "true"]
    removed_birds = {
        r.get("bird_name", "")
        for r in removal_rows
        if str(r.get("state", "")).startswith("REMOVED") and r.get("bird_name", "")
    }
    mission_text = " ".join(" ".join(r.values()) for r in mission_rows)
    metrics = {
        "frames": len(frame_rows),
        "valid_projected_frames": len(valid_frames),
        "gt_rows": len(gt_rows),
        "lidar_associated_rows": len(associated),
        "mission_events": len(mission_rows),
        "bird_confirmed_rows": len(bird_confirmed),
        "sound_done_rows": len(sound_done),
        "removed_bird_count": len(removed_birds),
        "two_bird_removal_success": len(removed_birds) >= 2,
        "has_patrol": "PATROL_NAVIGATING" in mission_text,
        "has_offset_approach": "APPROACH_TARGET_OFFSET" in mission_text,
        "has_sound_task": "SOUND_TASK" in mission_text,
    }
    out_json = run_dir / "metrics/paper_metrics_summary.json"
    out_csv = run_dir / "metrics/paper_metrics_summary.csv"
    out_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "value"])
        writer.writeheader()
        for k, v in metrics.items():
            writer.writerow({"metric": k, "value": v})
    print(f"PAPER_METRICS=OK {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

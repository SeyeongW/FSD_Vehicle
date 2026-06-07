#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


REQUIRED_FILES = [
    "metadata/run_metadata.json",
    "metadata/launch_args.yaml",
    "metadata/parameter_snapshot.yaml",
    "metadata/topic_info.txt",
    "metadata/topic_hz.csv",
    "metadata/replay_manifest.yaml",
    "annotations/frame_index.csv",
    "annotations/ground_truth_3d.csv",
    "annotations/camera_projection_debug.csv",
    "annotations/lidar_gt_association.csv",
    "annotations/coco_instances.json",
    "logs/mission_events.csv",
    "logs/lidar_detections.csv",
    "logs/camera_alignment.csv",
    "logs/classifier_events.csv",
    "logs/sound_events.csv",
    "logs/bird_removal_events.csv",
    "logs/safety_check.csv",
    "logs/experiment_summary.csv",
    "metrics/paper_metrics_summary.csv",
    "metrics/paper_metrics_summary.json",
]


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--min-images", type=int, default=1)
    parser.add_argument("--require-annotations", action="store_true")
    parser.add_argument("--min-removed-birds", type=int, default=0)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    failures: list[str] = []
    for rel in REQUIRED_FILES:
        p = run_dir / rel
        if not p.exists():
            failures.append(f"MISSING_FILE {rel}")
        elif p.stat().st_size <= 0:
            failures.append(f"EMPTY_FILE {rel}")

    metadata = {}
    try:
        metadata = json.loads((run_dir / "metadata/run_metadata.json").read_text())
    except Exception as exc:
        failures.append(f"BAD_METADATA {exc}")
    detector_mode = str(metadata.get("detector_mode", "")).lower()

    frames = read_rows(run_dir / "annotations/frame_index.csv")
    images = list((run_dir / "images/pt_camera").glob("*.png"))
    if len(images) < args.min_images:
        failures.append(f"IMAGE_COUNT_LOW count={len(images)} min={args.min_images}")

    valid_frames = [r for r in frames if str(r.get("gt_valid", "")).lower() == "true"]
    if args.require_annotations and not valid_frames:
        failures.append("NO_VALID_CAMERA_PROJECTIONS")

    gt_rows = read_rows(run_dir / "annotations/ground_truth_3d.csv")
    if not gt_rows:
        failures.append("NO_GT_ROWS")
    if detector_mode == "lidar":
        leaked = [r for r in gt_rows if str(r.get("used_for_decision", "")).lower() == "true"]
        if leaked:
            failures.append(f"GT_LEAKAGE_IN_LIDAR_MODE rows={len(leaked)}")

    mission = read_rows(run_dir / "logs/mission_events.csv")
    mission_text = " ".join(" ".join(r.values()) for r in mission)
    required_tokens = ["PATROL_NAVIGATING", "APPROACH_TARGET_OFFSET"]
    if args.min_removed_birds <= 0:
        required_tokens.append("SOUND_TASK")
    else:
        required_tokens.extend(["RETURN_TO_INTERRUPTED_WAYPOINT", "RESUME_PATROL"])
    for token in required_tokens:
        if token not in mission_text:
            failures.append(f"MISSION_TOKEN_MISSING {token}")

    coco = {}
    try:
        coco = json.loads((run_dir / "annotations/coco_instances.json").read_text())
    except Exception as exc:
        failures.append(f"BAD_COCO {exc}")
    if args.require_annotations and not coco.get("annotations"):
        failures.append("COCO_ANNOTATIONS_EMPTY")

    if args.min_removed_birds > 0:
        removal_rows = read_rows(run_dir / "logs/bird_removal_events.csv")
        removed_names = {
            row.get("bird_name", "")
            for row in removal_rows
            if str(row.get("state", "")).startswith("REMOVED") and row.get("bird_name", "")
        }
        if len(removed_names) < args.min_removed_birds:
            failures.append(
                f"REMOVED_BIRD_COUNT_LOW count={len(removed_names)} min={args.min_removed_birds}"
            )

    if failures:
        print("DATASET_INTEGRITY=FAIL")
        for failure in failures:
            print(failure)
        return 1
    print(
        f"DATASET_INTEGRITY=PASS run_dir={run_dir} detector_mode={detector_mode} "
        f"images={len(images)} valid_projected_frames={len(valid_frames)} gt_rows={len(gt_rows)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def truth(value: object) -> bool:
    return str(value).strip().lower() == "true"


def main() -> int:
    # 역할: CSV summary를 읽어 실차 전 PASS/FAIL Markdown 판정서를 만든다.
    parser = argparse.ArgumentParser(description="Generate Waver pre-real validation Markdown report")
    parser.add_argument("--results_dir", default="~/ugv_ws/FSD_Vehicle/experiments_result/results")
    parser.add_argument("--output", default="~/ugv_ws/FSD_Vehicle/experiments_result/results/final_pass_fail_report.md")
    args = parser.parse_args()
    results_dir = Path(args.results_dir).expanduser()
    output = Path(args.output).expanduser()
    summary = results_dir / "gazebo_trial_summary.csv"
    rows: list[dict[str, str]] = []
    if summary.exists():
        with summary.open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    total = len(rows)
    success = sum(1 for row in rows if truth(row.get("overall_success")))
    height_gate_success = success >= 3
    safety_ok = total > 0 and all(truth(row.get("safety_gate_pass", "true")) for row in rows if truth(row.get("overall_success")))
    decision = "PASS" if height_gate_success and safety_ok else "FAIL"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        f.write("# Waver Pre-Real Gazebo Validation Report\n\n")
        f.write(f"- Decision: **{decision}**\n")
        f.write(f"- Successful trials: {success}/{total}\n")
        f.write(f"- Height H1/H2/H3 gate: {height_gate_success}\n")
        f.write(f"- Safety gate on successful trials: {safety_ok}\n\n")
        f.write("## Trial Table\n\n")
        f.write("| trial | success | height_m | height_pass | dynamic_pass | elevated_valid | map_tf | target_goal | yaw | camera | sound | resume | safety | failure |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")
        for row in rows:
            f.write(
                "| {trial_id} | {overall_success} | {target_object_height_m} | {height_filter_pass} | "
                "{dynamic_filter_pass} | {elevated_dynamic_target_valid} | {map_transform_success} | "
                "{target_goal_success} | {yaw_alignment_success} | {camera_detection_success} | "
                "{sound_mission_success} | {patrol_resume_success} | {safety_gate_pass} | {failure_reason} |\n".format(
                    **{key: row.get(key, "") for key in [
                        "trial_id", "overall_success", "target_object_height_m", "height_filter_pass",
                        "dynamic_filter_pass", "elevated_dynamic_target_valid", "map_transform_success",
                        "target_goal_success", "yaw_alignment_success", "camera_detection_success",
                        "sound_mission_success", "patrol_resume_success", "safety_gate_pass", "failure_reason",
                    ]}
                )
            )
        f.write("\n## Real-Robot Gate\n\n")
        f.write(
            "This report is a simulation gate only. Real Waver deployment still requires wheel-off tests, "
            "physical E-stop, live /scan, TF, localization, single /cmd_vel publisher verification, "
            "and sound output disabled until separately approved.\n"
        )
    print(f"Wrote {output}")
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())

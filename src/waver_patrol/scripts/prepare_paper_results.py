#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Iterable


HEIGHT_FIELDS = [
    "source_run",
    "source_summary",
    "sequence_index",
    "trial_id",
    "scenario",
    "dynamic_motion_m",
    "target_min_height_m",
    "target_object_height_m",
    "z_valid",
    "height_filter_pass",
    "dynamic_filter_pass",
    "elevated_dynamic_target_valid",
    "classification",
    "expected_elevated_dynamic_target_valid",
    "expected_classification",
    "compensated_motion_m",
    "compensated_velocity_mps",
    "target_detected",
    "cluster_published",
    "map_transform_success",
    "moving_target_valid",
    "target_goal_success",
    "yaw_alignment_success",
    "camera_detection_success",
    "sound_mission_success",
    "patrol_resume_success",
    "safety_gate_pass",
    "overall_success",
    "failure_reason",
]

UI_FIELDS = [
    "source_run",
    "source_summary",
    "time_sec",
    "trial_id",
    "map_received",
    "map_mode",
    "slam_live",
    "map_fixed",
    "robot_pose_visible",
    "robot_yaw_visible",
    "global_path_visible",
    "local_path_visible",
    "waypoint_visible",
    "active_goal_visible",
    "object_goal_visible",
    "elevated_target_visible",
    "mission_state_visible",
    "safety_state_visible",
    "camera_state_visible",
    "sound_state_visible",
    "ui_command_panel_alive",
    "ui_direct_cmd_vel_disabled",
    "manual_cmd_seen",
    "mission_command_seen",
    "operator_command_seen",
    "emergency_stop_seen",
    "cmd_vel_publishers",
    "overall_ui_success",
]


def truth(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def as_float(value: object, default: float = math.nan) -> float:
    try:
        text = str(value).strip()
        return float(text) if text else default
    except Exception:
        return default


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, fields: list[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def normalize_row(row: dict[str, str], summary: Path, root: Path, fields: list[str]) -> dict[str, object]:
    out: dict[str, object] = {field: row.get(field, "") for field in fields}
    try:
        run_name = summary.parent.relative_to(root).parts[0]
    except Exception:
        run_name = summary.parent.name
    out["source_run"] = run_name
    out["source_summary"] = str(summary)
    return out


def find_height_rows(root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for summary in sorted(root.glob("pre_real_height_*/experiment_summary.csv")):
        for row in read_csv(summary):
            rows.append(normalize_row(row, summary, root, HEIGHT_FIELDS))
    return rows


def find_ui_rows(root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    summaries = list(root.glob("ui_validation_*/experiment_summary.csv"))
    if not summaries:
        summaries = list(root.glob("ui_validation_*/csv/ui_visualization_check.csv"))
    for summary in sorted(summaries):
        for row in read_csv(summary):
            rows.append(normalize_row(row, summary, root, UI_FIELDS))
    return rows


def latest_by_key(rows: list[dict[str, object]], key_field: str) -> list[dict[str, object]]:
    latest: dict[str, dict[str, object]] = {}
    for row in rows:
        key = str(row.get(key_field, "")).strip() or str(row.get("trial_id", "")).strip()
        path = Path(str(row.get("source_summary", "")))
        stamp = path.stat().st_mtime if path.exists() else 0.0
        previous = latest.get(key)
        if previous is None:
            row["_mtime"] = stamp
            latest[key] = row
            continue
        if stamp >= float(previous.get("_mtime", 0.0)):
            row["_mtime"] = stamp
            latest[key] = row
    return [latest[key] for key in sorted(latest)]


def metric_rows(height_rows: list[dict[str, object]], ui_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    def ratio(num: int, den: int) -> float:
        return round(num / den, 4) if den else 0.0

    height_total = len(height_rows)
    height_success = sum(1 for row in height_rows if truth(row.get("overall_success")))
    ui_total = len(ui_rows)
    ui_success = sum(1 for row in ui_rows if truth(row.get("overall_ui_success")))

    actual_valid = [row for row in height_rows if truth(row.get("expected_elevated_dynamic_target_valid"))]
    predicted_valid = [row for row in height_rows if truth(row.get("elevated_dynamic_target_valid"))]
    true_positive = [
        row for row in height_rows
        if truth(row.get("expected_elevated_dynamic_target_valid"))
        and truth(row.get("elevated_dynamic_target_valid"))
    ]
    false_positive = [
        row for row in height_rows
        if not truth(row.get("expected_elevated_dynamic_target_valid"))
        and truth(row.get("elevated_dynamic_target_valid"))
    ]
    false_negative = [
        row for row in height_rows
        if truth(row.get("expected_elevated_dynamic_target_valid"))
        and not truth(row.get("elevated_dynamic_target_valid"))
    ]

    height_correct = 0
    height_known = 0
    dynamic_correct = 0
    dynamic_known = 0
    static_high_total = 0
    static_high_fp = 0
    low_dynamic_total = 0
    low_dynamic_fp = 0
    mission_trigger_success = 0
    for row in height_rows:
        z_valid = truth(row.get("z_valid"))
        if z_valid:
            height_known += 1
            min_height = as_float(row.get("target_min_height_m"), 3.0)
            height = as_float(row.get("target_object_height_m"))
            expected_height = height >= min_height
            if truth(row.get("height_filter_pass")) == expected_height:
                height_correct += 1
        expected_class = str(row.get("expected_classification", ""))
        if expected_class:
            dynamic_known += 1
            expected_dynamic = expected_class in {"elevated_dynamic_object", "low_altitude_object"}
            if truth(row.get("dynamic_filter_pass")) == expected_dynamic:
                dynamic_correct += 1
        scenario = str(row.get("scenario", ""))
        if "elevated_static" in scenario:
            static_high_total += 1
            if truth(row.get("elevated_dynamic_target_valid")):
                static_high_fp += 1
        if "low_altitude_dynamic" in scenario:
            low_dynamic_total += 1
            if truth(row.get("elevated_dynamic_target_valid")):
                low_dynamic_fp += 1
        if truth(row.get("expected_elevated_dynamic_target_valid")) and truth(row.get("target_goal_success")):
            mission_trigger_success += 1

    metrics = [
        ("height_trial_count", height_total),
        ("height_trial_success_count", height_success),
        ("gazebo_simulation_success_rate", ratio(height_success, height_total)),
        ("height_filter_accuracy", ratio(height_correct, height_known)),
        ("dynamic_classification_accuracy", ratio(dynamic_correct, dynamic_known)),
        ("elevated_dynamic_target_precision", ratio(len(true_positive), len(predicted_valid))),
        ("elevated_dynamic_target_recall", ratio(len(true_positive), len(actual_valid))),
        ("elevated_dynamic_target_false_positive_count", len(false_positive)),
        ("elevated_dynamic_target_false_negative_count", len(false_negative)),
        ("static_elevated_false_positive_rate", ratio(static_high_fp, static_high_total)),
        ("low_altitude_dynamic_false_positive_rate", ratio(low_dynamic_fp, low_dynamic_total)),
        ("target_mission_trigger_success_rate", ratio(mission_trigger_success, len(actual_valid))),
        ("ui_trial_count", ui_total),
        ("ui_success_count", ui_success),
        ("ui_visualization_success_rate", ratio(ui_success, ui_total)),
        (
            "ui_direct_cmd_vel_disabled_rate",
            ratio(sum(1 for row in ui_rows if truth(row.get("ui_direct_cmd_vel_disabled"))), ui_total),
        ),
    ]
    return [{"metric": name, "value": value} for name, value in metrics]


def copy_selected_sources(rows: list[dict[str, object]], dst: Path) -> list[dict[str, object]]:
    manifest: list[dict[str, object]] = []
    dst.mkdir(parents=True, exist_ok=True)
    for index, row in enumerate(rows, start=1):
        src = Path(str(row.get("source_summary", "")))
        if not src.exists():
            continue
        name = f"{index:02d}_{row.get('source_run', src.parent.name)}_{src.name}"
        target = dst / sanitize_filename(name)
        shutil.copyfile(src, target)
        manifest.append(
            {
                "artifact_type": "selected_source_csv",
                "source": str(src),
                "paper_copy": str(target),
            }
        )
    return manifest


def sanitize_filename(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name)


def write_report(
    path: Path,
    selected_height: list[dict[str, object]],
    selected_ui: list[dict[str, object]],
    metrics: list[dict[str, object]],
) -> None:
    metrics_map = {str(row["metric"]): row["value"] for row in metrics}
    decision = (
        "PASS"
        if float(metrics_map.get("gazebo_simulation_success_rate", 0.0)) >= 1.0
        and float(metrics_map.get("ui_visualization_success_rate", 0.0)) >= 1.0
        and int(metrics_map.get("elevated_dynamic_target_false_positive_count", 1)) == 0
        and int(metrics_map.get("elevated_dynamic_target_false_negative_count", 1)) == 0
        else "REVIEW"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# Paper-Ready Waver Experiment Results\n\n")
        f.write(f"- Prepared at: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"- Decision from selected data: **{decision}**\n")
        f.write("- Target rule: height >= 3.0 m, z_valid=true, dynamic_filter_pass=true, ego-motion compensated.\n")
        f.write("- Note: 3 m is a height threshold, not a motion-distance threshold.\n\n")
        f.write("## Selected Height Trials\n\n")
        f.write("| trial | scenario | height_m | height_pass | dynamic_pass | elevated_valid | class | success |\n")
        f.write("|---|---|---:|---|---|---|---|---|\n")
        for row in selected_height:
            f.write(
                f"| {row.get('trial_id','')} | {row.get('scenario','')} | "
                f"{row.get('target_object_height_m','')} | {row.get('height_filter_pass','')} | "
                f"{row.get('dynamic_filter_pass','')} | {row.get('elevated_dynamic_target_valid','')} | "
                f"{row.get('classification','')} | {row.get('overall_success','')} |\n"
            )
        f.write("\n## Selected UI Trials\n\n")
        f.write("| trial | mode | map | robot | global path | local path | direct cmd disabled | success |\n")
        f.write("|---|---|---|---|---|---|---|---|\n")
        for row in selected_ui:
            f.write(
                f"| {row.get('trial_id','')} | {row.get('map_mode','')} | "
                f"{row.get('map_received','')} | {row.get('robot_pose_visible','')} | "
                f"{row.get('global_path_visible','')} | {row.get('local_path_visible','')} | "
                f"{row.get('ui_direct_cmd_vel_disabled','')} | {row.get('overall_ui_success','')} |\n"
            )
        f.write("\n## Metrics\n\n")
        f.write("| metric | value |\n")
        f.write("|---|---:|\n")
        for row in metrics:
            f.write(f"| {row['metric']} | {row['value']} |\n")


def make_plots(tables_dir: Path, figures_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib unavailable, skipping paper plots: {exc}")
        return
    figures_dir.mkdir(parents=True, exist_ok=True)

    selected = tables_dir / "height_target_trials_selected.csv"
    if selected.exists():
        rows = read_csv(selected)
        labels = [str(row.get("trial_id") or row.get("scenario")) for row in rows]
        heights = [as_float(row.get("target_object_height_m"), 0.0) for row in rows]
        valid = [1.0 if truth(row.get("elevated_dynamic_target_valid")) else 0.0 for row in rows]
        plt.figure(figsize=(7, 4))
        plt.bar(labels, heights, color="#4c78a8")
        plt.axhline(3.0, color="#e45756", linestyle="--", label="3.0 m threshold")
        plt.xlabel("Trial")
        plt.ylabel("Object height (m)")
        plt.legend()
        plt.tight_layout()
        plt.savefig(figures_dir / "paper_target_height_by_trial.png", dpi=180)
        plt.close()

        plt.figure(figsize=(7, 4))
        plt.bar(labels, valid, color="#54a24b")
        plt.ylim(0, 1.1)
        plt.xlabel("Trial")
        plt.ylabel("Elevated dynamic target valid")
        plt.tight_layout()
        plt.savefig(figures_dir / "paper_target_validity_by_trial.png", dpi=180)
        plt.close()

    ui = tables_dir / "ui_visualization_selected.csv"
    if ui.exists():
        rows = read_csv(ui)
        labels = [str(row.get("trial_id")) for row in rows]
        success = [1.0 if truth(row.get("overall_ui_success")) else 0.0 for row in rows]
        plt.figure(figsize=(7, 4))
        plt.bar(labels, success, color="#f58518")
        plt.ylim(0, 1.1)
        plt.xlabel("UI Trial")
        plt.ylabel("UI success")
        plt.tight_layout()
        plt.savefig(figures_dir / "paper_ui_success_by_trial.png", dpi=180)
        plt.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare paper-ready Waver experiment tables, plots, and report")
    parser.add_argument("--input-dir", default="~/ros2_ws5/FSD_Vehicle/experiments_result")
    parser.add_argument("--output-root", default="~/ros2_ws5/FSD_Vehicle/experiments_result/paper_ready")
    parser.add_argument("--name", default="")
    parser.add_argument("--no-latest", action="store_true", help="Do not update paper_ready/latest symlink")
    args = parser.parse_args()

    input_dir = Path(os.path.expanduser(args.input_dir))
    output_root = Path(os.path.expanduser(args.output_root))
    run_name = args.name.strip() or datetime.now().strftime("paper_%Y%m%d_%H%M%S")
    out = output_root / run_name
    tables_dir = out / "tables"
    figures_dir = out / "figures"
    reports_dir = out / "reports"
    raw_dir = out / "raw_selected"

    height_all = find_height_rows(input_dir)
    ui_all = find_ui_rows(input_dir)
    height_selected = latest_by_key(height_all, "scenario")
    ui_selected = latest_by_key(ui_all, "trial_id")
    metrics = metric_rows(height_selected, ui_selected)

    write_csv(tables_dir / "height_target_trials_all.csv", HEIGHT_FIELDS, height_all)
    write_csv(tables_dir / "height_target_trials_selected.csv", HEIGHT_FIELDS, height_selected)
    write_csv(tables_dir / "ui_visualization_all.csv", UI_FIELDS, ui_all)
    write_csv(tables_dir / "ui_visualization_selected.csv", UI_FIELDS, ui_selected)
    write_csv(tables_dir / "paper_metrics.csv", ["metric", "value"], metrics)

    manifest = copy_selected_sources(height_selected + ui_selected, raw_dir)
    write_csv(out / "source_manifest.csv", ["artifact_type", "source", "paper_copy"], manifest)
    write_report(reports_dir / "paper_results_summary.md", height_selected, ui_selected, metrics)
    make_plots(tables_dir, figures_dir)

    latest = output_root / "latest"
    if not args.no_latest:
        try:
            if latest.is_symlink() or latest.exists():
                latest.unlink()
            latest.symlink_to(out, target_is_directory=True)
        except Exception:
            pass

    print(f"Paper-ready results prepared under: {out}")
    print(f"Selected height trials: {len(height_selected)} / all rows: {len(height_all)}")
    print(f"Selected UI trials: {len(ui_selected)} / all rows: {len(ui_all)}")
    return 0 if height_selected else 1


if __name__ == "__main__":
    raise SystemExit(main())

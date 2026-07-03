#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import statistics
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Iterable


REQUIRED_SCENARIOS_DEFAULT = (
    "H1_elevated_dynamic",
    "H2_elevated_static",
    "H3_low_altitude_dynamic",
)

BASE_HEIGHT_FIELDS = [
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

EVIDENCE_FIELDS = [
    "sim_or_real",
    "fake_detector_used",
    "fake_sound_used",
    "serial_enabled",
    "ground_truth_source",
    "evidence_level",
]

HEIGHT_FIELDS = BASE_HEIGHT_FIELDS + EVIDENCE_FIELDS

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
] + EVIDENCE_FIELDS


def truth(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y", "pass", "passed"}


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
    out["sim_or_real"] = row.get("sim_or_real") or "sim"
    out["fake_detector_used"] = row.get("fake_detector_used") or "UNKNOWN"
    out["fake_sound_used"] = row.get("fake_sound_used") or "UNKNOWN"
    out["serial_enabled"] = row.get("serial_enabled") or "false"
    out["ground_truth_source"] = row.get("ground_truth_source") or "N/A"
    out["evidence_level"] = row.get("evidence_level") or "L2_GAZEBO_FUNCTIONAL"
    return out


def _candidate_summaries(root: Path, names: tuple[str, ...]) -> list[Path]:
    blocked = {"paper_ready", "results", "bags", "rosbag", "rosbags"}
    summaries: list[Path] = []
    for path in sorted(root.rglob("*.csv")):
        if set(path.relative_to(root).parts) & blocked:
            continue
        if path.name in names:
            summaries.append(path)
    return summaries


def find_height_rows(root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for summary in _candidate_summaries(root, ("experiment_summary.csv",)):
        try:
            csv_rows = read_csv(summary)
        except Exception:
            continue
        for row in csv_rows:
            if row.get("scenario") or row.get("expected_elevated_dynamic_target_valid") or row.get("target_object_height_m"):
                rows.append(normalize_row(row, summary, root, HEIGHT_FIELDS))
    return rows


def find_ui_rows(root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for summary in _candidate_summaries(root, ("ui_visualization_check.csv", "experiment_summary.csv")):
        try:
            csv_rows = read_csv(summary)
        except Exception:
            continue
        for row in csv_rows:
            if row.get("overall_ui_success") or row.get("map_received") or row.get("ui_command_panel_alive"):
                rows.append(normalize_row(row, summary, root, UI_FIELDS))
    return rows


def scenario_key(row: dict[str, object]) -> str:
    text = str(row.get("scenario", "")).strip()
    for scenario in REQUIRED_SCENARIOS_DEFAULT:
        if text == scenario or text.startswith(scenario) or scenario in text:
            return scenario
    if text:
        return text
    trial_id = str(row.get("trial_id", "")).strip()
    return trial_id or "UNKNOWN"


def ratio(num: int, den: int) -> float | str:
    return round(num / den, 4) if den else "N/A"


def summary_stats(values: list[float]) -> dict[str, object]:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return {"n": 0, "mean": "N/A", "std": "N/A", "min": "N/A", "max": "N/A", "ci95": "N/A"}
    mean = statistics.fmean(finite)
    std = statistics.stdev(finite) if len(finite) >= 2 else 0.0
    ci = 1.96 * std / math.sqrt(len(finite)) if len(finite) >= 2 else "N/A"
    return {"n": len(finite), "mean": mean, "std": std, "min": min(finite), "max": max(finite), "ci95": ci}


def latency_fields(rows: list[dict[str, object]]) -> list[str]:
    fields: set[str] = set()
    for row in rows:
        for key in row:
            low = key.lower()
            if "latency" in low or low.endswith("_sec") or low.endswith("_s"):
                if key not in {"source_run", "source_summary"}:
                    fields.add(key)
    return sorted(fields)


def scenario_summary_rows(height_rows: list[dict[str, object]], required: list[str]) -> list[dict[str, object]]:
    by_scenario: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in height_rows:
        by_scenario[scenario_key(row)].append(row)
    rows: list[dict[str, object]] = []
    for scenario in required:
        group = by_scenario.get(scenario, [])
        success = sum(1 for row in group if truth(row.get("overall_success")))
        failures = Counter(str(row.get("failure_reason", "") or "none") for row in group if not truth(row.get("overall_success")))
        rows.append(
            {
                "scenario": scenario,
                "n_total": len(group),
                "n_success": success,
                "success_rate": ratio(success, len(group)),
                "failure_reasons": ";".join(f"{k}:{v}" for k, v in sorted(failures.items())),
            }
        )
    return rows


def confusion_rows(height_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    labels = ("expected_true_predicted_true", "expected_true_predicted_false", "expected_false_predicted_true", "expected_false_predicted_false")
    counts = dict.fromkeys(labels, 0)
    for row in height_rows:
        expected = truth(row.get("expected_elevated_dynamic_target_valid"))
        predicted = truth(row.get("elevated_dynamic_target_valid"))
        if expected and predicted:
            counts["expected_true_predicted_true"] += 1
        elif expected and not predicted:
            counts["expected_true_predicted_false"] += 1
        elif not expected and predicted:
            counts["expected_false_predicted_true"] += 1
        else:
            counts["expected_false_predicted_false"] += 1
    return [{"cell": key, "count": value} for key, value in counts.items()]


def failure_reason_rows(height_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    counts: Counter[str] = Counter()
    for row in height_rows:
        if not truth(row.get("overall_success")):
            counts[str(row.get("failure_reason", "") or "unknown")] += 1
    return [{"failure_reason": key, "count": value} for key, value in sorted(counts.items())] or [{"failure_reason": "none", "count": 0}]


def evidence_summary_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    counter: Counter[tuple[str, str, str, str, str, str]] = Counter()
    for row in rows:
        counter[
            (
                str(row.get("evidence_level", "UNKNOWN")),
                str(row.get("sim_or_real", "UNKNOWN")),
                str(row.get("fake_detector_used", "UNKNOWN")),
                str(row.get("fake_sound_used", "UNKNOWN")),
                str(row.get("serial_enabled", "UNKNOWN")),
                str(row.get("ground_truth_source", "N/A")),
            )
        ] += 1
    return [
        {
            "evidence_level": key[0],
            "sim_or_real": key[1],
            "fake_detector_used": key[2],
            "fake_sound_used": key[3],
            "serial_enabled": key[4],
            "ground_truth_source": key[5],
            "count": count,
        }
        for key, count in sorted(counter.items())
    ]


def metric_rows(height_rows: list[dict[str, object]], ui_rows: list[dict[str, object]], required: list[str]) -> list[dict[str, object]]:
    total = len(height_rows)
    success = sum(1 for row in height_rows if truth(row.get("overall_success")))
    by_scenario = {row["scenario"]: row for row in scenario_summary_rows(height_rows, required)}

    actual_valid = [row for row in height_rows if truth(row.get("expected_elevated_dynamic_target_valid"))]
    predicted_valid = [row for row in height_rows if truth(row.get("elevated_dynamic_target_valid"))]
    true_positive = [row for row in height_rows if truth(row.get("expected_elevated_dynamic_target_valid")) and truth(row.get("elevated_dynamic_target_valid"))]
    false_positive = [row for row in height_rows if not truth(row.get("expected_elevated_dynamic_target_valid")) and truth(row.get("elevated_dynamic_target_valid"))]

    h2_rows = [row for row in height_rows if scenario_key(row) == "H2_elevated_static"]
    h3_rows = [row for row in height_rows if scenario_key(row) == "H3_low_altitude_dynamic"]
    ui_success = sum(1 for row in ui_rows if truth(row.get("overall_ui_success")))

    safety_gate_total = sum(1 for row in height_rows if str(row.get("safety_gate_pass", "")).strip() != "")
    safety_gate_success = sum(1 for row in height_rows if truth(row.get("safety_gate_pass")))
    mission_trigger_success = sum(1 for row in actual_valid if truth(row.get("target_goal_success")))

    metrics: list[dict[str, object]] = [
        {"metric": "n_total", "value": total, "n": total, "notes": ""},
        {"metric": "n_by_scenario", "value": ";".join(f"{s}:{by_scenario.get(s, {}).get('n_total', 0)}" for s in required), "n": total, "notes": ""},
        {"metric": "overall_success_rate", "value": ratio(success, total), "n": total, "notes": ""},
        {"metric": "elevated_dynamic_target_precision", "value": ratio(len(true_positive), len(predicted_valid)), "n": len(predicted_valid), "notes": "N/A - no predictions" if not predicted_valid else ""},
        {"metric": "elevated_dynamic_target_recall", "value": ratio(len(true_positive), len(actual_valid)), "n": len(actual_valid), "notes": "N/A - no expected positives" if not actual_valid else ""},
        {"metric": "false_positive_rate_H2_elevated_static", "value": ratio(sum(1 for r in h2_rows if truth(r.get("elevated_dynamic_target_valid"))), len(h2_rows)), "n": len(h2_rows), "notes": ""},
        {"metric": "false_positive_rate_H3_low_altitude_dynamic", "value": ratio(sum(1 for r in h3_rows if truth(r.get("elevated_dynamic_target_valid"))), len(h3_rows)), "n": len(h3_rows), "notes": ""},
        {"metric": "target_mission_trigger_success_rate", "value": ratio(mission_trigger_success, len(actual_valid)), "n": len(actual_valid), "notes": ""},
        {"metric": "safety_gate_pass_rate", "value": ratio(safety_gate_success, safety_gate_total), "n": safety_gate_total, "notes": "N/A - no safety gate column" if not safety_gate_total else ""},
        {"metric": "ui_visualization_success_rate", "value": ratio(ui_success, len(ui_rows)), "n": len(ui_rows), "notes": "N/A - no UI rows" if not ui_rows else ""},
    ]
    for scenario, row in by_scenario.items():
        metrics.append({"metric": f"success_rate_by_scenario_{scenario}", "value": row["success_rate"], "n": row["n_total"], "notes": ""})

    for field in latency_fields(height_rows):
        stats = summary_stats([as_float(row.get(field)) for row in height_rows])
        metrics.append({"metric": f"latency_{field}", "value": stats["mean"], **stats, "notes": "CI=N/A for n<2" if stats["ci95"] == "N/A" else ""})

    has_external_gt = any(str(row.get("ground_truth_source", "")).strip() not in {"", "N/A", "UNKNOWN"} for row in height_rows)
    if not has_external_gt:
        metrics.extend(
            [
                {"metric": "classification_precision", "value": "N/A", "n": 0, "notes": "no external ground truth"},
                {"metric": "classification_recall", "value": "N/A", "n": 0, "notes": "no external ground truth"},
                {"metric": "classification_mAP", "value": "N/A", "n": 0, "notes": "no external ground truth"},
            ]
        )
    return metrics


def copy_selected_sources(rows: list[dict[str, object]], dst: Path) -> list[dict[str, object]]:
    manifest: list[dict[str, object]] = []
    dst.mkdir(parents=True, exist_ok=True)
    seen: set[Path] = set()
    for index, row in enumerate(rows, start=1):
        src = Path(str(row.get("source_summary", "")))
        if not src.exists() or src in seen:
            continue
        seen.add(src)
        target = dst / sanitize_filename(f"{index:03d}_{row.get('source_run', src.parent.name)}_{src.name}")
        shutil.copyfile(src, target)
        manifest.append({"artifact_type": "source_csv", "source": str(src), "paper_copy": str(target)})
    return manifest


def sanitize_filename(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name)


def write_report(path: Path, status: str, reasons: list[str], metrics: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# Paper Result Preparation Report\n\n")
        f.write(f"- Prepared at: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"- Status: **{status}**\n\n")
        f.write("## Gate Reasons\n\n")
        for reason in reasons or ["none"]:
            f.write(f"- {reason}\n")
        f.write("\n## Metrics\n\n")
        f.write("| metric | value | n | notes |\n|---|---:|---:|---|\n")
        for row in metrics:
            f.write(f"| {row.get('metric')} | {row.get('value')} | {row.get('n', '')} | {row.get('notes', '')} |\n")


def make_plots(tables_dir: Path, figures_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib unavailable, skipping paper plots: {exc}")
        return
    rows = read_csv(tables_dir / "height_target_trials_all.csv") if (tables_dir / "height_target_trials_all.csv").exists() else []
    if not rows:
        return
    figures_dir.mkdir(parents=True, exist_ok=True)
    labels = [str(row.get("trial_id") or i) for i, row in enumerate(rows, 1)]
    heights = [as_float(row.get("target_object_height_m"), 0.0) for row in rows]
    plt.figure(figsize=(8, 4))
    plt.bar(labels, heights, color="#4c78a8")
    plt.axhline(3.0, color="#e45756", linestyle="--", label="3.0 m threshold")
    plt.xlabel("Trial")
    plt.ylabel("Object height (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(figures_dir / "paper_target_height_by_trial.png", dpi=180)
    plt.close()


def strict_reasons(height_rows: list[dict[str, object]], required: list[str], expected: int) -> list[str]:
    reasons: list[str] = []
    if not height_rows:
        return ["MISSING_RAW_EXPERIMENT_DATA"]
    if expected <= 0:
        return reasons
    by_scenario: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in height_rows:
        by_scenario[scenario_key(row)].append(row)
    for scenario in required:
        count = len(by_scenario.get(scenario, []))
        if count < expected:
            reasons.append(f"scenario {scenario} has {count} trials, expected >= {expected}")
    return reasons


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare Waver paper evidence tables from raw experiment trials.")
    parser.add_argument("--input-dir", default="~/ros2_ws5/FSD_Vehicle/experiments_result")
    parser.add_argument("--output-root", default="~/ros2_ws5/FSD_Vehicle/experiments_result/paper_ready")
    parser.add_argument("--name", default="")
    parser.add_argument(
        "--expected-trials-per-scenario",
        type=int,
        default=0,
        help="Optional minimum trial count per scenario. Default 0 disables repeated-trial count gating.",
    )
    parser.add_argument("--require-scenarios", default=",".join(REQUIRED_SCENARIOS_DEFAULT))
    parser.add_argument("--paper-strict", action="store_true")
    parser.add_argument("--allow-missing-ground-truth", action="store_true")
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
    required = [s.strip() for s in args.require_scenarios.split(",") if s.strip()]

    height_all = find_height_rows(input_dir) if input_dir.exists() else []
    ui_all = find_ui_rows(input_dir) if input_dir.exists() else []
    metrics = metric_rows(height_all, ui_all, required)
    scenario_rows = scenario_summary_rows(height_all, required)
    failures = failure_reason_rows(height_all)
    confusion = confusion_rows(height_all)
    evidence = evidence_summary_rows(height_all + ui_all)

    write_csv(tables_dir / "height_target_trials_all.csv", HEIGHT_FIELDS, height_all)
    write_csv(tables_dir / "height_target_trials_by_scenario.csv", ["scenario", "n_total", "n_success", "success_rate", "failure_reasons"], scenario_rows)
    write_csv(tables_dir / "ui_visualization_all.csv", UI_FIELDS, ui_all)
    write_csv(tables_dir / "paper_metrics.csv", ["metric", "value", "n", "mean", "std", "min", "max", "ci95", "notes"], metrics)
    write_csv(tables_dir / "failure_reason_counts.csv", ["failure_reason", "count"], failures)
    write_csv(tables_dir / "confusion_matrix_expected_vs_predicted.csv", ["cell", "count"], confusion)
    write_csv(tables_dir / "evidence_level_summary.csv", ["evidence_level", "sim_or_real", "fake_detector_used", "fake_sound_used", "serial_enabled", "ground_truth_source", "count"], evidence)
    write_csv(out / "source_manifest.csv", ["artifact_type", "source", "paper_copy"], copy_selected_sources(height_all + ui_all, raw_dir))

    reasons = strict_reasons(height_all, required, args.expected_trials_per_scenario)
    status = "PASS" if not reasons else "FAIL"
    write_report(reports_dir / "paper_results_summary.md", status, reasons, metrics)
    make_plots(tables_dir, figures_dir)

    latest = output_root / "latest"
    if not args.no_latest and status == "PASS":
        try:
            if latest.is_symlink() or latest.exists():
                latest.unlink()
            latest.symlink_to(out, target_is_directory=True)
        except Exception:
            pass

    print(f"PAPER_RESULTS_OUTPUT={out}")
    print(f"HEIGHT_TRIAL_ROWS={len(height_all)}")
    print(f"UI_TRIAL_ROWS={len(ui_all)}")
    print(f"PAPER_RESULTS_STATUS={status}")
    for reason in reasons:
        print(f"PAPER_RESULTS_REASON={reason}")
    if args.paper_strict and reasons:
        return 2
    return 0 if height_all else 1


if __name__ == "__main__":
    raise SystemExit(main())

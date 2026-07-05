#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

FIELD_BRIDGE_SCRIPTS = {
    "scripts/waver_field_docker_backend_start.sh",
    "scripts/waver_field_local_ui_start.sh",
    "scripts/waver_field_lidar_nav_backend_start.sh",
    "scripts/waver_start_field_backend.sh",
}

FIELD_RELEASE_REQUIRED = {
    "README_BIRD_PATROL_FIELD.md",
    "README_REAL_VEHICLE.md",
    "config/real_profiles/bird_patrol_production.yaml",
    "config/sensors/camera_lidar_extrinsic.yaml",
    "config/sensors/livox_topic_aliases.yaml",
    "config/sensors/livox_mid360_field.local.example",
    "config/perception/bird_model_registry.yaml",
    "scripts/waver_bird_patrol_field_start.sh",
    "scripts/waver_bird_mission_readiness_check.py",
    "scripts/waver_livox_mid360_probe.py",
    "scripts/waver_camera_probe.py",
    "scripts/waver_bird_detector_probe.py",
    "scripts/waver_camera_lidar_calibration_check.py",
    "src/waver_patrol/launch/bird_patrol_production.launch.py",
}

LOCAL_RUNTIME_REQUIRED = FIELD_BRIDGE_SCRIPTS | {
    "scripts/waver_field_env_load.sh",
    "scripts/waver_setup_local_pc.sh",
    "scripts/waver_check_local_operator_deps.py",
    "scripts/waver_field_operator_station_start.sh",
    "scripts/waver_field_rviz_start.sh",
    "scripts/waver_create_home_field_env.sh",
    "config/waver_field_env.local.example",
    "docs/LOCAL_OPERATOR_STATION.md",
}

GENERATED_DIR_NAMES = {"build", "install", "log", ".pytest_cache", "__pycache__", ".colcon"}
EXCLUDE_DIR_NAMES = {
    "bags",
    "rosbag",
    "rosbags",
    "experiment_results",
    "experiments_result",
    "experiments_result_compact",
    "waver_experiments",
}
GENERATED_SUFFIXES = {".pyc", ".pyo", ".pyd", ".db3", ".mcap", ".bag", ".sqlite3", ".log"}
PRIVATE_CONFIG_NAMES = {".env.local", ".env.private", "waver_field_env.local"}
VENDOR_PREFIXES = ("src/livox_ros_driver2",)
SIM_PREFIXES = ("src/ugv_main/ugv_gazebo/", "src/livox_laser_simulation_RO2/", "src/ros2_livox_simulation/")
REPORT_PREFIXES = (
    "reports/bird_mission_readiness/",
    "reports/camera/",
    "reports/bird_detector/",
    "reports/command_chain/",
    "reports/field_bridge_regression/",
    "reports/field_readiness/",
    "reports/launch_contract/",
    "reports/livox_mid360/",
    "reports/network/",
    "reports/release_self_test/",
    "reports/ui_slam_bird_detection/",
)

AUTO_APPLY_DIRS = {"build", "install", "log", ".pytest_cache"}


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def classify(rel_path: str, is_dir: bool) -> str:
    name = Path(rel_path).name
    parts = Path(rel_path).parts
    if rel_path in FIELD_BRIDGE_SCRIPTS or rel_path in LOCAL_RUNTIME_REQUIRED:
        return "LOCAL_RUNTIME_REQUIRED"
    if rel_path in FIELD_RELEASE_REQUIRED or rel_path.startswith(("config/sensors/", "config/perception/")):
        return "FIELD_RELEASE_REQUIRED"
    if any(rel_path == prefix or rel_path.startswith(prefix + "/") for prefix in VENDOR_PREFIXES):
        return "VENDOR_OR_SUBMODULE"
    if name in PRIVATE_CONFIG_NAMES or rel_path.startswith("config/waver_field_env.local"):
        return "PRIVATE_LOCAL_CONFIG"
    if is_dir and (name in GENERATED_DIR_NAMES or name in EXCLUDE_DIR_NAMES):
        return "GENERATED_ARTIFACT" if name in GENERATED_DIR_NAMES else "EXCLUDE_FROM_RELEASE"
    if any(part in GENERATED_DIR_NAMES for part in parts):
        return "GENERATED_ARTIFACT"
    if any(part in EXCLUDE_DIR_NAMES for part in parts):
        return "EXCLUDE_FROM_RELEASE"
    if any(rel_path.startswith(prefix) for prefix in REPORT_PREFIXES):
        return "GENERATED_REPORT"
    if any(rel_path.startswith(prefix) for prefix in SIM_PREFIXES):
        return "SIM_EVIDENCE"
    if Path(rel_path).suffix in GENERATED_SUFFIXES:
        return "GENERATED_ARTIFACT"
    if Path(rel_path).name in {"build_first.sh", "build_common.sh", "build_apriltag.sh"}:
        return "LEGACY_BUT_REFERENCED"
    if rel_path.startswith(("src/", "scripts/", "config/", "docs/")) or rel_path.startswith("README"):
        return "SOURCE_REQUIRED"
    return "CANDIDATE_FOR_ARCHIVE"


def iter_paths() -> list[tuple[str, bool, str]]:
    rows: list[tuple[str, bool, str]] = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        current = Path(dirpath)
        if ".git" in current.parts:
            continue
        kept_dirnames: list[str] = []
        for dirname in sorted(dirnames):
            if dirname == ".git":
                continue
            path = current / dirname
            rel_path = rel(path)
            category = classify(rel_path, True)
            rows.append((rel_path, True, category))
            if category in {"GENERATED_ARTIFACT", "EXCLUDE_FROM_RELEASE"}:
                continue
            kept_dirnames.append(dirname)
        dirnames[:] = kept_dirnames
        for filename in sorted(filenames):
            path = current / filename
            rel_path = rel(path)
            rows.append((rel_path, False, classify(rel_path, False)))
    return sorted(rows, key=lambda item: item[0])


def apply_cleanup(rows: list[tuple[str, bool, str]]) -> list[str]:
    removed: list[str] = []
    for rel_path, is_dir, category in rows:
        if category != "GENERATED_ARTIFACT":
            continue
        first = rel_path.split("/", 1)[0]
        if first not in AUTO_APPLY_DIRS and not rel_path.endswith("__pycache__"):
            continue
        path = ROOT / rel_path
        if not path.exists():
            continue
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
        removed.append(rel_path)
    return removed


def write_report(rows: list[tuple[str, bool, str]], removed: list[str], output: Path) -> None:
    by_category: dict[str, list[str]] = {}
    for rel_path, is_dir, category in rows:
        suffix = "/" if is_dir else ""
        by_category.setdefault(category, []).append(rel_path + suffix)
    lines = [
        "# Waver Conservative Cleanup Plan",
        "",
        f"- workspace: `{ROOT}`",
        f"- apply_removed_count: {len(removed)}",
        "",
        "## Preserve Rules",
        "",
        "- Field bridge scripts are `LOCAL_RUNTIME_REQUIRED` and are never cleanup candidates.",
        "- Source-like legacy files are reported for human review only.",
        "- `--apply` removes generated artifacts only: build/install/log/cache outputs.",
        "",
        "## Field Bridge Preserve List",
        "",
    ]
    for item in sorted(FIELD_BRIDGE_SCRIPTS):
        lines.append(f"- `{item}`")
    if removed:
        lines.extend(["", "## Removed By Apply", ""])
        lines.extend(f"- `{item}`" for item in removed)
    lines.extend(["", "## Categories", ""])
    for category in sorted(by_category):
        lines.extend([f"### {category}", ""])
        for item in by_category[category][:200]:
            lines.append(f"- `{item}`")
        if len(by_category[category]) > 200:
            lines.append(f"- ... {len(by_category[category]) - 200} more")
        lines.append("")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a conservative Waver cleanup plan.")
    parser.add_argument("--output", default=str(ROOT / "reports/cleanup_plan.md"))
    parser.add_argument("--apply", action="store_true", help="remove generated artifacts only")
    args = parser.parse_args()
    rows = iter_paths()
    removed = apply_cleanup(rows) if args.apply else []
    output = Path(args.output).expanduser().resolve()
    write_report(rows, removed, output)
    print(f"WAVER_CLEANUP_PLAN={output}")
    print(f"WAVER_CLEANUP_APPLY_REMOVED={len(removed)}")
    print("WAVER_CLEANUP_MODE=" + ("APPLY_GENERATED_ARTIFACTS_ONLY" if args.apply else "DRY_RUN"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

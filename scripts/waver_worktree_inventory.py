#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

FIELD_RELEASE_REQUIRED = {
    "README_BIRD_PATROL_FIELD.md",
    "config/real_profiles/bird_patrol_production.yaml",
    "config/sensors/camera_lidar_extrinsic.yaml",
    "config/sensors/livox_mid360_field.example.yaml",
    "config/perception/bird_model_registry.yaml",
    "scripts/waver_bird_patrol_field_start.sh",
    "scripts/waver_bird_mission_readiness_check.py",
    "scripts/waver_livox_mid360_probe.py",
    "scripts/waver_camera_probe.py",
    "scripts/waver_bird_detector_probe.py",
    "scripts/waver_camera_lidar_calibration_check.py",
    "scripts/check_bird_mission_field_release.py",
    "scripts/make_bird_mission_field_release.py",
    "scripts/run_bird_mission_release_self_tests.sh",
    "scripts/waver_launch_contract_check.py",
    "scripts/waver_field_bridge_regression_check.py",
    "scripts/waver_command_chain_check.py",
    "src/waver_patrol/launch/bird_patrol_production.launch.py",
}
FIELD_RELEASE_REQUIRED_PREFIXES = (
    "config/sensors/",
    "config/perception/",
)
FIELD_RELEASE_REQUIRED_SCRIPT_PREFIXES = (
    "scripts/waver_bird_",
)
FIELD_BRIDGE_CRITICAL = {
    "scripts/waver_field_docker_backend_start.sh",
    "scripts/waver_field_local_ui_start.sh",
    "scripts/waver_field_lidar_nav_backend_start.sh",
    "scripts/waver_start_field_backend.sh",
}
LOCAL_RUNTIME_REQUIRED = FIELD_BRIDGE_CRITICAL | {
    "scripts/waver_field_env_load.sh",
    "scripts/waver_setup_local_pc.sh",
    "scripts/waver_check_local_operator_deps.py",
    "scripts/waver_field_operator_station_start.sh",
    "scripts/waver_field_rviz_start.sh",
    "scripts/waver_create_home_field_env.sh",
    "config/waver_field_env.local.example",
    "docs/LOCAL_OPERATOR_STATION.md",
}
GENERATED_PREFIXES = ("build/", "install/", "log/", "reports/bird_mission_readiness/", "reports/livox_mid360/", "reports/camera/")
GENERATED_FILES = {"reports/source_manifest.json", "reports/source_sha256_manifest.csv"}
SIM_ONLY_PREFIXES = ("src/ugv_main/ugv_gazebo/", "src/livox_laser_simulation_RO2/")
LEGACY_NAMES = {"build_first.sh", "build_common.sh", "build_apriltag.sh"}
EXCLUDE_PREFIXES = ("experiment_results/", "bags/", ".pytest_cache/", "__pycache__/")


def run(cmd: list[str], cwd: Path = ROOT) -> str:
    proc = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=False)
    return (proc.stdout + proc.stderr).strip()


def git_status() -> list[tuple[str, str]]:
    out = run(["git", "status", "--short"])
    rows: list[tuple[str, str]] = []
    for line in out.splitlines():
        if not line:
            continue
        status = line[:2].strip()
        path = line[3:].strip()
        if " -> " in path:
            path = path.split(" -> ", 1)[1].strip()
        rows.append((status, path))
    return rows


def classify(path: str) -> str:
    if path in FIELD_BRIDGE_CRITICAL:
        return "FIELD_BRIDGE_CRITICAL"
    if path in LOCAL_RUNTIME_REQUIRED:
        return "LOCAL_RUNTIME_REQUIRED"
    if (
        path in FIELD_RELEASE_REQUIRED
        or any(path.startswith(prefix) for prefix in FIELD_RELEASE_REQUIRED_PREFIXES)
        or any(path.startswith(prefix) for prefix in FIELD_RELEASE_REQUIRED_SCRIPT_PREFIXES)
    ):
        return "FIELD_RELEASE_REQUIRED"
    if path == "src/livox_ros_driver2" or path.startswith("src/livox_ros_driver2/"):
        return "VENDOR_OR_SUBMODULE"
    if path in GENERATED_FILES or any(path.startswith(prefix) for prefix in GENERATED_PREFIXES):
        return "GENERATED_REPORT"
    if path.startswith("config/waver_field_env.local") or path.endswith(".local.yaml") or path.startswith(".env"):
        return "PRIVATE_LOCAL_CONFIG"
    if any(path.startswith(prefix) for prefix in EXCLUDE_PREFIXES):
        return "EXCLUDE_FROM_RELEASE"
    if any(path.startswith(prefix) for prefix in SIM_ONLY_PREFIXES):
        return "SIM_ONLY"
    if Path(path).name in LEGACY_NAMES:
        return "LEGACY"
    if path.startswith(("src/", "scripts/", "config/", "docs/", "README")):
        return "SOURCE_REQUIRED"
    return "SOURCE_OPTIONAL"


def main() -> int:
    parser = argparse.ArgumentParser(description="Create Waver worktree inventory and preservation policy report.")
    parser.add_argument("--output", default=str(ROOT / "reports/worktree_inventory.md"))
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    rows = git_status()
    by_category: dict[str, list[tuple[str, str]]] = {}
    for status, path in rows:
        by_category.setdefault(classify(path), []).append((status, path))

    lines = [
        "# Waver Worktree Inventory",
        "",
        f"- workspace: `{ROOT}`",
        f"- branch: `{run(['git', 'branch', '--show-current'])}`",
        f"- head: `{run(['git', 'rev-parse', '--short', 'HEAD'])}`",
        f"- total changed/untracked rows: {len(rows)}",
        "",
        "## Policy",
        "",
        "- `build/`, `install/`, and `log/` are generated artifacts and must never be edited as source.",
        "- `src/livox_ros_driver2` is treated as `VENDOR_OR_SUBMODULE` until the Livox overlay policy is finalized.",
        "- `scripts/waver_field_docker_backend_start.sh`, `scripts/waver_field_local_ui_start.sh`, `scripts/waver_field_lidar_nav_backend_start.sh`, and `scripts/waver_start_field_backend.sh` are field bridge critical and require regression checks.",
        "- Local operator PC scripts are `LOCAL_RUNTIME_REQUIRED`; they must not start real robot backend nodes locally.",
        "- Probe JSON files and source manifests under `reports/` are generated evidence, not hand-edited source.",
        "",
        "## Critical Field Bridge Scripts",
        "",
    ]
    for rel in sorted(FIELD_BRIDGE_CRITICAL):
        state = "present" if (ROOT / rel).exists() else "MISSING"
        lines.append(f"- `{rel}`: {state}")
    lines.extend(
        [
            "",
        "## Categories",
        "",
        ]
    )
    if not rows:
        lines.append("No dirty worktree entries were reported by git.")
    for category in sorted(by_category):
        lines.extend([f"### {category}", ""])
        for status, path in sorted(by_category[category], key=lambda item: item[1]):
            lines.append(f"- `{status}` `{path}`")
        lines.append("")

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    print(f"WAVER_WORKTREE_INVENTORY={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

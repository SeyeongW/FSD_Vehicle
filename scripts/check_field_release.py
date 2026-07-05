#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import tempfile
import tarfile
import zipfile
from pathlib import Path
import subprocess


REQUIRED = {
    "README_REAL_VEHICLE.md",
    "REAL_VEHICLE_TEST_PLAN.md",
    "docs/hardware_readiness_levels.md",
    "docs/hardware_acceptance_matrix.md",
    "docs/ros2_control_migration_plan.md",
    "scripts/waver_field_readiness_check.py",
    "scripts/waver_base_feedback_probe.py",
    "scripts/waver_motor_calibration_wizard.py",
    "scripts/waver_blackbox_recorder.sh",
    "scripts/waver_field_stop_all.sh",
    "scripts/waver_field_lidar_nav_backend_start.sh",
    "reports/source_manifest.json",
    "reports/source_sha256_manifest.csv",
    "config/hardware_acceptance_matrix.yaml",
    "config/real_profiles/wheel_on_low_speed.yaml",
}


def names(path: Path) -> set[str]:
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            return {n.split("/", 1)[1] if "/" in n else n for n in zf.namelist()}
    with tarfile.open(path, "r:*") as tar:
        return {m.name.split("/", 1)[1] if "/" in m.name else m.name for m in tar.getmembers()}


def extract_archive(path: Path, dest: Path) -> Path:
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            zf.extractall(dest)
    else:
        with tarfile.open(path, "r:*") as tar:
            tar.extractall(dest)
    children = [p for p in dest.iterdir() if p.is_dir()]
    if len(children) == 1:
        return children[0]
    return dest


def run_in_tree(tree: Path, cmd: list[str]) -> tuple[int, str]:
    proc = subprocess.run(cmd, cwd=tree, text=True, capture_output=True, check=False)
    return proc.returncode, proc.stdout + proc.stderr


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Waver field release archive.")
    parser.add_argument("--path", required=True)
    args = parser.parse_args()
    archive = Path(args.path).expanduser().resolve()
    privacy = subprocess.run(["python3", str(Path(__file__).resolve().parent / "check_submission_package.py"), "--path", str(archive)], text=True, capture_output=True)
    findings = []
    if privacy.returncode != 0:
        findings.append(privacy.stdout + privacy.stderr)
    present = names(archive)
    missing = sorted(req for req in REQUIRED if req not in present)
    findings.extend(f"missing required field release file: {req}" for req in missing)
    forbidden = {
        "config/waver_field_env",
        "config/waver_field_env.local",
        ".env.local",
    }
    leaked = sorted(item for item in forbidden if item in present)
    findings.extend(f"private field config must not be in field release: {item}" for item in leaked)
    heavy_patterns = (
        "build/",
        "install/",
        "log/",
        "build_docker/",
        "install_docker/",
        "log_docker/",
        ".git/",
        "experiment_results/",
        "src/ugv_main/ugv_gazebo/",
        "src/livox_laser_simulation_RO2/",
        "src/ros2_livox_simulation/",
        "src/livox_ros_driver2/",
        "src/Livox-SDK2/",
        "src/ugv_else/",
        ".mcap",
        ".db3",
    )
    heavy = sorted(item for item in present if any(pattern in item for pattern in heavy_patterns))
    findings.extend(f"default field release includes generated/heavy asset: {item}" for item in heavy[:20])

    if not findings:
        with tempfile.TemporaryDirectory(prefix="waver_field_release_") as td:
            tree = extract_archive(archive, Path(td))
            if (tree / "config/waver_field_env").exists():
                findings.append("extracted release includes config/waver_field_env")
            quickstart = [
                "bash",
                "scripts/waver_quickstart_field.sh",
                "--dry-run",
                "--force",
                "--jetson-host",
                "192.0.2.10",
                "--jetson-user",
                "waver",
                "--jetson-ws",
                "/home/waver/ros2_ws5/FSD_Vehicle",
                "--container",
                "fsd_dev_jetson",
                "--serial-port",
                "/dev/serial/by-id/usb-WAVER_BASE_TEST",
            ]
            rc, out = run_in_tree(tree, quickstart)
            if rc != 0:
                findings.append("quickstart dry-run failed in extracted release:\n" + out[-4000:])
            local_env = tree / "config/waver_field_env.local"
            if local_env.exists():
                local_env.unlink()
            rc, out = run_in_tree(tree, ["bash", "scripts/waver_clone_to_run_acceptance.sh", "--release-mode"])
            if rc != 0:
                findings.append("clone-to-run release acceptance failed:\n" + out[-4000:])
    if findings:
        print("FIELD_RELEASE_CHECK=FAIL")
        for item in findings:
            print(f"- {item}")
        return 1
    print("FIELD_RELEASE_CHECK=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

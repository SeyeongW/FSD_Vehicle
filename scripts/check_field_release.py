#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
    "config/real_profiles/wheel_on_low_speed.yaml",
}


def names(path: Path) -> set[str]:
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            return {n.split("/", 1)[1] if "/" in n else n for n in zf.namelist()}
    with tarfile.open(path, "r:*") as tar:
        return {m.name.split("/", 1)[1] if "/" in m.name else m.name for m in tar.getmembers()}


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
    if findings:
        print("FIELD_RELEASE_CHECK=FAIL")
        for item in findings:
            print(f"- {item}")
        return 1
    print("FIELD_RELEASE_CHECK=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

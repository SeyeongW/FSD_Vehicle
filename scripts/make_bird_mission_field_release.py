#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import io
import json
import tarfile
import time
from pathlib import Path

import make_source_archive


ROOT = Path(__file__).resolve().parents[1]
SIM_ONLY_PREFIXES = (
    "src/ugv_main/ugv_gazebo/",
    "src/livox_laser_simulation_RO2/",
    "src/ros2_livox_simulation/",
)
REQUIRED = (
    "src/waver_patrol",
    "scripts/waver_bird_patrol_field_start.sh",
    "scripts/waver_bird_mission_readiness_check.py",
    "scripts/waver_check_local_operator_deps.py",
    "scripts/waver_bird_mission_supervisor.py",
    "scripts/waver_livox_mid360_probe.py",
    "scripts/waver_camera_probe.py",
    "scripts/waver_bird_detector_probe.py",
    "scripts/waver_camera_lidar_calibration_check.py",
    "scripts/run_bird_mission_release_self_tests.sh",
    "scripts/run_ui_slam_bird_detection_gazebo_smoke.sh",
    "scripts/check_ui_slam_bird_detection_result.py",
    "scripts/waver_launch_contract_check.py",
    "scripts/waver_field_bridge_regression_check.py",
    "scripts/waver_field_operator_station_start.sh",
    "scripts/waver_field_rviz_start.sh",
    "scripts/waver_setup_local_pc.sh",
    "scripts/waver_command_chain_check.py",
    "config/real_profiles/bird_patrol_production.yaml",
    "config/real_profiles/sensor_live.yaml",
    "config/real_profiles/inspection_dry_run.yaml",
    "config/real_profiles/supervised_bird_patrol.yaml",
    "config/real_profiles/autonomous_bird_patrol_locked.yaml",
    "config/sensors/camera_lidar_extrinsic.yaml",
    "README_BIRD_PATROL_FIELD.md",
    "README_REAL_VEHICLE.md",
    "docs/LOCAL_OPERATOR_STATION.md",
    "docs/RVIZ_FIELD_RUNBOOK.md",
    "docs/bird_patrol_field_profiles.md",
    "docs/repository_cleanup_policy.md",
    "reports/remote_ui_slam_bird_feature_audit.md",
    "reports/cleanup_plan.md",
    "reports/bird_mission_readiness_audit.md",
    "reports/ui_slam_bird_detection/README.md",
    "docs/final_bird_patrol_architecture.md",
    "docs/bird_mission_readiness_levels.md",
    "src/waver_patrol/rviz/waver_field_operator.rviz",
    "requirements-jetson-perception.txt",
)


def excluded(rel: str, include_sim: bool) -> bool:
    if rel == "src/ugv_main/ugv_gazebo/param/ui_slam/gazebo.yaml":
        return False
    if not include_sim and any(rel.startswith(prefix) for prefix in SIM_ONLY_PREFIXES):
        return True
    return False


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a clean bird mission field release archive.")
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--output", required=True)
    parser.add_argument("--include-sim", action="store_true")
    parser.add_argument("--write-manifest-to-source", action="store_true")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    missing = [rel for rel in REQUIRED if not (root / rel).exists()]
    if missing:
        raise SystemExit("missing required bird mission release files: " + ", ".join(missing))
    files = [
        path
        for path in make_source_archive.iter_source_files(root)
        if not excluded(path.relative_to(root).as_posix(), args.include_sim)
    ]
    generated = make_source_archive.generated_release_metadata(root, files)
    generated[Path("reports/README.md")] = (
        "# Runtime Reports\n\n"
        "This directory contains source manifests in the release archive. "
        "Live hardware and Gazebo runtime reports are generated on the target robot or test PC and are intentionally not packaged as stale evidence.\n"
    ).encode()
    manifest = json.loads(generated[Path("reports/source_manifest.json")].decode())
    manifest.update(
        {
            "artifact_type": "bird_mission_field_release",
            "included_profile": "bird_patrol_production",
            "detector_model_included": False,
            "calibration_files_included": True,
            "release_size_bytes": 0,
            "release_sha256": "computed_after_archive_write",
            "included_file_count": len(files) + len(generated),
        }
    )
    generated[Path("reports/source_manifest.json")] = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
    if args.write_manifest_to_source:
        make_source_archive.write_generated_metadata(root, generated)
    output.parent.mkdir(parents=True, exist_ok=True)
    base = root.name
    with tarfile.open(output, "w:gz") as tar:
        for path in files:
            tar.add(path, arcname=str(Path(base) / path.relative_to(root)))
        for rel_path, payload in generated.items():
            info = tarfile.TarInfo(str(Path(base) / rel_path))
            info.size = len(payload)
            info.mtime = int(time.time())
            info.mode = 0o644
            tar.addfile(info, io.BytesIO(payload))
    size = output.stat().st_size
    sha = file_sha256(output)
    print(f"BIRD_MISSION_FIELD_RELEASE_WRITTEN={output}")
    print(f"BIRD_MISSION_FIELD_RELEASE_FILE_COUNT={len(files) + len(generated)}")
    print(f"BIRD_MISSION_FIELD_RELEASE_SIZE_BYTES={size}")
    print(f"BIRD_MISSION_FIELD_RELEASE_SHA256={sha}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

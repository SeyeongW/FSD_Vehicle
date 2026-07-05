#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import tarfile
import time
from pathlib import Path

import make_source_archive


ROOT = Path(__file__).resolve().parents[1]

SIM_PREFIXES = (
    "src/ugv_main/ugv_gazebo/",
    "src/livox_laser_simulation_RO2/",
    "src/ros2_livox_simulation/",
    "worlds/",
    "models/",
)
VENDOR_PREFIXES = (
    "src/livox_ros_driver2/",
    "src/Livox-SDK2/",
    "src/ugv_else/",
)
LEGACY_REPORT_PREFIXES = (
    "reports/pre_existing_",
    "reports/gazebo_functional_validation/",
    "reports/remote_ui_validation/",
    "reports/full_readiness_loop/",
)


def field_excluded(rel: str, include_sim: bool, include_vendor: bool, include_legacy_reports: bool) -> bool:
    if not include_sim and any(rel.startswith(prefix) for prefix in SIM_PREFIXES):
        return True
    if not include_vendor and any(rel.startswith(prefix) for prefix in VENDOR_PREFIXES):
        return True
    if not include_legacy_reports and any(rel.startswith(prefix) for prefix in LEGACY_REPORT_PREFIXES):
        return True
    return False


def make_field_release(
    root: Path,
    output: Path,
    *,
    include_sim: bool = False,
    include_vendor: bool = False,
    include_legacy_reports: bool = False,
) -> int:
    source_files = make_source_archive.iter_source_files(root)
    files = [
        path
        for path in source_files
        if not field_excluded(path.relative_to(root).as_posix(), include_sim, include_vendor, include_legacy_reports)
    ]
    generated = make_source_archive.generated_release_metadata(root, files)
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
    return len(files) + len(generated)


def main() -> int:
    parser = argparse.ArgumentParser(description="Create Waver hardware-focused field deployment source release.")
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--output", required=True)
    parser.add_argument("--include-sim", action="store_true", help="include Gazebo/world/model assets")
    parser.add_argument("--include-legacy-reports", action="store_true", help="include historical validation reports")
    parser.add_argument("--include-vendor", action="store_true", help="include vendor/Livox source trees")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    count = make_field_release(
        root,
        output,
        include_sim=args.include_sim,
        include_vendor=args.include_vendor,
        include_legacy_reports=args.include_legacy_reports,
    )
    print(f"FIELD_RELEASE_WRITTEN={output}")
    print(f"FIELD_RELEASE_FILE_COUNT={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

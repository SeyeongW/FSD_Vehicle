#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import json
import subprocess
import tarfile
import tempfile
import time
import zipfile
from pathlib import Path


REQUIRED = {
    "README_BIRD_PATROL_FIELD.md",
    "README_REAL_VEHICLE.md",
    "docs/LOCAL_OPERATOR_STATION.md",
    "docs/RVIZ_FIELD_RUNBOOK.md",
    "docs/bird_patrol_field_profiles.md",
    "docs/repository_cleanup_policy.md",
    "docs/final_bird_patrol_architecture.md",
    "docs/bird_mission_readiness_levels.md",
    "scripts/waver_bird_patrol_field_start.sh",
    "scripts/waver_bird_mission_readiness_check.py",
    "scripts/waver_bird_mission_supervisor.py",
    "scripts/waver_check_local_operator_deps.py",
    "scripts/waver_conservative_cleanup_plan.py",
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
    "config/sensors/livox_topic_aliases.yaml",
    "config/sensors/livox_mid360_field.local.example",
    "config/hardware_acceptance_matrix.yaml",
    "config/real_profiles/bird_patrol_production.yaml",
    "config/real_profiles/sensor_live.yaml",
    "config/real_profiles/inspection_dry_run.yaml",
    "config/real_profiles/supervised_bird_patrol.yaml",
    "config/real_profiles/autonomous_bird_patrol_locked.yaml",
    "config/sensors/camera_lidar_extrinsic.yaml",
    "reports/remote_ui_slam_bird_feature_audit.md",
    "reports/cleanup_plan.md",
    "reports/bird_mission_readiness_audit.md",
    "reports/ui_slam_bird_detection/README.md",
    "src/waver_patrol/launch/bird_patrol_production.launch.py",
    "src/waver_patrol/launch/waver_gazebo_mapping_bird_detection.launch.py",
    "src/waver_patrol/rviz/waver_field_operator.rviz",
    "src/waver_patrol/waver_patrol/perception/bird_detector_node.py",
    "src/waver_patrol/waver_patrol/perception/bird_3d_fusion_node.py",
    "src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py",
    "src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py",
}
FORBIDDEN_FRAGMENTS = (
    "/.git/",
    "/build/",
    "/install/",
    "/log/",
    ".db3",
    ".mcap",
    ".zip",
)
FORBIDDEN_EXACT = {
    ".env",
    ".env.local",
    ".env.private",
    "config/waver_field_env",
    "config/waver_field_env.local",
}
STALE_REPORT_PREFIXES = (
    "reports/bird_mission_readiness/",
    "reports/field_readiness/",
    "reports/livox_mid360/",
    "reports/camera/",
    "reports/bird_detector/",
    "reports/hardware_calibration/",
    "reports/calibration/",
    "reports/sound_events/",
    "reports/field_runs/",
    "reports/ui_slam_bird_detection/ui_slam_",
)
STALE_REPORT_EXACT = {
    "reports/command_chain/latest.json",
    "reports/field_bridge_regression/latest.json",
    "reports/launch_contract/latest.json",
    "reports/network/latest.json",
    "reports/ui_slam_bird_detection/latest.json",
}


def archive_names(path: Path) -> set[str]:
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            raw = zf.namelist()
    else:
        with tarfile.open(path, "r:*") as tar:
            raw = [m.name for m in tar.getmembers()]
    return normalize_archive_names(raw)


def normalize_archive_names(raw: list[str]) -> set[str]:
    cleaned = [item.strip("/") for item in raw if item and not item.endswith("/")]
    prefixes: list[str] = []
    for item in cleaned:
        if item.endswith("README_BIRD_PATROL_FIELD.md"):
            prefix = item[: -len("README_BIRD_PATROL_FIELD.md")].strip("/")
            if prefix:
                prefixes.append(prefix + "/")
            else:
                prefixes.append("")
    for prefix in sorted(prefixes, key=len, reverse=True):
        names = {item[len(prefix) :] for item in cleaned if item.startswith(prefix)}
        if "README_BIRD_PATROL_FIELD.md" in names and "src/waver_patrol/package.xml" in names:
            return names
    names = set()
    for item in cleaned:
        names.add(item.split("/", 1)[1] if "/" in item else item)
    return names


def extract_archive(path: Path, dest: Path) -> None:
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            zf.extractall(dest)
    else:
        with tarfile.open(path, "r:*") as tar:
            tar.extractall(dest)


def find_release_root(dest: Path) -> Path:
    for candidate in [dest, *dest.rglob("*")]:
        if candidate.is_dir() and (candidate / "README_BIRD_PATROL_FIELD.md").exists() and (candidate / "src/waver_patrol").exists():
            return candidate
    raise RuntimeError("extracted archive root not found")


def self_test(path: Path, report_output: Path | None = None, timeout_sec: int = 420) -> list[str]:
    findings: list[str] = []
    wrapper_payload: dict[str, object] = {}
    timed_out = False
    timed_out_step = ""
    with tempfile.TemporaryDirectory(prefix="waver_bird_release_", ignore_cleanup_errors=True) as td:
        dest = Path(td)
        extract_archive(path, dest)
        root = find_release_root(dest)
        commands = [(["bash", "scripts/run_bird_mission_release_self_tests.sh"], timeout_sec, "release_safe_self_tests")]
        env = dict(**{k: v for k, v in os.environ.items()})
        env["PYTHONPATH"] = f"{root / 'src/waver_patrol'}:{root / 'src/ugv_main/ugv_tools'}:{env.get('PYTHONPATH', '')}"
        extracted_wrapper_report = root / "reports/release_self_test/check_bird_mission_field_release_self_test.json"
        extracted_wrapper_report.parent.mkdir(parents=True, exist_ok=True)
        started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        step_reports: list[dict[str, object]] = []
        for cmd, timeout, label in commands:
            try:
                proc = subprocess.run(
                    cmd,
                    cwd=root,
                    text=True,
                    capture_output=True,
                    timeout=timeout,
                    check=False,
                    env=env,
                )
                combined = proc.stdout + proc.stderr
                step_reports.append(
                    {
                        "label": label,
                        "command": cmd,
                        "timeout_sec": timeout,
                        "returncode": proc.returncode,
                        "status": "PASS" if proc.returncode == 0 else "FAIL",
                        "output_tail": combined[-4000:],
                    }
                )
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                timed_out_step = label
                stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
                stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
                combined = stdout + stderr
                step_reports.append(
                    {
                        "label": label,
                        "command": cmd,
                        "timeout_sec": timeout,
                        "returncode": 124,
                        "status": "TIMEOUT",
                        "output_tail": combined[-4000:],
                    }
                )
                findings.append(f"self-test {label} timed out after {timeout}s:\n{combined[-4000:]}")
                break
            if step_reports[-1]["returncode"] != 0:
                findings.append(f"self-test {label} failed:\n{combined[-4000:]}")
                break
        wrapper_payload = {
            "status": "FAIL" if findings else "PASS",
            "started_at": started_at,
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "archive": str(path),
            "release_root": str(root),
            "release_root_is_temporary": True,
            "timed_out": timed_out,
            "timed_out_step": timed_out_step,
            "steps": step_reports,
            "findings": findings,
        }
        extracted_wrapper_report.write_text(json.dumps(wrapper_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if report_output is not None:
        report_output.parent.mkdir(parents=True, exist_ok=True)
        report_output.write_text(json.dumps(wrapper_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate bird mission field release archive.")
    parser.add_argument("--path", required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--self-test-timeout-sec",
        type=int,
        default=420,
        help="timeout for the release-safe self-test wrapper",
    )
    parser.add_argument(
        "--self-test-report",
        default="reports/release_self_test/check_bird_mission_field_release_self_test.json",
        help="where to store the wrapper self-test stdout/stderr tail report",
    )
    args = parser.parse_args()
    path = Path(args.path).expanduser().resolve()
    if not path.exists():
        print("BIRD_MISSION_FIELD_RELEASE_CHECK=FAIL")
        print(f"- archive missing: {path}")
        return 1
    names = archive_names(path)
    findings = []
    if "reports/source_manifest.json" not in names:
        findings.append("missing generated source manifest")
    if "reports/source_sha256_manifest.csv" not in names:
        findings.append("missing generated sha256 source manifest")
    if "reports/README.md" not in names:
        findings.append("missing reports/README.md runtime evidence policy")
    missing = sorted(item for item in REQUIRED if item not in names)
    findings.extend(f"missing required bird mission file: {item}" for item in missing)
    leaked = sorted(
        item for item in names if item in FORBIDDEN_EXACT or any(fragment in item for fragment in FORBIDDEN_FRAGMENTS)
    )
    findings.extend(f"forbidden release path: {item}" for item in leaked[:30])
    stale = sorted(
        item
        for item in names
        if item in STALE_REPORT_EXACT
        or any(item.startswith(prefix) and item.endswith((".json", ".csv", ".db3", ".mcap")) for prefix in STALE_REPORT_PREFIXES)
        or item.startswith("reports/release_self_test/latest_")
    )
    findings.extend(f"stale runtime report leaked into release: {item}" for item in stale[:30])
    if path.name.endswith(".zip"):
        findings.append("raw workspace zip is not accepted as bird mission field release; use make_bird_mission_field_release.py")
    sidecar = Path(str(path) + ".manifest.json")
    if sidecar.exists():
        try:
            payload = json.loads(sidecar.read_text(encoding="utf-8"))
            actual_sha = subprocess.check_output(["sha256sum", str(path)], text=True).split()[0]
            if payload.get("archive_sha256") != actual_sha:
                findings.append("sidecar archive_sha256 does not match archive")
            if payload.get("archive_size_bytes") != path.stat().st_size:
                findings.append("sidecar archive_size_bytes does not match archive")
        except Exception as exc:
            findings.append(f"sidecar manifest parse/check failed: {exc}")
    if args.self_test and not findings:
        findings.extend(
            self_test(
                path,
                Path(args.self_test_report).expanduser().resolve(),
                timeout_sec=max(1, args.self_test_timeout_sec),
            )
        )
    if findings:
        print("BIRD_MISSION_FIELD_RELEASE_CHECK=FAIL")
        for item in findings:
            print(f"- {item}")
        return 1
    print("BIRD_MISSION_FIELD_RELEASE_CHECK=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

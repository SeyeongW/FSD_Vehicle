#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILES = [
    "src/waver_patrol/launch/bird_patrol_production.launch.py",
    "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py",
    "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py",
]
REQUIRED_ARGS = {
    "bird_patrol_production.launch.py": {
        "profile",
        "use_sim_time",
        "serial_port",
        "bird_model_path",
        "camera_lidar_extrinsic",
        "scan_topic",
        "pointcloud_topic",
        "camera_image_topic",
        "camera_info_topic",
        "enable_collision_monitor",
        "enable_sound_output",
        "sound_safety_ack",
        "safety_max_linear_speed",
        "safety_max_angular_speed",
    },
    "waver_real_bird_autonomy.launch.py": {
        "use_sim_time",
        "real_profile",
        "serial_port",
        "bird_model_path",
        "camera_lidar_extrinsic",
        "scan_topic",
        "pointcloud_topic",
        "camera_image_topic",
        "camera_info_topic",
        "enable_collision_monitor",
        "enable_sound_output",
        "sound_safety_ack",
        "safety_max_linear_speed",
        "safety_max_angular_speed",
    },
    "waver_nav2_radar_bird_mission.launch.py": {
        "serial_port",
        "scan_topic",
        "pointcloud_topic",
        "enable_collision_monitor",
        "collision_cmd_vel_in_topic",
        "collision_cmd_vel_out_topic",
        "safety_cmd_vel_out_topic",
        "enable_sound_output",
        "sound_safety_ack",
    },
}


def declared_args(text: str) -> set[str]:
    return set(re.findall(r"DeclareLaunchArgument\(\s*['\"]([^'\"]+)['\"]", text))


def default_for_arg(text: str, name: str) -> str:
    pattern = re.compile(rf"DeclareLaunchArgument\(\s*['\"]{re.escape(name)}['\"].*?default_value\s*=\s*([^,\)]+)", re.S)
    match = pattern.search(text)
    return match.group(1).strip() if match else ""


def check_profile(path: Path) -> list[str]:
    findings: list[str] = []
    data = yaml.safe_load(path.read_text()) if path.exists() else {}
    if not data:
        return [f"profile missing or empty: {path}"]
    if data.get("navigation", {}).get("collision_monitor_required_for_autonomous") is not True:
        findings.append("production profile must require collision monitor for autonomous mode")
    if data.get("navigation", {}).get("enable_collision_monitor") is not True:
        findings.append("production profile navigation.enable_collision_monitor must default true")
    if data.get("enable_sound_output") is not False:
        findings.append("production profile must default enable_sound_output=false")
    if data.get("pointcloud_topic") != "/livox/lidar":
        findings.append("production profile pointcloud_topic must be /livox/lidar")
    return findings


def maybe_show_args(report_dir: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not shutil.which("ros2"):
        return result
    out_file = report_dir / "bird_patrol_production_show_args.txt"
    try:
        proc = subprocess.run(
            ["timeout", "20", "ros2", "launch", "waver_patrol", "bird_patrol_production.launch.py", "--show-args"],
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=25,
            check=False,
        )
        out_file.write_text(proc.stdout + proc.stderr, encoding="utf-8")
        result["bird_patrol_production_show_args"] = str(out_file)
        result["bird_patrol_production_show_args_rc"] = str(proc.returncode)
    except Exception as exc:
        result["bird_patrol_production_show_args_error"] = str(exc)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Static Waver production launch contract checker.")
    parser.add_argument("--profile", default=str(ROOT / "config/real_profiles/bird_patrol_production.yaml"))
    parser.add_argument("--output", default=str(ROOT / "reports/launch_contract/latest.json"))
    parser.add_argument("--show-args", action="store_true")
    args = parser.parse_args()

    findings: list[str] = []
    launch_summary: dict[str, object] = {}
    for rel in LAUNCH_FILES:
        path = ROOT / rel
        text = path.read_text(errors="replace") if path.exists() else ""
        args_found = declared_args(text)
        missing = sorted(REQUIRED_ARGS[path.name] - args_found)
        launch_summary[rel] = {"declared_args": sorted(args_found), "missing_required_args": missing}
        findings.extend(f"{rel}: missing launch arg {name}" for name in missing)
        if path.name == "bird_patrol_production.launch.py":
            use_sim_default = default_for_arg(text, "use_sim_time")
            if "false" not in use_sim_default.lower():
                findings.append("bird_patrol_production.launch.py use_sim_time must default false")
    profile_path = Path(args.profile).expanduser().resolve()
    findings.extend(check_profile(profile_path))

    field_start = (ROOT / "scripts/waver_bird_patrol_field_start.sh").read_text(errors="replace")
    lidar_backend = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text(errors="replace")
    for token in ("--bird-model", "--camera-extrinsic", "BIRD_MODEL_PATH", "CAMERA_LIDAR_EXTRINSIC"):
        if token not in field_start and token not in lidar_backend:
            findings.append(f"field backend does not expose/pass {token}")
    for token in ("bird_model_path:=", "camera_lidar_extrinsic:=", "scan_topic:=", "pointcloud_topic:=", "enable_collision_monitor:="):
        if token not in lidar_backend:
            findings.append(f"strict lidar backend does not pass launch argument {token}")

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "FAIL" if findings else "PASS",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "profile": str(profile_path),
        "launch_files": launch_summary,
        "findings": findings,
    }
    if args.show_args:
        report["show_args"] = maybe_show_args(output.parent)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"WAVER_LAUNCH_CONTRACT={report['status']}")
    print(f"WAVER_LAUNCH_CONTRACT_REPORT={output}")
    for finding in findings:
        print(f"- {finding}")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())

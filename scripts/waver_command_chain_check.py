#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, proc.stdout + proc.stderr
    except Exception as exc:
        return 99, str(exc)


def count_publishers(raw: str) -> int:
    match = re.search(r"Publisher count:\s*(\d+)", raw)
    return int(match.group(1)) if match else -1


def publisher_nodes(raw: str) -> list[str]:
    nodes: list[str] = []
    current = ""
    for line in raw.splitlines():
        node_match = re.search(r"Node name:\s*(\S+)", line)
        if node_match:
            current = node_match.group(1)
        elif "Endpoint type: PUBLISHER" in line and current:
            nodes.append(current)
            current = ""
    return nodes


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Waver final command chain contract.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--profile", default=str(ROOT / "config/real_profiles/bird_patrol_production.yaml"))
    parser.add_argument("--require-collision-monitor", action="store_true")
    parser.add_argument("--output", default=str(ROOT / "reports/command_chain/latest.json"))
    args = parser.parse_args()

    profile_path = Path(args.profile).expanduser().resolve()
    profile = yaml.safe_load(profile_path.read_text()) if profile_path.exists() else {}
    required_collision = bool(
        args.require_collision_monitor
        or profile.get("navigation", {}).get("collision_monitor_required_for_autonomous")
        or profile.get("navigation", {}).get("enable_collision_monitor")
    )
    findings: list[str] = []
    mission_launch = (ROOT / "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py").read_text(errors="replace")
    real_launch = (ROOT / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py").read_text(errors="replace")
    if required_collision:
        if "nav2_collision_monitor" not in mission_launch:
            findings.append("production command chain requires nav2_collision_monitor node in mission launch")
        if "/waver/cmd_vel_safety" not in mission_launch:
            findings.append("safety mux output /waver/cmd_vel_safety is missing from mission launch")
        if "collision_cmd_vel_out_topic" not in mission_launch or "/cmd_vel" not in mission_launch:
            findings.append("collision monitor output /cmd_vel contract missing")
        if "enable_collision_monitor" not in real_launch:
            findings.append("real launch does not expose enable_collision_monitor")
    if "safety_cmd_mux_node" not in mission_launch:
        findings.append("mission launch must include safety_cmd_mux_node")
    if "mission_patrol_manager_node" not in mission_launch:
        findings.append("mission launch must include mission_patrol_manager_node")

    runtime: dict[str, object] = {}
    if not args.dry_run:
        rc, cmd_vel = run(["ros2", "topic", "info", "-v", "/cmd_vel"])
        runtime["cmd_vel_info"] = cmd_vel[-5000:]
        if rc != 0:
            findings.append("/cmd_vel topic info failed")
        else:
            pubs = count_publishers(cmd_vel)
            nodes = publisher_nodes(cmd_vel)
            if pubs != 1:
                findings.append(f"/cmd_vel publisher count must be 1, got {pubs}")
            if required_collision and not any("collision" in node for node in nodes):
                findings.append("/cmd_vel final publisher must be collision monitor when required")
        rc, mode = run(["ros2", "topic", "info", "-v", "/waver/mode"])
        runtime["mode_info"] = mode[-5000:]
        if rc == 0:
            pubs = count_publishers(mode)
            nodes = publisher_nodes(mode)
            if pubs != 1:
                findings.append(f"/waver/mode publisher count must be 1, got {pubs}")
            if not any(node == "mission_patrol_manager_node" for node in nodes):
                findings.append("/waver/mode publisher must be mission_patrol_manager_node")

    report = {
        "status": "FAIL" if findings else "PASS",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "dry_run": args.dry_run,
        "profile": str(profile_path),
        "collision_monitor_required": required_collision,
        "expected_chain": "sources -> velocity_smoother(optional) -> safety_cmd_mux_node -> /waver/cmd_vel_safety -> nav2_collision_monitor -> /cmd_vel -> waver_base_driver_node",
        "findings": findings,
        "runtime": runtime,
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"WAVER_COMMAND_CHAIN={report['status']}")
    print(f"WAVER_COMMAND_CHAIN_REPORT={output}")
    for finding in findings:
        print(f"- {finding}")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())

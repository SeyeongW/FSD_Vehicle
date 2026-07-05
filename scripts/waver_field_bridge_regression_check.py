#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def add(findings: list[str], ok: bool, msg: str) -> None:
    if not ok:
        findings.append(msg)


def remote_smoke(host: str, user: str, container: str) -> dict[str, str | int]:
    if not host or not user:
        return {"status": "SKIP", "reason": "host/user not provided"}
    cmd = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=5",
        f"{user}@{host}",
        f"docker ps --format '{{{{.Names}}}}' | grep -qx {container!r} && echo DOCKER_OK || echo DOCKER_MISSING",
    ]
    proc = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=8, check=False)
    return {"status": "PASS" if proc.returncode == 0 and "DOCKER_OK" in proc.stdout else "FAIL", "rc": proc.returncode, "output": proc.stdout + proc.stderr}


def main() -> int:
    parser = argparse.ArgumentParser(description="Dry-run regression checker for Waver local UI -> SSH -> Jetson Docker bridge.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--remote-smoke", action="store_true")
    parser.add_argument("--host", default=os.environ.get("JETSON_HOST", ""))
    parser.add_argument("--user", default=os.environ.get("JETSON_USER", "sw"))
    parser.add_argument("--container", default=os.environ.get("CONTAINER", "fsd_dev_jetson"))
    parser.add_argument("--jetson-ws", default=os.environ.get("JETSON_WS", "/home/sw/ros2_ws5/FSD_Vehicle"))
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "0"))
    parser.add_argument("--serial-port", default=os.environ.get("SERIAL_PORT", "auto"))
    parser.add_argument("--output", default=str(ROOT / "reports/field_bridge_regression/latest.json"))
    args = parser.parse_args()

    backend = text("scripts/waver_field_docker_backend_start.sh")
    strict_backend = text("scripts/waver_field_lidar_nav_backend_start.sh")
    ui = text("scripts/waver_field_local_ui_start.sh")
    env_loader = text("scripts/waver_field_env_load.sh")
    findings: list[str] = []
    add(findings, "waver_ssh_cmd" in env_loader and "sshpass" in env_loader, "env loader must provide password/key SSH command helper")
    add(findings, "WAVER_ALLOW_PASSWORD_SSH" in env_loader, "password SSH must remain explicitly gated")
    add(findings, "WAVER_ALLOW_LEGACY_OPEN_LOOP_MICRO_PATROL" in backend, "legacy open-loop backend must be explicitly gated")
    add(findings, "docker exec" in backend, "legacy backend must preserve Jetson Docker topology")
    add(findings, "docker exec" in strict_backend, "strict backend must use Jetson Docker topology")
    add(findings, "ros2 launch waver_patrol bird_patrol_production.launch.py" in strict_backend, "strict backend must launch bird_patrol_production")
    add(findings, "ROS_DOMAIN_ID" in backend and "ROS_DOMAIN_ID" in strict_backend and "remote_bridge_ros_domain_id" in ui, "ROS_DOMAIN_ID must be passed through backend and UI")
    add(findings, "remote_bridge_enabled:=true" in ui, "local UI must enable remote bridge")
    add(findings, "publish_direct_cmd_vel:=false" in ui, "local UI must not publish final /cmd_vel directly")
    add(findings, "/waver/manual_cmd_vel" in text("src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py"), "remote panel must publish manual candidate topic")
    add(findings, "remote_bridge_password" in ui and "JETSON_PASS" in ui, "local UI must pass password only when local ignored env provides it")

    report: dict[str, object] = {
        "status": "FAIL" if findings else "PASS",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "dry_run": bool(args.dry_run),
        "field_bridge": "local PC -> SSH -> Jetson host -> docker exec -> fsd_dev_jetson -> ROS2 nodes -> Waver USB serial",
        "jetson_host": args.host,
        "jetson_user": args.user,
        "jetson_ws": args.jetson_ws,
        "container": args.container,
        "ros_domain_id": args.ros_domain_id,
        "docker_exec_command": f"docker exec {args.container} bash -lc 'cd /ros2_ws/ros2_ws5 && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && ros2 launch waver_patrol bird_patrol_production.launch.py'",
        "product_launch_target": "waver_patrol bird_patrol_production.launch.py",
        "serial_port_policy": args.serial_port,
        "expected_command_chain": "/waver/manual_cmd_vel -> safety_cmd_mux_node -> /waver/cmd_vel_safety -> nav2_collision_monitor -> /cmd_vel -> waver_base_driver_node",
        "findings": findings,
    }
    if args.remote_smoke:
        report["remote_smoke"] = remote_smoke(args.host, args.user, args.container)
        if report["remote_smoke"].get("status") == "FAIL":
            findings.append("remote smoke failed")
            report["status"] = "FAIL"

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"WAVER_FIELD_BRIDGE_REGRESSION={report['status']}")
    print(f"WAVER_FIELD_BRIDGE_REGRESSION_REPORT={output}")
    for finding in findings:
        print(f"- {finding}")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())

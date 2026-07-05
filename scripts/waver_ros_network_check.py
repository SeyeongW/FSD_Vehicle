#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Check local/field ROS_DOMAIN_ID and DDS networking assumptions.")
    parser.add_argument("--remote", action="store_true")
    parser.add_argument("--host", default=os.environ.get("JETSON_HOST", ""))
    parser.add_argument("--user", default=os.environ.get("JETSON_USER", "sw"))
    parser.add_argument("--container", default=os.environ.get("CONTAINER", "fsd_dev_jetson"))
    parser.add_argument("--output", default=str(ROOT / "reports/network/latest.json"))
    args = parser.parse_args()
    config_path = ROOT / "config/network/waver_ros_domain.yaml"
    config = yaml.safe_load(config_path.read_text()) if config_path.exists() else {}
    expected_domain = str(config.get("field_ros_domain_id", 0))
    local_domain = os.environ.get("ROS_DOMAIN_ID", str(config.get("default_ros_domain_id", 0)))
    local_rmw = os.environ.get("RMW_IMPLEMENTATION", config.get("rmw_implementation", ""))
    rc, ip_out = run(["bash", "-lc", "ip -4 addr show | sed -n 's/^[[:space:]]*inet //p'"], timeout=3)
    findings: list[str] = []
    if str(local_domain) != expected_domain:
        findings.append(f"local ROS_DOMAIN_ID={local_domain} does not match field_ros_domain_id={expected_domain}")
    remote: dict[str, object] = {}
    if args.remote:
        if not args.host:
            findings.append("--remote requires --host or JETSON_HOST")
        else:
            cmd = [
                "ssh",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=5",
                f"{args.user}@{args.host}",
                (
                    "printf 'host_ros_domain=%s\\n' \"${ROS_DOMAIN_ID:-}\"; "
                    f"docker exec {args.container} bash -lc 'printf docker_ros_domain=%s\\\\n \"${{ROS_DOMAIN_ID:-}}\"' 2>/dev/null || true"
                ),
            ]
            rrc, rout = run(cmd, timeout=8)
            remote = {"rc": rrc, "output": rout[-3000:]}
            if rrc != 0:
                findings.append("remote SSH/network smoke failed")
            if f"docker_ros_domain={expected_domain}" not in rout:
                findings.append("Docker ROS_DOMAIN_ID was not confirmed")

    report = {
        "status": "FAIL" if findings else "PASS",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "config": str(config_path),
        "expected_field_ros_domain_id": expected_domain,
        "local_ros_domain_id": local_domain,
        "local_rmw": local_rmw,
        "local_ipv4": ip_out if rc == 0 else "",
        "remote": remote,
        "findings": findings,
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"WAVER_ROS_NETWORK={report['status']}")
    print(f"WAVER_ROS_NETWORK_REPORT={output}")
    for finding in findings:
        print(f"- {finding}")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())

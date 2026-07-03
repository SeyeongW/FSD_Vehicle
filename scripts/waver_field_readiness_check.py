#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
LEVELS = {"L0", "L1", "L2", "L3", "L4", "L5"}
FORBIDDEN_REAL_PATTERNS = ("gazebo", "fake", "mock", "test_publisher", "simple_sim")


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=timeout, check=False)
        return p.returncode, (p.stdout + p.stderr).strip()
    except Exception as exc:
        return 99, str(exc)


def ros2(args: list[str], timeout: float = 5.0) -> tuple[int, str]:
    return run(["ros2", *args], timeout=timeout)


def topic_info(topic: str) -> dict[str, Any]:
    rc, out = ros2(["topic", "info", "-v", topic], timeout=5)
    info: dict[str, Any] = {
        "topic": topic,
        "rc": rc,
        "raw": out,
        "publishers": 0,
        "subscribers": 0,
        "nodes": [],
        "publisher_nodes": [],
        "subscriber_nodes": [],
    }
    if rc != 0:
        return info
    m = re.search(r"Publisher count:\s*(\d+)", out)
    if m:
        info["publishers"] = int(m.group(1))
    m = re.search(r"Subscription count:\s*(\d+)", out)
    if m:
        info["subscribers"] = int(m.group(1))
    info["nodes"] = re.findall(r"Node name:\s*([^\s]+)", out)
    current_node = ""
    for line in out.splitlines():
        node_match = re.search(r"Node name:\s*([^\s]+)", line)
        if node_match:
            current_node = node_match.group(1)
            continue
        endpoint_match = re.search(r"Endpoint type:\s*([A-Z]+)", line)
        if endpoint_match and current_node:
            endpoint_type = endpoint_match.group(1)
            if endpoint_type == "PUBLISHER":
                info["publisher_nodes"].append(current_node)
            elif endpoint_type == "SUBSCRIPTION":
                info["subscriber_nodes"].append(current_node)
            current_node = ""
    return info


def topic_once(topic: str, timeout_s: float = 4.0) -> tuple[bool, str]:
    rc, out = run(["timeout", str(timeout_s), "ros2", "topic", "echo", "--once", "--full-length", topic], timeout=timeout_s + 2)
    return rc == 0 and bool(out.strip()), out


def topic_rate(topic: str, timeout_s: float = 4.0) -> tuple[float, str]:
    rc, out = run(["timeout", str(timeout_s), "ros2", "topic", "hz", topic], timeout=timeout_s + 2)
    if rc != 0 and not out:
        return 0.0, out
    matches = re.findall(r"average rate:\s*([0-9.]+)", out)
    return (float(matches[-1]) if matches else 0.0), out


def tf_check(parent: str, child: str, timeout_s: float = 3.0) -> tuple[bool, str]:
    rc, out = run(["timeout", str(timeout_s), "ros2", "run", "tf2_ros", "tf2_echo", parent, child], timeout=timeout_s + 2)
    ok = rc == 0 and ("At time" in out or "Translation:" in out)
    return ok, out


def add_check(report: dict[str, Any], name: str, required: bool, ok: bool, detail: str) -> None:
    row = {"name": name, "required": required, "ok": ok, "detail": detail[:1200]}
    key = "required_checks" if required else "optional_checks"
    report[key].append(row)
    if required and not ok:
        report["failed_checks"].append(row)
    elif not required and not ok:
        report["warnings"].append(row)


def load_acceptance_matrix() -> list[dict[str, str]]:
    path = ROOT / "docs/hardware_acceptance_matrix.md"
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("|") or line.startswith("|---") or " item " in line:
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if len(cells) >= 6:
            rows.append(
                {
                    "item": cells[0],
                    "required_for_level": cells[1],
                    "evidence_command": cells[2],
                    "evidence_file": cells[3],
                    "status": cells[4],
                    "notes": cells[5],
                }
            )
    return rows


def level_index(level: str) -> int:
    return int(level[1])


def main() -> int:
    parser = argparse.ArgumentParser(description="Fail-closed Waver field readiness checker.")
    parser.add_argument("--level", required=True, choices=sorted(LEVELS))
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--no-hardware", action="store_true")
    parser.add_argument("--output", default="")
    parser.add_argument("--scan-topic", default=os.environ.get("SCAN_TOPIC", "/scan"))
    parser.add_argument("--scan-safety-topic", default=os.environ.get("SCAN_TOPIC_SAFETY", "/scan_safety"))
    parser.add_argument("--odom-topic", default=os.environ.get("ODOM_TOPIC", "/odom"))
    parser.add_argument("--min-scan-hz", type=float, default=float(os.environ.get("MIN_SCAN_HZ", "3.0")))
    parser.add_argument("--min-odom-hz", type=float, default=float(os.environ.get("MIN_ODOM_HZ", "5.0")))
    parser.add_argument("--serial-port", default=os.environ.get("SERIAL_PORT", ""))
    parser.add_argument("--odom-source", default=os.environ.get("ODOM_SOURCE", "ekf"))
    parser.add_argument("--require-scan", default=os.environ.get("REQUIRE_SCAN", "true"))
    parser.add_argument("--enable-waver-base-driver", default=os.environ.get("ENABLE_WAVER_BASE_DRIVER", "false"))
    parser.add_argument("--enable-bird-stack", default=os.environ.get("ENABLE_BIRD_STACK", "false"))
    parser.add_argument("--enable-sound-output", default=os.environ.get("ENABLE_SOUND_OUTPUT", "false"))
    args = parser.parse_args()

    now = time.strftime("%Y%m%d_%H%M%S")
    report: dict[str, Any] = {
        "level": args.level,
        "status": "FAIL",
        "required_checks": [],
        "optional_checks": [],
        "failed_checks": [],
        "warnings": [],
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "node_list": [],
        "topic_rates": {},
        "cmd_chain": {},
        "tf_status": {},
        "serial_status": {},
        "base_feedback_status": {},
        "battery_status": {},
        "forbidden_nodes": [],
        "manual_ack_status": {},
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }

    idx = level_index(args.level)
    if args.no_hardware and idx >= 2:
        add_check(report, "no_hardware_level_limit", True, False, "--no-hardware can only PASS L0/L1 checks")
    add_check(report, "ros_domain_id_present", False, bool(report["ros_domain_id"] != ""), f"ROS_DOMAIN_ID={report['ros_domain_id']}")

    if args.level == "L0":
        add_check(report, "source_mode_no_graph_required", True, True, "L0 relies on compile/static/unit checks outside this script")
    elif args.no_hardware and args.level == "L1":
        add_check(report, "dry_run_no_hardware_guard", True, True, "serial/sound/GPIO are not accessed in --no-hardware mode")
    else:
        rc, nodes_out = ros2(["node", "list"], timeout=5)
        nodes = sorted(line.strip() for line in nodes_out.splitlines() if line.strip().startswith("/"))
        report["node_list"] = nodes
        add_check(report, "ros_node_list", True, rc == 0, nodes_out)

        forbidden = [node for node in nodes if any(token in node.lower() for token in FORBIDDEN_REAL_PATTERNS)]
        report["forbidden_nodes"] = forbidden
        add_check(report, "real_profile_forbidden_nodes_absent", True, not forbidden, ",".join(forbidden))

        cmd = topic_info("/cmd_vel")
        report["cmd_chain"]["cmd_vel"] = cmd
        allowed_final_nodes = {"safety_cmd_mux_node"}
        if os.environ.get("WAVER_ALLOW_COLLISION_MONITOR_FINAL") == "1":
            allowed_final_nodes.add("collision_monitor")
        publisher_nodes = set(cmd.get("publisher_nodes", []))
        cmd_pub_ok = int(cmd.get("publishers", 0)) <= 1 and (not publisher_nodes or bool(publisher_nodes & allowed_final_nodes))
        add_check(report, "cmd_vel_final_publisher_count", idx >= 3, cmd_pub_ok, cmd.get("raw", ""))
        actuator_allowed = idx >= 3
        actuator_subs = int(cmd.get("subscribers", 0))
        add_check(report, "actuator_subscriber_policy", True, (actuator_subs == 0 if not actuator_allowed else actuator_subs == 1), f"subscribers={actuator_subs}")

        safety_ok, safety_sample = topic_once("/waver/safety_state")
        add_check(report, "safety_state_sample", idx >= 1, safety_ok, safety_sample)

        scan_topic = args.scan_topic
        if topic_info(args.scan_safety_topic).get("publishers", 0):
            scan_topic = args.scan_safety_topic
        if idx >= 2 or args.require_scan.lower() == "true":
            rate, raw = topic_rate(scan_topic)
            report["topic_rates"][scan_topic] = rate
            add_check(report, "scan_rate", idx >= 2, rate >= args.min_scan_hz, f"{scan_topic} hz={rate}\n{raw}")

        if idx >= 3:
            odom_rate, odom_raw = topic_rate(args.odom_topic)
            report["topic_rates"][args.odom_topic] = odom_rate
            add_check(report, "odom_rate", True, odom_rate >= args.min_odom_hz, f"{args.odom_topic} hz={odom_rate}\n{odom_raw}")
            base_ok, base_sample = topic_once("/waver/base_driver_state")
            report["base_feedback_status"]["sample"] = base_sample
            add_check(report, "base_driver_state_sample", True, base_ok, base_sample)
            serial_ok, serial_sample = topic_once("/waver/serial_owner_state")
            report["serial_status"]["sample"] = serial_sample
            add_check(report, "serial_owner_state_sample", True, serial_ok, serial_sample)
            add_check(report, "serial_by_id_path", True, args.serial_port.startswith("/dev/serial/by-id/"), args.serial_port)
            tf_ok, tf_raw = tf_check("odom", "base_link")
            report["tf_status"]["odom_to_base_link"] = tf_ok
            add_check(report, "tf_odom_to_base_link", True, tf_ok, tf_raw)

        if idx >= 4:
            volt_ok, volt_sample = topic_once("/voltage")
            report["battery_status"]["voltage_sample"] = volt_sample
            add_check(report, "battery_voltage_sample", True, volt_ok, volt_sample)
            for row in load_acceptance_matrix():
                if level_index(row["required_for_level"]) <= idx and row["status"].upper() != "OPTIONAL":
                    add_check(report, f"acceptance_matrix_{row['item']}", True, row["status"].upper() == "PASS", row["notes"])

    sound_enabled = args.enable_sound_output.lower() == "true"
    sound_ack = os.environ.get("WAVER_ACK_SOUND_HARDWARE") == "1" and os.environ.get("WAVER_ACK_LOCAL_SOUND_LAW") == "1"
    add_check(report, "sound_output_ack_policy", sound_enabled, (not sound_enabled) or sound_ack, "sound output requires WAVER_ACK_SOUND_HARDWARE=1 and WAVER_ACK_LOCAL_SOUND_LAW=1")

    failed = bool(report["failed_checks"])
    warned = bool(report["warnings"])
    if failed:
        report["status"] = "FAIL"
    elif warned and not args.strict:
        report["status"] = "PASS_LIMITED"
    else:
        report["status"] = "PASS"

    output = Path(args.output).expanduser() if args.output else ROOT / "reports/field_readiness" / f"{now}_{args.level}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    latest = output.parent / "latest.json"
    latest.write_text(output.read_text(encoding="utf-8"), encoding="utf-8")

    print(f"FIELD_READINESS_REPORT={output}")
    print(f"FAILED_CHECKS={len(report['failed_checks'])}")
    print(f"WARNINGS={len(report['warnings'])}")
    print(f"FIELD_READINESS={report['status']}")
    if args.strict and report["status"] != "PASS":
        return 1
    if report["status"] == "FAIL":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

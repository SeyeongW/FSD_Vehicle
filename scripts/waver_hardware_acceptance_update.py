#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_json(path: str) -> dict[str, Any]:
    p = Path(path).expanduser()
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text())
    except Exception as exc:
        return {"status": "REPORT_PARSE_FAIL", "findings": [str(exc)]}


def set_status(items: list[dict[str, Any]], item_name: str, status: str, evidence_file: str, notes: str = "") -> None:
    now = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    for item in items:
        if item.get("item") == item_name:
            item["status"] = status
            item["evidence_file"] = evidence_file
            item["last_checked"] = now
            if notes:
                item["notes"] = notes
            return


def status_from(report: dict[str, Any], ready: str, degraded_ok: bool = False) -> str:
    status = str(report.get("status", "")).upper()
    if status == ready:
        return "PASS"
    if degraded_ok and status.endswith("_DEGRADED"):
        return "BLOCKED"
    if not status:
        return "TODO"
    return "FAIL"


def main() -> int:
    parser = argparse.ArgumentParser(description="Update Waver hardware acceptance matrix from probe reports.")
    parser.add_argument("--matrix", default=str(ROOT / "config/hardware_acceptance_matrix.yaml"))
    parser.add_argument("--livox-report", default=str(ROOT / "reports/livox_mid360/latest.json"))
    parser.add_argument("--camera-report", default=str(ROOT / "reports/camera/latest.json"))
    parser.add_argument("--detector-report", default=str(ROOT / "reports/bird_detector/latest.json"))
    parser.add_argument("--calibration-report", default=str(ROOT / "reports/calibration/latest.json"))
    parser.add_argument("--base-report", default=str(ROOT / "reports/base_feedback/latest.json"))
    parser.add_argument("--command-chain-report", default=str(ROOT / "reports/command_chain/latest.json"))
    parser.add_argument("--network-report", default=str(ROOT / "reports/network/latest.json"))
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    matrix_path = Path(args.matrix).expanduser()
    data = yaml.safe_load(matrix_path.read_text()) if matrix_path.exists() else {"items": []}
    items = data.setdefault("items", [])

    livox = load_json(args.livox_report)
    camera = load_json(args.camera_report)
    detector = load_json(args.detector_report)
    calibration = load_json(args.calibration_report)
    base = load_json(args.base_report)
    command = load_json(args.command_chain_report)
    network = load_json(args.network_report)

    if network:
        net_status = "PASS" if str(network.get("status", "")).upper() in {"PASS", "NETWORK_READY"} else "FAIL"
        set_status(items, "ros_domain_id_match", net_status, args.network_report)
    if livox:
        lidar_status = status_from(livox, "LIDAR_READY", degraded_ok=True)
        set_status(items, "livox_mid360_pointcloud", lidar_status, args.livox_report)
        set_status(items, "scan_safety", lidar_status, args.livox_report)
        tf_status = "PASS" if livox.get("tf_livox_to_base_link_ready") is True else ("TODO" if not livox else "FAIL")
        set_status(items, "tf_base_livox", tf_status, args.livox_report)
    if camera:
        camera_status = status_from(camera, "CAMERA_READY", degraded_ok=True)
        set_status(items, "camera_image", camera_status, args.camera_report)
        set_status(items, "camera_info", camera_status, args.camera_report)
        set_status(items, "tf_base_camera", "PASS" if camera_status == "PASS" else camera_status, args.camera_report)
    if calibration:
        calibration_status = status_from(calibration, "CALIBRATION_READY")
        set_status(items, "camera_lidar_extrinsic", calibration_status, args.calibration_report)
        set_status(items, "fusion_sync", calibration_status, args.calibration_report)
    if detector:
        detector_status = status_from(detector, "DETECTOR_READY")
        set_status(items, "bird_detector_model", detector_status, args.detector_report)
    if base:
        text = json.dumps(base)
        base_ok = "PASS" if ("ODOM_FEEDBACK_OK" in text or base.get("odom_feedback_status") == "ODOM_FEEDBACK_OK") else "FAIL"
        serial_ok = "PASS" if ("connected=True" in text or base.get("connected") is True) else "FAIL"
        set_status(items, "base_driver_serial", serial_ok, args.base_report)
        set_status(items, "odom_feedback", base_ok, args.base_report)
        set_status(items, "tf_odom_base", base_ok, args.base_report)
        set_status(items, "robot_localization_ekf", base_ok, args.base_report)
        set_status(items, "voltage_scale", "BLOCKED", args.base_report, "External meter verification still required.")
    if command:
        command_status = "PASS" if str(command.get("status", "")).upper() in {"PASS", "WAVER_COMMAND_CHAIN=PASS"} else "FAIL"
        set_status(items, "final_cmd_vel_single_publisher", command_status, args.command_chain_report)
        set_status(items, "mission_mode_single_publisher", command_status, args.command_chain_report)
        set_status(items, "collision_monitor_active", command_status, args.command_chain_report)

    output = Path(args.output).expanduser() if args.output else matrix_path
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"HARDWARE_ACCEPTANCE_MATRIX_UPDATED={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, proc.stdout + proc.stderr
    except Exception as exc:
        return 124, str(exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate camera-LiDAR calibration artifact and optional live TF/topics.")
    parser.add_argument("--config", default=str(ROOT / "config/sensors/camera_lidar_extrinsic.yaml"))
    parser.add_argument("--extrinsic", dest="config", default=argparse.SUPPRESS)
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--pointcloud-topic", default="/livox/lidar")
    parser.add_argument("--require-live-tf", action="store_true")
    parser.add_argument("--require-live-topics", action="store_true")
    parser.add_argument("--output", default=str(ROOT / "reports/hardware_calibration/camera_lidar_latest.json"))
    parser.add_argument("--max-reprojection-error-px", type=float, default=5.0)
    args = parser.parse_args()
    path = Path(args.config).expanduser().resolve()
    data = yaml.safe_load(path.read_text()) if path.exists() else {}
    findings: list[str] = []
    status = "CALIBRATION_READY"
    if not path.exists():
        status = "CALIBRATION_FAIL"
        findings.append("extrinsic config missing")
    if not data.get("calibrated", False):
        status = "CALIBRATION_FAIL"
        findings.append("calibrated=false")
    err = data.get("reprojection_error_px")
    if err is None:
        status = "CALIBRATION_DEGRADED" if status == "CALIBRATION_READY" else status
        findings.append("reprojection_error_px missing")
    elif float(err) > args.max_reprojection_error_px:
        status = "CALIBRATION_FAIL"
        findings.append(f"reprojection_error_px too high: {err}")
    camera_frame = str(data.get("camera_frame", ""))
    lidar_frame = str(data.get("lidar_frame", ""))
    base_frame = str(data.get("base_frame", ""))
    for field, value in (("camera_frame", camera_frame), ("lidar_frame", lidar_frame), ("base_frame", base_frame)):
        if not value:
            status = "CALIBRATION_FAIL"
            findings.append(f"{field} missing")
    tf_base_lidar = run(["timeout", "3", "ros2", "run", "tf2_ros", "tf2_echo", base_frame or "base_link", lidar_frame or "livox"], timeout=4.0)[1]
    tf_base_camera = run(["timeout", "3", "ros2", "run", "tf2_ros", "tf2_echo", base_frame or "base_link", camera_frame or "camera_color_optical_frame"], timeout=4.0)[1]
    tf_camera_lidar = run(["timeout", "3", "ros2", "run", "tf2_ros", "tf2_echo", camera_frame or "camera_color_optical_frame", lidar_frame or "livox"], timeout=4.0)[1]
    camera_info_frame = run(["timeout", "3", "ros2", "topic", "echo", "--once", args.camera_info_topic, "--field", "header.frame_id"], timeout=4.0)[1].replace('"', "").strip()
    pointcloud_frame = run(["timeout", "3", "ros2", "topic", "echo", "--once", args.pointcloud_topic, "--field", "header.frame_id"], timeout=4.0)[1].replace('"', "").strip()
    tf_ready = all("Exception" not in text and "Invalid frame" not in text and text for text in (tf_base_lidar, tf_base_camera, tf_camera_lidar))
    live_topics_ready = bool(camera_info_frame and pointcloud_frame)
    if args.require_live_tf and not tf_ready:
        status = "CALIBRATION_FAIL"
        findings.append("live TF check failed")
    if args.require_live_topics and not live_topics_ready:
        status = "CALIBRATION_FAIL"
        findings.append("live camera_info/pointcloud sample missing")
    if camera_info_frame and camera_frame and camera_info_frame not in {camera_frame, camera_frame.replace("_link", "_optical_frame")}:
        status = "CALIBRATION_FAIL"
        findings.append(f"camera_info frame mismatch: {camera_info_frame} expected {camera_frame}")
    if pointcloud_frame and lidar_frame and pointcloud_frame != lidar_frame:
        status = "CALIBRATION_FAIL"
        findings.append(f"pointcloud frame mismatch: {pointcloud_frame} expected {lidar_frame}")
    report = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": status,
        "config": str(path),
        "camera_frame": camera_frame,
        "lidar_frame": lidar_frame,
        "base_frame": base_frame,
        "calibrated": bool(data.get("calibrated", False)),
        "reprojection_error_px": err,
        "camera_info_topic": args.camera_info_topic,
        "pointcloud_topic": args.pointcloud_topic,
        "camera_info_frame_id": camera_info_frame,
        "pointcloud_frame_id": pointcloud_frame,
        "tf_base_to_lidar_ready": "Exception" not in tf_base_lidar and "Invalid frame" not in tf_base_lidar and bool(tf_base_lidar),
        "tf_base_to_camera_ready": "Exception" not in tf_base_camera and "Invalid frame" not in tf_base_camera and bool(tf_base_camera),
        "tf_camera_to_lidar_ready": "Exception" not in tf_camera_lidar and "Invalid frame" not in tf_camera_lidar and bool(tf_camera_lidar),
        "live_topics_ready": live_topics_ready,
        "tf_probe_base_to_lidar": tf_base_lidar[-2000:],
        "tf_probe_base_to_camera": tf_base_camera[-2000:],
        "tf_probe_camera_to_lidar": tf_camera_lidar[-2000:],
        "findings": findings,
    }
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(status)
    print(f"CAMERA_LIDAR_CALIBRATION_REPORT={out}")
    return 0 if status == "CALIBRATION_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())

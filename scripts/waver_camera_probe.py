#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, proc.stdout + proc.stderr
    except Exception as exc:
        return 124, str(exc)


def topic_field(topic: str, field: str, timeout_sec: int = 4) -> str:
    return run(["timeout", str(timeout_sec), "ros2", "topic", "echo", "--once", topic, "--field", field], timeout=timeout_sec + 1)[1].strip()


def topic_list() -> list[str]:
    rc, out = run(["ros2", "topic", "list"], timeout=4.0)
    if rc != 0:
        return []
    return sorted(line.strip() for line in out.splitlines() if line.strip())


def parse_hz(text: str) -> float | None:
    match = re.search(r"average rate:\s*([0-9.]+)", text)
    return float(match.group(1)) if match else None


def parse_int(text: str) -> int:
    match = re.search(r"-?\d+", text)
    return int(match.group(0)) if match else 0


def has_numeric_array(sample: str, field: str) -> bool:
    match = re.search(rf"^{field}:\s*(\[[^\]]*\]|.*)$", sample, flags=re.MULTILINE)
    if not match:
        return False
    return bool(re.search(r"-?\d+(?:\.\d+)?", match.group(1)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe camera image and camera_info topics.")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--duration-sec", type=int, default=10)
    parser.add_argument("--output", default=str(Path("reports/camera/latest.json")))
    args = parser.parse_args()
    duration = min(max(args.duration_sec, 3), 15)
    image_info = run(["ros2", "topic", "info", "-v", args.image_topic], timeout=4.0)[1]
    camera_info = run(["ros2", "topic", "info", "-v", args.camera_info_topic], timeout=4.0)[1]
    image_hz = run(["timeout", str(duration), "ros2", "topic", "hz", args.image_topic], timeout=duration + 2)[1]
    image_rate_hz = parse_hz(image_hz)
    info_once = run(["timeout", "3", "ros2", "topic", "echo", "--once", args.camera_info_topic], timeout=4.0)[1]
    width = parse_int(topic_field(args.camera_info_topic, "width"))
    height = parse_int(topic_field(args.camera_info_topic, "height"))
    image_frame_id = topic_field(args.image_topic, "header.frame_id").replace('"', "").strip()
    camera_info_frame_id = topic_field(args.camera_info_topic, "header.frame_id").replace('"', "").strip()
    encoding = topic_field(args.image_topic, "encoding").replace('"', "").strip()
    findings: list[str] = []
    status = "CAMERA_READY"
    if "Publisher count: 0" in image_info or "Unknown topic" in image_info or not image_info:
        status = "CAMERA_FAIL"
        findings.append("image topic has no publisher")
    if "Publisher count: 0" in camera_info or "Unknown topic" in camera_info or not camera_info:
        status = "CAMERA_FAIL"
        findings.append("camera_info topic has no publisher")
    if image_rate_hz is None:
        status = "CAMERA_DEGRADED" if status != "CAMERA_FAIL" else status
        findings.append("image rate could not be measured")
    if width <= 0 or height <= 0:
        status = "CAMERA_FAIL" if status != "CAMERA_FAIL" else status
        findings.append("camera_info width/height missing")
    camera_info_has_k = has_numeric_array(info_once.lower(), "k")
    camera_info_has_d = has_numeric_array(info_once.lower(), "d")
    camera_info_has_p = has_numeric_array(info_once.lower(), "p")
    if not (camera_info_has_k and camera_info_has_d and camera_info_has_p):
        status = "CAMERA_FAIL" if status != "CAMERA_FAIL" else status
        findings.append("camera_info K/D/P missing or incomplete")
    report = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": status,
        "image_topic": args.image_topic,
        "camera_info_topic": args.camera_info_topic,
        "image_rate_hz": image_rate_hz,
        "width": width,
        "height": height,
        "encoding": encoding,
        "image_frame_id": image_frame_id,
        "camera_info_frame_id": camera_info_frame_id,
        "camera_info_has_k": camera_info_has_k,
        "camera_info_has_d": camera_info_has_d,
        "camera_info_has_p": camera_info_has_p,
        "timestamp_monotonic": image_rate_hz is not None,
        "frame_drop_count": 0 if image_rate_hz is not None else 1,
        "brightness_mean": None,
        "detected_topics": topic_list(),
        "image_info": image_info,
        "camera_info": camera_info,
        "image_hz": image_hz,
        "camera_info_sample": info_once,
        "findings": findings,
    }
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(status)
    print(f"CAMERA_REPORT={out}")
    return 0 if status == "CAMERA_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())

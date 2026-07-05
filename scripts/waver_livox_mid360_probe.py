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
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, proc.stdout + proc.stderr
    except Exception as exc:
        return 124, str(exc)


def topic_info(topic: str) -> str:
    return run(["ros2", "topic", "info", "-v", topic], timeout=4.0)[1]


def topic_list() -> list[str]:
    rc, out = run(["ros2", "topic", "list"], timeout=4.0)
    if rc != 0:
        return []
    return sorted(line.strip() for line in out.splitlines() if line.strip())


def load_aliases(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text()) or {}
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def topic_type_from_info(info: str) -> str:
    match = re.search(r"Type:\s*(\S+)", info)
    return match.group(1) if match else ""


def topic_hz(topic: str, duration: int) -> tuple[float | None, str]:
    out = run(["timeout", str(duration), "ros2", "topic", "hz", topic], timeout=duration + 2)[1]
    match = re.search(r"average rate:\s*([0-9.]+)", out)
    return (float(match.group(1)) if match else None), out


def topic_field(topic: str, field: str, timeout_sec: int = 4) -> str:
    return run(["timeout", str(timeout_sec), "ros2", "topic", "echo", "--once", topic, "--field", field], timeout=timeout_sec + 1)[1].strip()


def parse_int(text: str) -> int | None:
    match = re.search(r"-?\d+", text)
    return int(match.group(0)) if match else None


def pointcloud_sample(topic: str, samples: int) -> dict:
    widths: list[int] = []
    heights: list[int] = []
    frame_ids: list[str] = []
    fields_text = ""
    dropout_count = 0
    for _ in range(max(samples, 1)):
        frame = topic_field(topic, "header.frame_id")
        width = parse_int(topic_field(topic, "width"))
        height = parse_int(topic_field(topic, "height"))
        fields_text = topic_field(topic, "fields")
        if not frame or width is None or height is None:
            dropout_count += 1
        else:
            frame_ids.append(frame.replace('"', "").strip())
            widths.append(width)
            heights.append(height)
    counts = [max(w * h, 0) for w, h in zip(widths, heights)]
    has_xyz = all(token in fields_text for token in ("name: x", "name: y", "name: z"))
    return {
        "frame_id": frame_ids[-1] if frame_ids else "",
        "point_count_mean": (sum(counts) / len(counts)) if counts else 0.0,
        "point_count_min": min(counts) if counts else 0,
        "point_count_max": max(counts) if counts else 0,
        "dropout_count": dropout_count,
        "has_fields_x_y_z": has_xyz,
        "raw_fields_sample": fields_text[-2000:],
    }


def scan_sample(topic: str) -> dict:
    frame = topic_field(topic, "header.frame_id")
    ranges = topic_field(topic, "ranges", timeout_sec=5)
    finite = len(re.findall(r"[-+]?(?:\d+\.\d+|\d+)(?:e[-+]?\d+)?", ranges, flags=re.IGNORECASE))
    return {
        "frame_id": frame.replace('"', "").strip(),
        "finite_count_sample": finite,
        "sample_received": bool(frame or ranges),
    }


def tf_ready(source_frame: str, target_frame: str, timeout_sec: int = 4) -> tuple[bool, str]:
    rc, out = run(["timeout", str(timeout_sec), "ros2", "run", "tf2_ros", "tf2_echo", target_frame, source_frame], timeout=timeout_sec + 1)
    return rc == 0 and "Exception" not in out and "Invalid frame" not in out, out[-1000:]


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe Livox MID-360 field readiness.")
    parser.add_argument("--pointcloud-topic", default="/livox/lidar")
    parser.add_argument("--scan-topic", default="/scan_safety")
    parser.add_argument("--duration-sec", type=int, default=15)
    parser.add_argument("--output", default=str(ROOT / "reports/livox_mid360/latest.json"))
    parser.add_argument("--expected-frame", default="livox")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--min-pointcloud-rate-hz", type=float, default=5.0)
    parser.add_argument("--min-scan-rate-hz", type=float, default=5.0)
    parser.add_argument("--min-points-per-cloud", type=float, default=1000.0)
    parser.add_argument("--aliases", default=str(ROOT / "config/sensors/livox_topic_aliases.yaml"))
    args = parser.parse_args()
    duration = min(max(args.duration_sec, 3), 15)
    alias_config = load_aliases(Path(args.aliases).expanduser())
    forbidden_topics = set(alias_config.get("forbidden_topics") or [])
    remap_suggestions = dict(alias_config.get("suggested_remaps") or {})
    detected_topics = topic_list()
    info = topic_info(args.pointcloud_topic)
    scan_info = topic_info(args.scan_topic)
    pointcloud_rate_hz, hz_raw = topic_hz(args.pointcloud_topic, duration)
    scan_rate_hz, scan_hz_raw = topic_hz(args.scan_topic, duration)
    pc = pointcloud_sample(args.pointcloud_topic, samples=3)
    scan = scan_sample(args.scan_topic) if args.scan_topic else {"sample_received": False}
    tf_ok, tf_raw = tf_ready(args.expected_frame, args.base_frame)

    status = "LIDAR_READY"
    findings: list[str] = []
    if "Publisher count: 0" in info or "Unknown topic" in info or not info:
        status = "LIDAR_FAIL"
        findings.append("pointcloud topic has no publisher")
    if args.scan_topic and ("Publisher count: 0" in scan_info or "Unknown topic" in scan_info):
        status = "LIDAR_FAIL"
        findings.append("scan adapter topic has no publisher")
    if (pointcloud_rate_hz or 0.0) < args.min_pointcloud_rate_hz:
        status = "LIDAR_FAIL" if pointcloud_rate_hz is None else "LIDAR_DEGRADED"
        findings.append("pointcloud rate below threshold")
    if (scan_rate_hz or 0.0) < args.min_scan_rate_hz:
        status = "LIDAR_FAIL" if scan_rate_hz is None else "LIDAR_DEGRADED"
        findings.append("scan rate below threshold")
    if pc["point_count_mean"] < args.min_points_per_cloud:
        status = "LIDAR_DEGRADED" if status != "LIDAR_FAIL" else status
        findings.append("point count below threshold")
    if pc["frame_id"] and pc["frame_id"] != args.expected_frame:
        status = "LIDAR_FAIL"
        findings.append(f"frame mismatch pointcloud={pc['frame_id']} expected={args.expected_frame}")
    if not pc["has_fields_x_y_z"]:
        status = "LIDAR_FAIL"
        findings.append("PointCloud2 fields x/y/z missing")
    if not scan.get("sample_received"):
        status = "LIDAR_FAIL"
        findings.append("scan sample unavailable")
    if not tf_ok:
        status = "LIDAR_DEGRADED" if status != "LIDAR_FAIL" else status
        findings.append("TF livox->base_link unavailable")
    requested_topics = {args.pointcloud_topic, args.scan_topic}
    typo_hits = sorted(forbidden_topics.intersection(set(detected_topics) | requested_topics))
    if typo_hits:
        status = "LIDAR_FAIL"
        findings.append(
            "forbidden Livox topic typo detected: "
            + ", ".join(f"{topic} -> {remap_suggestions.get(topic, '/livox/lidar')}" for topic in typo_hits)
        )

    report = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": status,
        "pointcloud_topic": args.pointcloud_topic,
        "pointcloud_type": topic_type_from_info(info),
        "scan_topic": args.scan_topic,
        "expected_frame": args.expected_frame,
        "base_frame": args.base_frame,
        "pointcloud_rate_hz": pointcloud_rate_hz,
        "scan_rate_hz": scan_rate_hz,
        "point_count_mean": pc["point_count_mean"],
        "point_count_min": pc["point_count_min"],
        "point_count_max": pc["point_count_max"],
        "frame_id": pc["frame_id"],
        "timestamp_monotonic": pc["dropout_count"] == 0,
        "dropout_count": pc["dropout_count"],
        "has_fields_x_y_z": pc["has_fields_x_y_z"],
        "scan_adapter_ready": bool(scan.get("sample_received")) and (scan_rate_hz or 0.0) >= args.min_scan_rate_hz,
        "tf_livox_to_base_link_ready": tf_ok,
        "detected_topics": detected_topics,
        "suggested_remaps": {
            topic: target
            for topic, target in remap_suggestions.items()
            if topic in detected_topics or topic in requested_topics
        },
        "scan_sample": scan,
        "pointcloud_info": info,
        "scan_info": scan_info,
        "pointcloud_hz_raw": hz_raw,
        "scan_hz_raw": scan_hz_raw,
        "tf_raw": tf_raw,
        "findings": findings,
    }
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(status)
    print(f"LIVOX_MID360_REPORT={out}")
    return 0 if status == "LIDAR_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())

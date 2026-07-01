#!/usr/bin/env bash
set -euo pipefail

TOPIC="/scan"
STRICT=false
for arg in "$@"; do
  case "$arg" in
    --strict) STRICT=true ;;
    --help|-h)
      echo "Usage: $0 [scan_topic] [--strict]"
      exit 0
      ;;
    *) TOPIC="$arg" ;;
  esac
done

DURATION="${DURATION:-6.0}"
ADAPTER_STATE_TOPIC="${WAVER_LIVOX_SCAN_ADAPTER_STATE_TOPIC:-/waver/livox_scan_adapter_state}"
MIN_HZ="${WAVER_SCAN_MIN_HZ:-5.0}"
MIN_FINITE_RATIO="${WAVER_SCAN_MIN_FINITE_RATIO:-0.05}"
MIN_VALID_SCAN_POINTS="${WAVER_MIN_VALID_SCAN_POINTS:-${WAVER_SCAN_MIN_VALID_POINTS:-20}}"
MIN_ANGLE_SPAN_DEG="${WAVER_SCAN_MIN_ANGLE_SPAN_DEG:-90.0}"
FRONT_SECTOR_DEG="${WAVER_FRONT_SECTOR_DEG:-${WAVER_SCAN_FRONT_SECTOR_DEG:-55.0}}"
REAR_SECTOR_DEG="${WAVER_REAR_SECTOR_DEG:-${WAVER_SCAN_REAR_SECTOR_DEG:-55.0}}"
HARD_STOP_DISTANCE_M="${WAVER_HARD_STOP_DISTANCE_M:-${WAVER_SCAN_HARD_STOP_DISTANCE_M:-0.45}}"

python3 - "$TOPIC" "$DURATION" "$STRICT" "$ADAPTER_STATE_TOPIC" "$MIN_HZ" \
  "$MIN_FINITE_RATIO" "$MIN_VALID_SCAN_POINTS" "$MIN_ANGLE_SPAN_DEG" \
  "$FRONT_SECTOR_DEG" "$REAR_SECTOR_DEG" "$HARD_STOP_DISTANCE_M" <<'PY'
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

topic = sys.argv[1]
duration = float(sys.argv[2])
strict = sys.argv[3].lower() == "true"
adapter_topic = sys.argv[4]
min_hz = float(sys.argv[5])
min_finite_ratio = float(sys.argv[6])
min_valid_points = int(float(sys.argv[7]))
min_angle_span_deg = float(sys.argv[8])
front_sector = math.radians(float(sys.argv[9]) * 0.5)
rear_sector = math.radians(float(sys.argv[10]) * 0.5)
hard_stop = float(sys.argv[11])


class ScanQuality(Node):
    def __init__(self):
        super().__init__("waver_scan_quality_check")
        self.samples = []
        self.adapter_state = "UNKNOWN"
        self.adapter_seen = False
        self.create_subscription(LaserScan, topic, self.cb, 10)
        self.create_subscription(String, adapter_topic, self.adapter_cb, 10)

    def adapter_cb(self, msg):
        self.adapter_seen = True
        self.adapter_state = str(msg.data).strip().upper() or "UNKNOWN"

    def cb(self, msg):
        finite = []
        front = []
        rear = []
        for i, raw in enumerate(msg.ranges):
            value = float(raw)
            if not math.isfinite(value):
                continue
            if value < float(msg.range_min) or value > float(msg.range_max):
                continue
            finite.append(value)
            angle = float(msg.angle_min) + i * float(msg.angle_increment)
            if abs(angle) <= front_sector:
                front.append(value)
            if abs(abs(angle) - math.pi) <= rear_sector:
                rear.append(value)
        span = abs(float(msg.angle_max) - float(msg.angle_min))
        count = len(msg.ranges)
        self.samples.append(
            {
                "stamp": time.time(),
                "frame": msg.header.frame_id,
                "n": count,
                "finite": len(finite),
                "finite_ratio": len(finite) / float(count or 1),
                "min": min(finite) if finite else math.nan,
                "max": max(finite) if finite else math.nan,
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "span_deg": math.degrees(span),
                "angle_increment": float(msg.angle_increment),
                "front_min": min(front) if front else math.inf,
                "rear_min": min(rear) if rear else math.inf,
            }
        )


rclpy.init()
node = ScanQuality()
deadline = time.time() + duration
try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    samples = list(node.samples)
    adapter_state = node.adapter_state
    adapter_seen = node.adapter_seen
    node.destroy_node()
    rclpy.shutdown()

if not samples:
    print(f"STATUS=FAIL reason=NO_SCAN topic={topic}")
    sys.exit(1)

elapsed = max(samples[-1]["stamp"] - samples[0]["stamp"], 1e-6)
hz = (len(samples) - 1) / elapsed if len(samples) > 1 else 0.0
last = samples[-1]
print(f"SCAN_TOPIC={topic}")
print(f"SCAN_HZ={hz:.3f}")
print(f"SCAN_FRAME={last['frame']}")
print(f"RANGES={last['n']}")
print(f"FINITE_COUNT={last['finite']}")
print(f"FINITE_RATIO={last['finite_ratio']:.3f}")
print(f"ANGLE_SPAN_DEG={last['span_deg']:.1f}")
print(f"RANGE_MIN={last['range_min']:.3f}")
print(f"RANGE_MAX={last['range_max']:.3f}")
print(f"FINITE_MIN={last['min']}")
print(f"FINITE_MAX={last['max']}")
print(f"FRONT_MIN={last['front_min']}")
print(f"REAR_MIN={last['rear_min']}")
print(f"ADAPTER_STATE_TOPIC={adapter_topic}")
print(f"ADAPTER_STATE={adapter_state}")
print(f"ADAPTER_STATE_SEEN={str(adapter_seen).upper()}")

failures = []
warnings = []
if hz < min_hz:
    failures.append("HZ_LOW")
if last["n"] <= 0:
    failures.append("EMPTY_RANGES")
if last["finite"] < min_valid_points:
    failures.append("FINITE_COUNT_LOW")
if last["finite_ratio"] < min_finite_ratio:
    failures.append("FINITE_RATIO_LOW")
if last["span_deg"] < min_angle_span_deg:
    failures.append("ANGLE_SPAN_LOW")
if any(token in adapter_state for token in ("DEGRADED", "STALE", "FAILED", "EMPTY", "NO_POINTS", "NOT_ENABLED")):
    failures.append("ADAPTER_DEGRADED")
if strict and not adapter_seen:
    failures.append("ADAPTER_STATE_MISSING")
if last["front_min"] <= hard_stop:
    (failures if strict else warnings).append("FRONT_HARD_STOP_DISTANCE")
if last["rear_min"] <= hard_stop:
    (failures if strict else warnings).append("REAR_HARD_STOP_DISTANCE")

if failures:
    print("STATUS=FAIL reason=" + ",".join(failures))
    if warnings:
        print("WARNINGS=" + ",".join(warnings))
    sys.exit(1)
if warnings:
    print("STATUS=WARN reason=" + ",".join(warnings))
    sys.exit(0)
print("STATUS=OK")
PY

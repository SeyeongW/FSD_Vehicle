#!/usr/bin/env bash
set -euo pipefail

TOPIC="${1:-/scan}"
DURATION="${DURATION:-6.0}"

python3 - "$TOPIC" "$DURATION" <<'PY'
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

topic = sys.argv[1]
duration = float(sys.argv[2])


class ScanQuality(Node):
    def __init__(self):
        super().__init__("waver_scan_quality_check")
        self.samples = []
        self.create_subscription(LaserScan, topic, self.cb, 10)

    def cb(self, msg):
        finite = [v for v in msg.ranges if math.isfinite(float(v))]
        span = abs(float(msg.angle_max) - float(msg.angle_min))
        self.samples.append(
            {
                "stamp": time.time(),
                "frame": msg.header.frame_id,
                "n": len(msg.ranges),
                "finite": len(finite),
                "finite_ratio": len(finite) / float(len(msg.ranges) or 1),
                "min": min(finite) if finite else math.nan,
                "max": max(finite) if finite else math.nan,
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "span_deg": math.degrees(span),
                "angle_increment": float(msg.angle_increment),
            }
        )


rclpy.init()
node = ScanQuality()
deadline = time.time() + duration
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

samples = node.samples
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

failures = []
if hz < 5.0:
    failures.append("HZ_LOW")
if last["n"] <= 0:
    failures.append("EMPTY_RANGES")
if last["finite_ratio"] < 0.05:
    failures.append("FINITE_RATIO_LOW")
if last["span_deg"] < 90.0:
    failures.append("ANGLE_SPAN_LOW")
if failures:
    print("STATUS=FAIL reason=" + ",".join(failures))
    sys.exit(1)
print("STATUS=OK")
PY

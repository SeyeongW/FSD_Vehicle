#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
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


def import_check(name: str) -> tuple[bool, str]:
    rc, out = run(["python3", "-c", f"import {name}; print('OK')"], timeout=5.0)
    return rc == 0, out


def topic_available(topic: str) -> tuple[bool, str]:
    out = run(["ros2", "topic", "info", "-v", topic], timeout=4.0)[1]
    ok = "Publisher count: 0" not in out and "Unknown topic" not in out and "Could not determine" not in out
    return ok, out


def warmup_yolo(model_path: Path, samples: int) -> tuple[bool, list[float], list[str], str]:
    code = r"""
import json
import sys
import time

import numpy as np
from ultralytics import YOLO

model = YOLO(sys.argv[1])
names = getattr(model, "names", {}) or {}
classes = [str(v).strip().lower() for v in (names.values() if hasattr(names, "values") else names)]
image = np.zeros((480, 640, 3), dtype=np.uint8)
latencies = []
for _ in range(int(sys.argv[2])):
    t0 = time.monotonic()
    model.predict(image, verbose=False, conf=0.01)
    latencies.append((time.monotonic() - t0) * 1000.0)
print(json.dumps({"latencies_ms": latencies, "classes": classes}))
"""
    rc, out = run(["python3", "-c", code, str(model_path), str(max(samples, 1))], timeout=60.0)
    if rc != 0:
        return False, [], [], out[-2000:]
    try:
        payload = json.loads(out.strip().splitlines()[-1])
    except Exception as exc:
        return False, [], [], f"warmup parse failed: {exc}\n{out[-2000:]}"
    return True, [float(v) for v in payload.get("latencies_ms", [])], [str(v) for v in payload.get("classes", [])], out[-1000:]


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe bird detector deployment on Jetson/local system.")
    parser.add_argument("--model-path", default="")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--duration-sec", type=int, default=15)
    parser.add_argument("--warmup-samples", type=int, default=3)
    parser.add_argument("--registry", default=str(ROOT / "config/perception/bird_model_registry.yaml"))
    parser.add_argument("--model-name", default="default")
    parser.add_argument("--class-map-required", default="bird,person,vehicle,drone,unknown")
    parser.add_argument("--classification-requires-camera-alignment", default="false")
    parser.add_argument("--output", default=str(ROOT / "reports/bird_detector/latest.json"))
    args = parser.parse_args()

    registry_path = Path(args.registry).expanduser()
    registry: dict = {}
    registry_model: dict = {}
    if registry_path.exists():
        registry = yaml.safe_load(registry_path.read_text()) or {}
        registry_model = (registry.get("models") or {}).get(args.model_name, {})
    model_arg = args.model_path or str(registry_model.get("path", ""))
    model = Path(model_arg).expanduser() if model_arg else None
    registry_classes = registry_model.get("expected_classes") or []
    required_raw = ",".join(str(item) for item in registry_classes) if registry_classes else args.class_map_required
    required_classes = {item.strip().lower() for item in required_raw.split(",") if item.strip()}
    findings: list[str] = []
    status = "DETECTOR_READY"
    model_hash = ""
    latency_values: list[float] = []
    model_classes: list[str] = []

    if model is None or not model.exists():
        status = "DETECTOR_FAIL"
        findings.append("MODEL_MISSING")
    else:
        model_hash = hashlib.sha256(model.read_bytes()).hexdigest()

    ultra_ok, ultra_out = import_check("ultralytics")
    bridge_ok, bridge_out = import_check("cv_bridge")
    numpy_ok, numpy_out = import_check("numpy")
    if not ultra_ok:
        status = "DETECTOR_FAIL"
        findings.append("INFERENCE_BACKEND_UNAVAILABLE ultralytics")
    if not bridge_ok:
        status = "DETECTOR_FAIL"
        findings.append("INFERENCE_BACKEND_UNAVAILABLE cv_bridge")
    if not numpy_ok:
        status = "DETECTOR_FAIL"
        findings.append("INFERENCE_BACKEND_UNAVAILABLE numpy")

    image_ok, image_info = topic_available(args.image_topic)
    camera_info_ok, camera_info = topic_available(args.camera_info_topic)
    if not image_ok:
        status = "DETECTOR_FAIL"
        findings.append("camera image unavailable")
    if not camera_info_ok:
        status = "DETECTOR_FAIL"
        findings.append("camera_info unavailable")

    warmup_ok = False
    warmup_output = ""
    if model is not None and model.exists() and ultra_ok and numpy_ok:
        warmup_ok, latency_values, model_classes, warmup_output = warmup_yolo(model, args.warmup_samples)
        if not warmup_ok:
            status = "DETECTOR_FAIL"
            findings.append("WARMUP_INFERENCE_FAILED")
    else:
        warmup_output = "warmup skipped because model/import checks failed"

    normalized_classes = set(model_classes)
    class_map_ok = required_classes.issubset(normalized_classes)
    if warmup_ok and not class_map_ok:
        status = "DETECTOR_FAIL"
        findings.append("CLASS_MAP_MISSING " + ",".join(sorted(required_classes - normalized_classes)))

    report = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": status,
        "registry_path": str(registry_path),
        "registry_model": args.model_name,
        "registry_compatible": bool(registry_model),
        "fallback_unknown_class": bool(registry_model.get("fallback_unknown_class", True)),
        "model_path": str(model) if model else "",
        "model_sha256": model_hash,
        "ultralytics_import": ultra_ok,
        "cv_bridge_import": bridge_ok,
        "numpy_import": numpy_ok,
        "warmup_inference_ok": warmup_ok,
        "latencies_ms": latency_values,
        "mean_latency_ms": statistics.mean(latency_values) if latency_values else None,
        "max_latency_ms": max(latency_values) if latency_values else None,
        "class_names": model_classes,
        "class_map_required": sorted(required_classes),
        "class_map_ok": class_map_ok,
        "classification_requires_camera_alignment": args.classification_requires_camera_alignment.lower() == "true",
        "image_topic_available": image_ok,
        "camera_info_topic_available": camera_info_ok,
        "image_info": image_info,
        "camera_info": camera_info,
        "ultralytics_output": ultra_out[-1000:],
        "cv_bridge_output": bridge_out[-1000:],
        "numpy_output": numpy_out[-1000:],
        "warmup_output": warmup_output[-2000:],
        "findings": findings,
    }
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(status)
    print(f"BIRD_DETECTOR_REPORT={out}")
    return 0 if status == "DETECTOR_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


FAKE_TOKENS = ("fake", "mock", "sim_only", "gazebo_only", "synthetic")
REAL_TOKENS = ("real", "yolo", "onnx", "external")


def read_csv_rows(path: Path, limit: int = 200) -> list[dict[str, str]]:
    try:
        with path.open(newline="", errors="replace") as handle:
            reader = csv.DictReader(handle)
            return [row for _, row in zip(range(limit), reader)]
    except Exception:
        return []


def classify_text(value: str) -> str:
    lower = value.lower()
    if any(token in lower for token in FAKE_TOKENS):
        return "fake_or_sim"
    if any(token in lower for token in REAL_TOKENS):
        return "real_or_external"
    return "unknown"


def summarize(root: Path) -> dict[str, object]:
    files = sorted(root.rglob("*.csv"))
    evidence = {
        "root": str(root),
        "csv_files": len(files),
        "fake_detector_used": False,
        "fake_sound_used": False,
        "serial_enabled": False,
        "sim_or_real": "unknown",
        "detector_backend": "unknown",
        "evidence_level": "unknown",
        "observations": [],
    }
    observations: list[str] = []
    for path in files:
        for row in read_csv_rows(path):
            merged = " ".join(str(v) for v in row.values())
            lower = merged.lower()
            if "fake_detector" in lower or "fake camera" in lower:
                evidence["fake_detector_used"] = True
            if "fake_sound" in lower or "sound_state=fake" in lower:
                evidence["fake_sound_used"] = True
            if "serial_enabled=true" in lower or row.get("serial_enabled", "").lower() == "true":
                evidence["serial_enabled"] = True
            for key in ("evidence_level", "sim_or_real", "detector_backend"):
                value = row.get(key, "").strip()
                if value and evidence[key] == "unknown":
                    evidence[key] = value
            classification = classify_text(merged)
            if classification != "unknown":
                observations.append(f"{path.name}: {classification}")
    if evidence["sim_or_real"] == "unknown":
        evidence["sim_or_real"] = "sim" if evidence["fake_detector_used"] or evidence["fake_sound_used"] else "unknown"
    if evidence["evidence_level"] == "unknown" and evidence["sim_or_real"] == "sim":
        evidence["evidence_level"] = "L2"
    evidence["observations"] = sorted(set(observations))[:20]
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize Waver trial evidence level and fake/real component use.")
    parser.add_argument("path", nargs="?", default="experiment_results", help="Trial directory or experiment_results root")
    parser.add_argument("--json", action="store_true", help="Print JSON")
    args = parser.parse_args()

    report = summarize(Path(args.path).expanduser().resolve())
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        for key, value in report.items():
            print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

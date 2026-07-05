#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(cmd: list[str], timeout: float = 4.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, (proc.stdout + proc.stderr).strip()
    except Exception as exc:
        return 99, str(exc)


def topic_once(topic: str, timeout_s: float) -> tuple[bool, str]:
    rc, out = run(["timeout", str(timeout_s), "ros2", "topic", "echo", "--once", "--full-length", topic], timeout_s + 2)
    return rc == 0 and bool(out.strip()), out


def main() -> int:
    parser = argparse.ArgumentParser(description="Passive Waver field health supervisor.")
    parser.add_argument("--once", action="store_true", help="run one passive health check and exit")
    parser.add_argument("--period-sec", type=float, default=1.0)
    parser.add_argument("--output", default="")
    parser.add_argument("--require-base-driver", action="store_true")
    parser.add_argument("--require-scan", action="store_true")
    parser.add_argument("--scan-topic", default="/scan")
    args = parser.parse_args()

    def check_once() -> dict:
        report = {
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "status": "PASS",
            "checks": [],
        }

        def add(name: str, required: bool, ok: bool, detail: str) -> None:
            row = {"name": name, "required": required, "ok": ok, "detail": detail[:1000]}
            report["checks"].append(row)
            if required and not ok:
                report["status"] = "FAIL"
            elif not required and not ok and report["status"] == "PASS":
                report["status"] = "PASS_LIMITED"

        add("ros2_cli_available", True, shutil.which("ros2") is not None, shutil.which("ros2") or "missing")
        if shutil.which("ros2"):
            for topic, required in (
                ("/waver/safety_state", True),
                ("/waver/mode", True),
                ("/waver/base_driver_state", args.require_base_driver),
                ("/waver/serial_owner_state", args.require_base_driver),
                (args.scan_topic, args.require_scan),
            ):
                ok, detail = topic_once(topic, 2.5)
                add(f"topic_sample_{topic}", required, ok, detail)
        return report

    while True:
        report = check_once()
        payload = json.dumps(report, indent=2, ensure_ascii=False)
        if args.output:
            out = Path(args.output).expanduser()
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(payload + "\n", encoding="utf-8")
        print(f"WAVER_HEALTH_SUPERVISOR={report['status']}")
        if args.once:
            return 0 if report["status"] != "FAIL" else 1
        time.sleep(max(0.1, args.period_sec))


if __name__ == "__main__":
    raise SystemExit(main())

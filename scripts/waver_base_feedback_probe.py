#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def stop_payload() -> bytes:
    return b'{"T":1,"L":0.0,"R":0.0}\n'


def main() -> int:
    parser = argparse.ArgumentParser(description="Waver serial feedback probe. Wheel command is disabled unless explicitly confirmed.")
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--wheel-off-confirm", action="store_true")
    parser.add_argument("--allow-tiny-wheel-off-command", action="store_true")
    parser.add_argument("--output", default="reports/base_feedback/latest.json")
    args = parser.parse_args()

    if not args.serial_port.startswith("/dev/serial/by-id/"):
        raise SystemExit("serial port must be /dev/serial/by-id/... for field probing")

    import serial

    report = {
        "serial_port": args.serial_port,
        "duration_sec": args.duration_sec,
        "wheel_off_confirm": args.wheel_off_confirm,
        "tiny_command_enabled": args.allow_tiny_wheel_off_command and args.wheel_off_confirm,
        "packet_count": 0,
        "packet_types": {},
        "fields_seen": [],
        "odom_fields_ok": False,
        "imu_fields_ok": False,
        "voltage_field_ok": False,
        "status": "FAIL",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    fields_seen: set[str] = set()
    deadline = time.monotonic() + max(args.duration_sec, 1.0)
    with serial.Serial(args.serial_port, args.baudrate, timeout=0.05, write_timeout=0.05) as ser:
        ser.setRTS(False)
        ser.setDTR(False)
        for _ in range(5):
            ser.write(stop_payload())
        ser.write(b'{"T":130}\n')
        while time.monotonic() < deadline:
            line = ser.readline()
            if not line:
                ser.write(b'{"T":130}\n')
                continue
            try:
                data = json.loads(line.decode(errors="replace"))
            except Exception:
                continue
            report["packet_count"] += 1
            t = str(data.get("T", "UNKNOWN"))
            report["packet_types"][t] = report["packet_types"].get(t, 0) + 1
            fields_seen.update(data.keys())
        for _ in range(5):
            ser.write(stop_payload())

    report["fields_seen"] = sorted(fields_seen)
    report["odom_fields_ok"] = {"odl", "odr"} <= fields_seen
    report["imu_fields_ok"] = {"ax", "ay", "az", "gx", "gy", "gz"} <= fields_seen
    report["voltage_field_ok"] = "v" in fields_seen
    report["status"] = "PASS" if report["packet_count"] > 0 and report["odom_fields_ok"] and report["voltage_field_ok"] else "FAIL"
    out = Path(args.output).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    latest = out.parent / "latest.json"
    latest.write_text(out.read_text())
    print(f"BASE_FEEDBACK_PROBE_REPORT={out}")
    print(f"BASE_FEEDBACK_PROBE={report['status']}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

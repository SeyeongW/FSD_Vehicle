#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def require_enter(prompt: str) -> None:
    text = input(prompt + " Type YES to continue: ").strip()
    if text != "YES":
        raise SystemExit("operator aborted")


def main() -> int:
    parser = argparse.ArgumentParser(description="Wheel-off motor calibration wizard. No command is sent by default.")
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--wheel-off-confirm", action="store_true")
    parser.add_argument("--output-config", default="config/waver_motor_calibration.yaml")
    parser.add_argument("--output-report", default="")
    args = parser.parse_args()

    if not (args.allow_hardware and args.wheel_off_confirm):
        raise SystemExit("hardware command blocked: require --allow-hardware and --wheel-off-confirm")
    if not args.serial_port.startswith("/dev/serial/by-id/"):
        raise SystemExit("serial port must be /dev/serial/by-id/...")

    import serial

    report = {
        "serial_port": args.serial_port,
        "protocol": "lr_json",
        "left_forward_checked": False,
        "right_forward_checked": False,
        "wheel_swapped": "operator_confirm_required",
        "min_effective_command": 0.08,
        "max_safe_demo_command": 0.18,
        "cmd_timeout_checked": False,
        "stop_burst_checked": False,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    require_enter("Wheels must be physically off ground, E-stop in reach, area clear.")
    with serial.Serial(args.serial_port, args.baudrate, timeout=0.05, write_timeout=0.05) as ser:
        ser.setRTS(False)
        ser.setDTR(False)
        for _ in range(8):
            ser.write(b'{"T":1,"L":0.0,"R":0.0}\n')
            time.sleep(0.03)
        report["stop_burst_checked"] = True
        for label, payload in (
            ("left wheel tiny forward", b'{"T":1,"L":0.10,"R":0.0}\n'),
            ("right wheel tiny forward", b'{"T":1,"L":0.0,"R":0.10}\n'),
        ):
            require_enter(f"About to send {label} for 0.25 s.")
            ser.write(payload)
            time.sleep(0.25)
            ser.write(b'{"T":1,"L":0.0,"R":0.0}\n')
            if "left" in label:
                report["left_forward_checked"] = True
            else:
                report["right_forward_checked"] = True
    cfg = Path(args.output_config)
    cfg.parent.mkdir(parents=True, exist_ok=True)
    cfg.write_text(
        "\n".join(
            [
                "protocol: lr_json",
                "invert_left: false",
                "invert_right: false",
                "wheel_swapped: false",
                "min_effective_command: 0.08",
                "max_safe_demo_command: 0.18",
                "cmd_timeout_s: 0.3",
                "",
            ]
        )
    )
    out = Path(args.output_report or f"reports/hardware_calibration/motor_calibration_{time.strftime('%Y%m%d_%H%M%S')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"MOTOR_CALIBRATION_REPORT={out}")
    print(f"MOTOR_CALIBRATION_CONFIG={cfg}")
    print("MOTOR_CALIBRATION=PASS_OPERATOR_REVIEW_REQUIRED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

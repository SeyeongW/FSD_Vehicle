from __future__ import annotations

import argparse

from waver_patrol.comms.serial_json_client import SerialClientConfig, SerialJsonClient
from waver_patrol.config import WaverConfig


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Send repeated Waver stop commands.")
    parser.add_argument("--config", default="")
    parser.add_argument("--repeat", type=int, default=5)
    args = parser.parse_args(argv)
    cfg = WaverConfig.from_file(args.config if args.config else None)
    serial = cfg.section("serial")
    client = SerialJsonClient(
        SerialClientConfig(
            serial.get("preferred_ports", ["/dev/ttyTHS0", "/dev/serial0"]),
            serial.get("glob_ports", ["/dev/ttyUSB*", "/dev/ttyACM*"]),
            int(serial.get("baudrate", 115200)),
            float(serial.get("write_timeout_s", 0.05)),
            float(serial.get("reconnect_interval_s", 1.0)),
            args.repeat,
        )
    )
    if not client.connect():
        print("No serial port found. Stop command not sent; check port wiring and lsof output.")
        return
    try:
        client.send_stop(repeat=args.repeat)
        print(f"Sent stop {args.repeat} times on {client.port}")
    finally:
        client.close()

from __future__ import annotations

import argparse

from waver_patrol.comms.serial_json_client import SerialClientConfig, SerialJsonClient
from waver_patrol.config import WaverConfig
from waver_patrol.safety.acceleration_limiter import AccelerationLimiter
from waver_patrol.safety.command_sanitizer import CommandSanitizer
from waver_patrol.safety.runaway_guard import RunawayGuard
from waver_patrol.teleop.keyboard_curses import KeyboardTeleopState, run_curses


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="SSH-safe curses keyboard teleop for Waver.")
    parser.add_argument("--config", default="")
    parser.add_argument("--dry-run", action="store_true")
    args, unknown = parser.parse_known_args(argv)
    for idx, token in enumerate(unknown):
        if token.startswith("config:="):
            args.config = token.split(":=", 1)[1]
        elif token == "-p" and idx + 1 < len(unknown) and unknown[idx + 1].startswith("config:="):
            args.config = unknown[idx + 1].split(":=", 1)[1]
    cfg = WaverConfig.from_file(args.config if args.config else None)
    serial_cfg = cfg.section("serial")
    rover = cfg.section("rover")
    motor = cfg.section("motor")
    keyboard = cfg.section("keyboard")
    sanitizer = CommandSanitizer.from_mapping(rover, motor)
    accel = AccelerationLimiter()
    runaway = RunawayGuard(sanitizer=sanitizer, acceleration_limiter=accel)
    client = SerialJsonClient(
        SerialClientConfig(
            serial_cfg.get("preferred_ports", ["/dev/ttyTHS0", "/dev/serial0"]),
            serial_cfg.get("glob_ports", ["/dev/ttyUSB*", "/dev/ttyACM*"]),
            int(serial_cfg.get("baudrate", 115200)),
            float(serial_cfg.get("write_timeout_s", 0.05)),
            float(serial_cfg.get("reconnect_interval_s", 1.0)),
            int(rover.get("stop_repeat", 5)),
        )
    )
    if not args.dry_run and not client.connect():
        print("No serial port found. Running dry-run teleop.")
        args.dry_run = True

    def send(command):
        safe = sanitizer.sanitize(
            command.left,
            command.right,
            source=command.source,
            reason=command.reason,
            is_stop=command.is_stop,
        ).command
        safe = runaway.evaluate(safe).command
        if args.dry_run:
            print(safe.as_rover_json())
        else:
            client.send_command(safe)

    try:
        run_curses(
            send,
            KeyboardTeleopState(
                speed=float(rover.get("default_speed", 0.16)),
                turn_gain=float(rover.get("turn_gain", 0.65)),
                timeout_s=float(keyboard.get("input_timeout_s", 0.3)),
            ),
        )
    finally:
        if not args.dry_run:
            client.send_stop(repeat=int(rover.get("stop_repeat", 5)))
            client.close()

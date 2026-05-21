from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Any, Protocol

from waver_patrol.comms.port_detect import detect_ports
from waver_patrol.safety.command import WheelCommand


class SerialTransport(Protocol):
    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class SerialClientConfig:
    preferred_ports: list[str]
    glob_ports: list[str]
    baudrate: int = 115200
    write_timeout_s: float = 0.05
    reconnect_interval_s: float = 1.0
    stop_repeat: int = 5


class SerialJsonClient:
    def __init__(
        self,
        config: SerialClientConfig | None = None,
        transport: SerialTransport | None = None,
    ):
        self.config = config or SerialClientConfig(["/dev/ttyTHS0", "/dev/serial0"], ["/dev/ttyUSB*", "/dev/ttyACM*"])
        self.transport = transport
        self.port: str | None = None
        self.lock = threading.Lock()
        self.connected = transport is not None

    def connect(self) -> bool:
        if self.transport is not None:
            self.connected = True
            return True
        try:
            import serial
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("pyserial is required for serial transport") from exc
        for port in detect_ports(self.config.preferred_ports, self.config.glob_ports):
            try:
                self.transport = serial.Serial(
                    port,
                    self.config.baudrate,
                    timeout=0,
                    write_timeout=self.config.write_timeout_s,
                )
                self.port = port
                self.connected = True
                self.send_stop(repeat=self.config.stop_repeat)
                return True
            except Exception:
                self.transport = None
                self.connected = False
        return False

    def send_json(self, payload: dict[str, Any]) -> None:
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self.lock:
            if self.transport is None:
                raise RuntimeError("serial transport is not connected")
            self.transport.write(line.encode("utf-8"))
            try:
                self.transport.flush()
            except AttributeError:
                pass

    def send_command(self, command: WheelCommand) -> None:
        self.send_json(command.as_rover_json())

    def send_stop(self, repeat: int | None = None, delay_s: float = 0.02) -> None:
        count = self.config.stop_repeat if repeat is None else repeat
        for _ in range(max(1, count)):
            self.send_json({"T": 1, "L": 0, "R": 0})
            time.sleep(delay_s)

    def close(self) -> None:
        if self.transport is None:
            return
        try:
            self.send_stop(repeat=self.config.stop_repeat)
        finally:
            self.transport.close()
            self.connected = False


class FakeSerial:
    def __init__(self, fail_after: int | None = None):
        self.lines: list[bytes] = []
        self.closed = False
        self.fail_after = fail_after

    def write(self, data: bytes) -> int:
        if self.fail_after is not None and len(self.lines) >= self.fail_after:
            raise OSError("fake serial disconnect")
        self.lines.append(data)
        return len(data)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.closed = True

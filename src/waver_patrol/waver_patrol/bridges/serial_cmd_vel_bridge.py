from __future__ import annotations

import math
import signal
import threading
import time

from waver_patrol.bridges.cmd_vel_to_json import CmdVelToJson
from waver_patrol.comms.serial_json_client import SerialClientConfig, SerialJsonClient
from waver_patrol.config import WaverConfig
from waver_patrol.safety.command import TwistCommand, WheelCommand


class BridgeCore:
    """ROS-independent serial sender core.

    역할:
      - 최신 `/cmd_vel`이 stale이면 마지막 nonzero 명령을 재사용하지 않고 stop만 반복 송신한다.
      - serial reconnect 직후에도 반드시 stop 상태에서 재시작한다.
      - 실제 WAVE ROVER serial 소유자는 이 bridge 또는 기존 `ugv_driver` 중 하나만 있어야 한다.
    """

    def __init__(
        self,
        client: SerialJsonClient,
        converter: CmdVelToJson,
        rate_hz: float = 20.0,
        cmd_timeout_s: float = 0.3,
        stop_repeat: int = 5,
    ):
        self.client = client
        self.converter = converter
        self.rate_hz = rate_hz
        self.cmd_timeout_s = cmd_timeout_s
        self.stop_repeat = stop_repeat
        self.stop_event = threading.Event()
        self.latest = WheelCommand.stop(reason="bridge startup")
        self.last_cmd_time = 0.0
        self.estop = False
        self.external_stop = False
        self.thread: threading.Thread | None = None
        self.state_text = "STARTUP_STOP"
        self.lock = threading.Lock()

    def update_twist(self, linear_x: float, angular_z: float, source: str = "cmd_vel") -> WheelCommand:
        with self.lock:
            if not math.isfinite(linear_x) or not math.isfinite(angular_z):
                self.latest = WheelCommand.stop(source="safety_stop", reason="invalid twist")
                self.last_cmd_time = time.monotonic()
                self.state_text = "INVALID_TWIST_STOP"
                return self.latest
            if self.estop or self.external_stop:
                self.latest = WheelCommand.stop(source="safety_stop", reason="estop/external stop")
                self.last_cmd_time = time.monotonic()
                return self.latest
            self.latest = self.converter.convert(TwistCommand(linear_x, angular_z, source=source), source=source)
            self.last_cmd_time = time.monotonic()
            self.state_text = "CMD_FRESH"
            return self.latest

    def set_estop(self, value: bool) -> None:
        with self.lock:
            self.estop = bool(value)
            if self.estop:
                self.latest = WheelCommand.stop(source="safety_stop", reason="emergency stop")
                self.state_text = "EMERGENCY_STOP"

    def set_external_stop(self, value: bool) -> None:
        with self.lock:
            self.external_stop = bool(value)
            if self.external_stop:
                self.latest = WheelCommand.stop(source="safety_stop", reason="external stop")
                self.state_text = "EXTERNAL_STOP"

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(self.rate_hz, 1.0)
        while not self.stop_event.is_set():
            command = self._safe_command()
            try:
                self.client.send_command(command)
            except Exception as exc:
                self.state_text = f"SERIAL_WRITE_FAILED_STOP error={exc}"
                try:
                    self.client.connect()
                    self.latest = WheelCommand.stop(reason="reconnect stop")
                    self.last_cmd_time = 0.0
                    self.client.send_stop(repeat=1)
                except Exception:
                    pass
            time.sleep(period)

    def _safe_command(self) -> WheelCommand:
        with self.lock:
            if self.estop:
                self.state_text = "EMERGENCY_STOP"
                return WheelCommand.stop(reason="emergency stop")
            if self.external_stop:
                self.state_text = "EXTERNAL_STOP"
                return WheelCommand.stop(reason="external stop")
            if self.last_cmd_time == 0.0 or time.monotonic() - self.last_cmd_time > self.cmd_timeout_s:
                self.state_text = "CMD_TIMEOUT_STOP"
                return WheelCommand.stop(reason="cmd_vel timeout")
            return self.latest

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self.client.send_stop(repeat=self.stop_repeat)
            self.state_text = "SHUTDOWN_STOP_BURST"
        finally:
            if self.thread:
                self.thread.join(timeout=1.0)
            self.client.close()

    def state(self) -> str:
        port = self.client.port or "none"
        connected = "connected" if self.client.connected else "disconnected"
        return f"{self.state_text} port={port} {connected} estop={self.estop} external_stop={self.external_stop}"


def main(args: list[str] | None = None) -> None:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from std_msgs.msg import Bool, String
    except ImportError:
        raise RuntimeError("serial_cmd_vel_bridge requires ROS2 rclpy and geometry_msgs")

    class SerialCmdVelBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("serial_cmd_vel_bridge")
            self.declare_parameter("config", "")
            self.declare_parameter("cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
            self.declare_parameter("external_stop_topic", "/waver/external_stop")
            self.declare_parameter("state_topic", "/waver/serial_bridge_state")
            self.declare_parameter("cmd_timeout_s", 0.3)
            self.declare_parameter("serial_port", "")
            self.declare_parameter("baudrate", 115200)
            self.declare_parameter("command_rate_hz", 20.0)
            self.declare_parameter("stop_repeat", 5)
            self.declare_parameter("write_timeout_s", 0.05)
            self.declare_parameter("reconnect_interval_s", 1.0)

            config_path = self.get_parameter("config").get_parameter_value().string_value
            cfg = WaverConfig.from_file(config_path if config_path else None)
            serial_cfg = cfg.section("serial")
            rover_cfg = cfg.section("rover")
            preferred_ports = list(serial_cfg.get("preferred_ports", ["/dev/ttyTHS0", "/dev/serial0"]))
            requested_port = str(self.get_parameter("serial_port").value).strip()
            if requested_port:
                preferred_ports = [requested_port]
            client = SerialJsonClient(
                SerialClientConfig(
                    preferred_ports,
                    serial_cfg.get("glob_ports", ["/dev/ttyUSB*", "/dev/ttyACM*"]),
                    int(self.get_parameter("baudrate").value or serial_cfg.get("baudrate", 115200)),
                    float(self.get_parameter("write_timeout_s").value or serial_cfg.get("write_timeout_s", 0.05)),
                    float(self.get_parameter("reconnect_interval_s").value or serial_cfg.get("reconnect_interval_s", 1.0)),
                    int(self.get_parameter("stop_repeat").value or rover_cfg.get("stop_repeat", 5)),
                )
            )
            if not client.connect():
                self.get_logger().warn("No serial port connected; bridge will keep trying and publish stop state")
            self.core = BridgeCore(
                client,
                CmdVelToJson(),
                float(self.get_parameter("command_rate_hz").value or serial_cfg.get("command_rate_hz", 20.0)),
                float(self.get_parameter("cmd_timeout_s").value),
                int(self.get_parameter("stop_repeat").value),
            )
            self.core.start()
            self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
            self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_twist, 10)
            self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: self.core.set_estop(bool(m.data)), 10)
            self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), lambda m: self.core.set_external_stop(bool(m.data)), 10)
            self.create_timer(0.2, self.publish_state)
            self.get_logger().warn("Do not run ugv_driver and serial_cmd_vel_bridge on the same serial port.")

        def on_twist(self, msg: Twist) -> None:
            self.core.update_twist(float(msg.linear.x), float(msg.angular.z), source="cmd_vel")

        def publish_state(self) -> None:
            self.state_pub.publish(String(data=self.core.state()))

        def destroy_node(self) -> bool:
            self.core.stop()
            return super().destroy_node()

    rclpy.init(args=args)
    node = SerialCmdVelBridgeNode()

    def _handle_signal(_signum: int, _frame: object) -> None:
        node.core.stop()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

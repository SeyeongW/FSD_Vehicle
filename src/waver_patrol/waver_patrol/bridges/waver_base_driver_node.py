from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from waver_patrol.bridges.cmd_vel_to_json import CmdVelToJson
from waver_patrol.safety.command import TwistCommand, WheelCommand


class WaverBaseDriverNode(Node):
    """Single-owner real base serial driver.

    This node owns one serial port, reads base feedback, and writes only the final
    safety-filtered /cmd_vel. It is intended to replace split feedback/command
    serial ownership during real wheel-off and low-speed wheel-on tests.
    """

    def __init__(self) -> None:
        super().__init__("waver_base_driver_node")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("stop_repeat", 5)
        self.declare_parameter("serial_reconnect_interval_s", 1.0)

        self.serial_port = str(self.get_parameter("serial_port").value).strip()
        if not self.serial_port:
            raise RuntimeError("waver_base_driver_node requires serial_port=/dev/serial/by-id/<WAVER_SERIAL_ID>")
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.stop_repeat = int(self.get_parameter("stop_repeat").value)
        self.reconnect_interval_s = float(self.get_parameter("serial_reconnect_interval_s").value)

        self.converter = CmdVelToJson()
        self.serial = None
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.estop = False
        self.external_stop = False
        self.latest = WheelCommand.stop(reason="driver startup")
        self.last_cmd_time = 0.0
        self.last_feedback_time = 0.0
        self.state = "STARTUP_STOP"

        self.odom_raw_pub = self.create_publisher(Float32MultiArray, "odom/odom_raw", 50)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 50)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 20)
        self.voltage_pub = self.create_publisher(Float32, "voltage", 20)
        self.base_state_pub = self.create_publisher(String, "/waver/base_driver_state", 10)
        self.serial_owner_pub = self.create_publisher(String, "/waver/serial_owner_state", 10)

        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd_vel, 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), self.on_estop, 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), self.on_external_stop, 10)

        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()
        rate = float(self.get_parameter("command_rate_hz").value)
        self.command_timer = self.create_timer(1.0 / max(rate, 1.0), self.command_tick)
        self.state_timer = self.create_timer(0.2, self.publish_state)
        self.get_logger().warn(
            f"waver_base_driver_node owns serial port {self.serial_port}. "
            "Do not run ugv_bringup feedback, ugv_driver, or serial_cmd_vel_bridge on this port."
        )

    def open_serial(self) -> bool:
        try:
            import serial
        except ImportError as exc:
            self.state = f"SERIAL_IMPORT_FAILED {exc}"
            return False
        with self.serial_lock:
            if self.serial is not None and getattr(self.serial, "is_open", False):
                return True
            try:
                self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=0.05, write_timeout=0.05)
                self.state = "CONNECTED_STARTUP_STOP"
                self.send_stop_locked(repeat=self.stop_repeat)
                return True
            except Exception as exc:
                self.serial = None
                self.state = f"SERIAL_CONNECT_FAILED {exc}"
                return False

    def reader_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.open_serial():
                time.sleep(self.reconnect_interval_s)
                continue
            try:
                with self.serial_lock:
                    line = self.serial.readline() if self.serial is not None else b""
                if not line:
                    continue
                data = json.loads(line.decode("utf-8", errors="replace"))
                self.publish_feedback(data)
            except Exception as exc:
                self.state = f"FEEDBACK_READ_FAILED {exc}"
                time.sleep(0.05)

    def publish_feedback(self, data: dict[str, Any]) -> None:
        if int(data.get("T", 1001)) != 1001:
            return
        self.last_feedback_time = time.monotonic()

        odom_left = float(data.get("odl", 0.0)) / 100.0
        odom_right = float(data.get("odr", 0.0)) / 100.0
        self.odom_raw_pub.publish(Float32MultiArray(data=[odom_left, odom_right]))

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_imu_link"
        imu.linear_acceleration.x = 9.8 * float(data.get("ax", 0.0)) / 8192.0
        imu.linear_acceleration.y = 9.8 * float(data.get("ay", 0.0)) / 8192.0
        imu.linear_acceleration.z = 9.8 * float(data.get("az", 0.0)) / 8192.0
        imu.angular_velocity.x = math.pi * float(data.get("gx", 0.0)) / (16.4 * 180.0)
        imu.angular_velocity.y = math.pi * float(data.get("gy", 0.0)) / (16.4 * 180.0)
        imu.angular_velocity.z = math.pi * float(data.get("gz", 0.0)) / (16.4 * 180.0)
        self.imu_pub.publish(imu)

        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x = float(data.get("mx", 0.0)) * 0.15
        mag.magnetic_field.y = float(data.get("my", 0.0)) * 0.15
        mag.magnetic_field.z = float(data.get("mz", 0.0)) * 0.15
        self.mag_pub.publish(mag)

        self.voltage_pub.publish(Float32(data=float(data.get("v", 0.0)) / 100.0))

    def on_cmd_vel(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.latest = WheelCommand.stop(reason="invalid cmd_vel")
            self.last_cmd_time = time.monotonic()
            self.state = "INVALID_CMD_STOP"
            return
        if self.estop or self.external_stop:
            self.latest = WheelCommand.stop(reason="estop/external stop")
            self.last_cmd_time = time.monotonic()
            return
        self.latest = self.converter.convert(TwistCommand(float(msg.linear.x), float(msg.angular.z), source="cmd_vel"), source="cmd_vel")
        self.last_cmd_time = time.monotonic()
        self.state = "CMD_FRESH"

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)
        if self.estop:
            self.latest = WheelCommand.stop(reason="emergency stop")
            self.state = "EMERGENCY_STOP"

    def on_external_stop(self, msg: Bool) -> None:
        self.external_stop = bool(msg.data)
        if self.external_stop:
            self.latest = WheelCommand.stop(reason="external stop")
            self.state = "EXTERNAL_STOP"

    def command_tick(self) -> None:
        if not self.open_serial():
            return
        cmd = self.safe_command()
        try:
            with self.serial_lock:
                if self.serial is not None:
                    self.serial.write((json.dumps(cmd.as_rover_json()) + "\n").encode("utf-8"))
        except Exception as exc:
            self.state = f"SERIAL_WRITE_FAILED {exc}"
            with self.serial_lock:
                try:
                    if self.serial is not None:
                        self.serial.close()
                except Exception:
                    pass
                self.serial = None

    def safe_command(self) -> WheelCommand:
        if self.estop:
            self.state = "EMERGENCY_STOP"
            return WheelCommand.stop(reason="emergency stop")
        if self.external_stop:
            self.state = "EXTERNAL_STOP"
            return WheelCommand.stop(reason="external stop")
        if self.last_cmd_time == 0.0 or time.monotonic() - self.last_cmd_time > self.cmd_timeout_s:
            self.state = "CMD_TIMEOUT_STOP"
            return WheelCommand.stop(reason="cmd_vel timeout")
        return self.latest

    def send_stop_locked(self, repeat: int = 1) -> None:
        if self.serial is None:
            return
        stop = WheelCommand.stop(reason="serial stop").as_rover_json()
        for _ in range(max(1, repeat)):
            self.serial.write((json.dumps(stop) + "\n").encode("utf-8"))

    def publish_state(self) -> None:
        age = time.monotonic() - self.last_feedback_time if self.last_feedback_time else -1.0
        connected = self.serial is not None and getattr(self.serial, "is_open", False)
        text = f"{self.state} port={self.serial_port} connected={connected} feedback_age_sec={age:.2f}"
        self.base_state_pub.publish(String(data=text))
        self.serial_owner_pub.publish(String(data=f"waver_base_driver_node port={self.serial_port}"))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        with self.serial_lock:
            if self.serial is not None:
                try:
                    self.send_stop_locked(repeat=self.stop_repeat)
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaverBaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

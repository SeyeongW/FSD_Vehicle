#!/usr/bin/env python3
# Copyright 2026 Waver project contributors.
"""Bridge ROS `/cmd_vel` into WAVE ROVER serial JSON.

Role:
  - Provide the real-robot command path that Gazebo normally hides.
  - Convert Twist linear/angular commands into WAVE ROVER `T:1/L/R` ratios.
  - Fail-stop on stale commands, invalid values, serial errors, and shutdown.

Only run one hardware command path at a time. If `keyboard_ctrl --serial-json` is
controlling the rover directly, do not run this bridge at the same time.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from ugv_tools.waver_drive_assist import AssistConfig, DriveAssist, DriveCommand


# 역할: Twist를 WAVE ROVER 좌/우 비율로 바꾼 뒤 최종 범위를 제한한다.
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class WheelCommand:
    # 역할: WAVE ROVER ESP32에 보낼 좌/우 PWM 비율 명령이다.
    # 4륜 Waver에서는 left가 좌측 앞/뒤 바퀴 쌍, right가 우측 앞/뒤 바퀴 쌍을 의미한다.
    left: float = 0.0
    right: float = 0.0


# 역할: 실차 serial 출력과 Gazebo dry-run 테스트를 같은 인터페이스로 감싼다.
class JsonTransport:
    """Small serial/dry-run transport.

    Role:
      - Keep serial writes isolated from ROS callbacks.
      - Always serialize stop as `{"T":1,"L":0.0,"R":0.0}`.
    """

    def __init__(self, port: str, baudrate: int, dry_run: bool, reconnect_interval_s: float):
        self.port = port
        self.requested_port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.reconnect_interval_s = reconnect_interval_s
        self.serial = None
        self.last_open_attempt = 0.0
        self.last_error = ""

    def open(self) -> None:
        # 역할: Gazebo smoke test에서는 dry-run으로 serial port를 열지 않는다.
        if self.dry_run:
            return
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval_s:
            return
        self.last_open_attempt = now
        import serial

        try:
            port = self._resolved_port()
            self.serial = serial.Serial(port, self.baudrate, timeout=0, write_timeout=0.05)
            self.port = port
            self.last_error = ""
        except Exception as exc:
            self.serial = None
            self.last_error = str(exc)

    def write(self, command: WheelCommand) -> None:
        # 역할: 최종 하위 제어기 명령은 반드시 T:1/L/R JSON 한 줄로 직렬화한다.
        payload = {"T": 1, "L": round(command.left, 4), "R": round(command.right, 4)}
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        if self.dry_run:
            return
        if self.serial is None:
            self.open()
        if self.serial is None:
            raise RuntimeError(f"serial transport is not open: {self.last_error}")
        try:
            self.serial.write(line.encode("utf-8"))
        except Exception as exc:
            self.last_error = str(exc)
            self.close()
            raise

    def close(self) -> None:
        # 역할: serial port 사용을 마치면 명시적으로 닫아 ugv_driver와의 포트 충돌을 줄인다.
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _resolved_port(self) -> str:
        # 역할: auto 포트 모드에서 Jetson UART, serial0, USB/ACM 순서로 안전하게 탐색한다.
        if self.requested_port != "auto":
            return self.requested_port
        candidates = ["/dev/ttyTHS0", "/dev/serial0"]
        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
            candidates.extend(sorted(glob.glob(pattern)))
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        raise FileNotFoundError("no WAVE ROVER serial candidate found")


# 역할: ROS /cmd_vel을 실차 WAVE ROVER serial JSON으로 변환하는 마지막 하드웨어 경로다.
class WaverCmdVelSerialBridge(Node):
    """Safety-bounded cmd_vel to WAVE ROVER JSON bridge.

    Role:
      - Let Nav2, patrol, or other autonomy publish `/cmd_vel`.
      - Enforce low demo limits before the ESP32 sees a command.
      - Repeat the most recent safe command at a fixed rate and stop on timeout.
    """

    def __init__(self, transport: JsonTransport):
        super().__init__("waver_cmd_vel_serial_bridge")
        # 역할: 실차 적용 전 launch 파라미터로 포트/속도/장애물 제한을 보수적으로 조정한다.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_left_right", 0.5)
        self.declare_parameter("max_demo_ratio", 0.30)
        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 0.65)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 1.2)
        self.declare_parameter("reverse_speed", 0.08)
        self.declare_parameter("deadband", 0.01)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("stop_repeat", 5)
        self.declare_parameter("enable_scan_assist", True)
        self.declare_parameter("lidar_required", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("hard_stop_distance_m", 0.45)
        self.declare_parameter("slow_down_distance_m", 1.2)
        self.declare_parameter("min_valid_scan_points", 40)
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")

        # 역할: 파라미터 값을 내부 제한값으로 고정한다.
        self.transport = transport
        self.max_left_right = float(self.get_parameter("max_left_right").value)
        self.max_demo_ratio = float(self.get_parameter("max_demo_ratio").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.stop_repeat = int(self.get_parameter("stop_repeat").value)
        # 역할: serial로 보내기 전에도 한 번 더 저속/가속/scan 제한을 적용한다.
        self.assist = DriveAssist(
            AssistConfig(
                max_linear_speed=float(self.get_parameter("max_linear_speed").value),
                max_angular_speed=float(self.get_parameter("max_angular_speed").value),
                max_linear_accel=float(self.get_parameter("max_linear_accel").value),
                max_angular_accel=float(self.get_parameter("max_angular_accel").value),
                command_timeout_s=self.cmd_timeout_s,
                reverse_speed=float(self.get_parameter("reverse_speed").value),
                lidar_required=bool(self.get_parameter("lidar_required").value),
                hard_stop_distance_m=float(self.get_parameter("hard_stop_distance_m").value),
                slow_down_distance_m=float(self.get_parameter("slow_down_distance_m").value),
                min_valid_scan_points=int(self.get_parameter("min_valid_scan_points").value),
            )
        )

        topic = str(self.get_parameter("cmd_vel_topic").value)
        rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.latest = DriveCommand()
        self.last_msg_time = 0.0
        self.have_cmd = False
        self.emergency_stop_active = False
        self.external_stop_active = False
        self.last_serial_warn_time = 0.0

        # 역할: 하나의 최종 /cmd_vel만 구독하고, 오래된 명령은 0.3초 내 stop으로 바꾼다.
        self.create_subscription(Twist, topic, self.cmd_vel_callback, 20)
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.emergency_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("external_stop_topic").value),
            self.external_stop_callback,
            10,
        )
        if bool(self.get_parameter("enable_scan_assist").value):
            # 역할: 실차에서 /scan이 있으면 serial 직전 hard stop 방어선을 하나 더 둔다.
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self.scan_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(1.0 / max(rate_hz, 1.0), self.send_tick)
        self.get_logger().info(f"Waver cmd_vel serial bridge listening on {topic}")
        self.get_logger().warn(
            "Do not run ugv_driver/app.py and waver_cmd_vel_serial_bridge on the same serial port."
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        # 역할: ROS 속도 명령의 NaN/Inf를 막고 마지막 수신 시간을 기록한다.
        x = float(msg.linear.x)
        z = float(msg.angular.z)
        if not math.isfinite(x) or not math.isfinite(z):
            self.get_logger().error("Invalid cmd_vel NaN/inf received; stopping")
            self.latest = DriveCommand(reason="invalid cmd_vel stop")
            self.have_cmd = False
            return

        self.latest = DriveCommand(x, z, "cmd_vel", "ros cmd_vel")
        self.last_msg_time = time.monotonic()
        self.have_cmd = True

    def scan_callback(self, msg: LaserScan) -> None:
        # 역할: 실차 bridge도 scan sector 상태를 계속 갱신한다.
        self.assist.update_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

    def emergency_stop_callback(self, msg: Bool) -> None:
        # 역할: GUI/외부 E-Stop topic이 true면 어떤 /cmd_vel도 serial로 통과시키지 않는다.
        self.emergency_stop_active = bool(msg.data)

    def external_stop_callback(self, msg: Bool) -> None:
        # 역할: 물리 스위치나 외부 감시 노드가 정지를 요청할 때 최우선으로 반영한다.
        self.external_stop_active = bool(msg.data)

    def send_tick(self) -> None:
        # 역할: ESP32 하위 제어기에 20Hz로 최신 안전 명령을 반복 송신한다.
        command = self.latest
        if self.emergency_stop_active:
            command = DriveCommand(reason="emergency stop topic")
        elif self.external_stop_active:
            command = DriveCommand(reason="external stop topic")
        elif not self.have_cmd or time.monotonic() - self.last_msg_time > self.cmd_timeout_s:
            command = DriveCommand(reason="cmd_vel stale stop")
        assisted = self.assist.assisted_command(command)
        wheel_command = self.drive_to_wheel(assisted)
        try:
            self.transport.write(wheel_command)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_serial_warn_time > 1.0:
                self.last_serial_warn_time = now
                self.get_logger().error(
                    f"Serial write/reconnect failed; holding stop until recovered: {exc}"
                )
            self.latest = DriveCommand(reason="serial write failed stop")
            self.have_cmd = False

    def drive_to_wheel(self, command: DriveCommand) -> WheelCommand:
        # 역할: differential/skid-steer 근사식으로 linear/angular를 L/R 비율로 변환한다.
        left = self.linear_gain * command.linear - self.angular_gain * command.angular
        right = self.linear_gain * command.linear + self.angular_gain * command.angular
        limit = min(self.max_left_right, self.max_demo_ratio)
        left = 0.0 if abs(left) < self.deadband else clamp(left, -limit, limit)
        right = 0.0 if abs(right) < self.deadband else clamp(right, -limit, limit)
        return WheelCommand(left, right)

    def send_stop_burst(self) -> None:
        # 역할: 예외/종료 시 stop 명령을 여러 번 보내 하위 제어기 마지막 명령을 정지로 만든다.
        for _ in range(max(1, self.stop_repeat)):
            try:
                self.transport.write(WheelCommand())
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().debug(f"Stop write skipped during shutdown: {exc}")
            time.sleep(0.02)

    def destroy_node(self) -> bool:
        # 역할: ROS 노드 종료 경로에서도 serial close 전에 stop burst를 보장한다.
        self.send_stop_burst()
        self.transport.close()
        return super().destroy_node()


def parse_args(argv: list[str] | None) -> tuple[argparse.Namespace, list[str]]:
    # 역할: 실차 serial 포트 인자와 ROS 인자를 분리해 ros2 run에서 안전하게 사용한다.
    parser = argparse.ArgumentParser(description="Waver cmd_vel to serial JSON bridge")
    parser.add_argument("--serial-port", default="/dev/ttyTHS0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--reconnect-interval", type=float, default=1.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not open serial, useful for ROS smoke tests",
    )
    raw_args = sys.argv[1:] if argv is None else argv
    known, ros_args = parser.parse_known_args(raw_args)
    if argv is None:
        ros_args = [sys.argv[0], *ros_args]
    return known, ros_args


def main(args=None):
    # 역할: dry-run/Gazebo와 실차 serial 모드를 같은 노드로 실행한다.
    cli, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    transport = JsonTransport(
        cli.serial_port,
        cli.baudrate,
        cli.dry_run,
        cli.reconnect_interval,
    )
    transport.open()
    node = WaverCmdVelSerialBridge(transport)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

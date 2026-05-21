#!/usr/bin/env python3
# Copyright 2026 Waver project contributors.
"""Waver keyboard teleop for ROS/Gazebo and optional WAVE ROVER serial JSON.

Role:
  - Read local or SSH terminal keyboard input.
  - Convert familiar teleop keys into left/right WAVE ROVER style wheel ratios.
  - Publish safe low-speed `/cmd_vel` for Gazebo/ROS simulation by default.
  - Optionally send Waveshare JSON `{"T":1,"L":left,"R":right}` to hardware.

The Waveshare WAVE ROVER wiki documents `T:1` as the recommended left/right wheel
command. L/R are -0.5..0.5 ratios, and on WAVE ROVER these are PWM ratios rather
than closed-loop m/s velocities. This node keeps that mental model internally even
when publishing `Twist` for Gazebo.
"""

from __future__ import annotations

import argparse
import json
import math
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ugv_tools.waver_drive_assist import AssistConfig, DriveAssist, DriveCommand


# 역할: 터미널에 표시할 조작 안내 문구를 한 곳에서 관리한다.
HELP = """
Waver keyboard teleop
---------------------
Move:
  q/u  w/i  e/o
  a/j   k   d/l
  z/m  s/,  c/.

Stop/exit:
  Space or k : immediate stop
  ESC/Ctrl-C : stop repeatedly and exit

Adjust:
  + / - : speed ratio
  [ / ] : turn gain

Intent keys:
  p pause/resume patrol intent, r reset intent, n skip, b return home, h help
"""


# 역할: Waveshare/WAVE ROVER 계열에서 익숙한 키 배열을 전후/회전 의도로 변환한다.
MOVE_KEYS = {
    "w": (1.0, 0.0),
    "i": (1.0, 0.0),
    "s": (-1.0, 0.0),
    ",": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "j": (0.0, 1.0),
    "d": (0.0, -1.0),
    "l": (0.0, -1.0),
    "q": (1.0, 1.0),
    "u": (1.0, 1.0),
    "e": (1.0, -1.0),
    "o": (1.0, -1.0),
    "z": (-1.0, 1.0),
    "m": (-1.0, 1.0),
    "c": (-1.0, -1.0),
    ".": (-1.0, -1.0),
}


# 역할: 모든 수동 입력값을 안전 범위 안으로 제한한다.
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# 역할: NaN/Inf가 모터 명령으로 흘러가지 못하게 차단한다.
def finite(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("command value must be finite")
    return value


@dataclass
class WheelRatioCommand:
    """Left/right command in WAVE ROVER ratio space.

    Role:
      - Keep hardware and Gazebo control consistent.
      - Stop is always exactly 0,0 and never affected by later inversion logic.
    """

    left: float = 0.0
    right: float = 0.0
    reason: str = "stop"

    @property
    def is_stop(self) -> bool:
        # 역할: stop 명령은 inversion/swap 같은 후처리와 무관하게 항상 0,0으로 판단한다.
        return abs(self.left) < 1e-9 and abs(self.right) < 1e-9


# 역할: 실차 직접 제어가 필요할 때만 사용하는 선택적 serial JSON 출력기다.
class SerialJsonWriter:
    """Optional hardware writer for WAVE ROVER JSON commands.

    Role:
      - Send only sanitized `T:1/L/R` commands.
      - Repeat stop on shutdown because losing the terminal must not leave motion active.
    """

    def __init__(self, port: str, baudrate: int, stop_repeat: int):
        self.port = port
        self.baudrate = baudrate
        self.stop_repeat = stop_repeat
        self.serial = None

    def connect(self) -> None:
        # 역할: pyserial은 실차 모드에서만 필요하므로 지연 import로 Gazebo 테스트 의존성을 줄인다.
        import serial

        self.serial = serial.Serial(self.port, self.baudrate, timeout=0, write_timeout=0.05)

    def send(self, command: WheelRatioCommand) -> None:
        # 역할: WAVE ROVER 하위 제어기가 이해하는 {"T":1,"L":...,"R":...} 형식만 보낸다.
        if self.serial is None:
            return
        payload = {"T": 1, "L": command.left, "R": command.right}
        self.serial.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))

    def stop(self) -> None:
        # 역할: 종료/예외 상황에서 정지 명령을 여러 번 반복해 마지막 명령이 전진으로 남지 않게 한다.
        for _ in range(max(1, self.stop_repeat)):
            self.send(WheelRatioCommand())
            time.sleep(0.02)

    def close(self) -> None:
        # 역할: 포트를 닫기 전 항상 stop burst를 먼저 내보낸다.
        if self.serial is not None:
            self.stop()
            self.serial.close()
            self.serial = None


# 역할: SSH/로컬 키보드 입력을 안전한 저속 Twist 또는 선택적 serial JSON으로 변환한다.
class WaverKeyboard(Node):
    """Terminal keyboard node with safety limits.

    Role:
      - Source of manual commands.
      - Manual commands should outrank patrol/autonomy in the larger stack.
      - Timeout turns into stop so a stuck SSH session cannot keep driving.
    """

    def __init__(self, serial_writer: Optional[SerialJsonWriter] = None):
        super().__init__("keyboard_ctrl")
        # 역할: 실증 전/후에 launch 파라미터만으로 속도, LiDAR 보조, 출력 토픽을 조정한다.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_mode", "twist")
        self.declare_parameter("default_ratio", 0.16)
        self.declare_parameter("max_ratio", 0.22)
        self.declare_parameter("turn_gain", 0.65)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 1.2)
        self.declare_parameter("reverse_speed", 0.08)
        self.declare_parameter("input_timeout_s", 0.3)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("stop_repeat", 5)
        self.declare_parameter("enable_scan_assist", True)
        self.declare_parameter("lidar_required", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("hard_stop_distance_m", 0.45)
        self.declare_parameter("slow_down_distance_m", 1.2)
        self.declare_parameter("min_valid_scan_points", 40)
        self.declare_parameter("mode_topic", "/waver/mode")

        # 역할: ROS 파라미터를 노드 내부 안전 제한값으로 확정한다.
        topic = self.get_parameter("cmd_vel_topic").value
        self.output_mode = str(self.get_parameter("output_mode").value)
        self.speed_ratio = float(self.get_parameter("default_ratio").value)
        self.max_ratio = float(self.get_parameter("max_ratio").value)
        self.turn_gain = float(self.get_parameter("turn_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.stop_repeat = int(self.get_parameter("stop_repeat").value)
        self.enable_scan_assist = bool(self.get_parameter("enable_scan_assist").value)

        # 역할: Gazebo와 ROS 제어기는 기본적으로 Twist를 받으므로 /cmd_vel 계열로 발행한다.
        self.publisher = self.create_publisher(Twist, topic, 10)
        # 역할: 터미널 키보드가 수동 우선권을 잡고 있음을 다른 Waver 노드가 볼 수 있게 한다.
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("mode_topic").value),
            10,
        )

        # 역할: 키 입력이 들어와도 최종 명령은 공통 DriveAssist를 거쳐 저속/가속/장애물 제한을 적용한다.
        self.assist = DriveAssist(
            AssistConfig(
                max_linear_speed=self.max_linear_speed,
                max_angular_speed=self.max_angular_speed,
                max_linear_accel=self.max_linear_accel,
                max_angular_accel=self.max_angular_accel,
                command_timeout_s=self.input_timeout_s,
                reverse_speed=float(self.get_parameter("reverse_speed").value),
                lidar_required=bool(self.get_parameter("lidar_required").value),
                hard_stop_distance_m=float(self.get_parameter("hard_stop_distance_m").value),
                slow_down_distance_m=float(self.get_parameter("slow_down_distance_m").value),
                min_valid_scan_points=int(self.get_parameter("min_valid_scan_points").value),
            )
        )
        if self.enable_scan_assist:
            # 역할: /scan이 있으면 수동 조작 중에도 전방 hard stop/slow down을 독립 적용한다.
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self.scan_callback,
                qos_profile_sensor_data,
            )
        # 역할: 터미널 상태와 최근 명령을 보관해 SSH 세션 끊김/키 입력 timeout에 대응한다.
        self.serial_writer = serial_writer
        self.settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        self.last_key_time = 0.0
        self.latest_command = WheelRatioCommand()
        self.running = True
        self.last_mode_publish = ""

    def read_key(self) -> str:
        # 역할: curses 없이 raw terminal에서 20Hz 루프를 유지하며 비동기 키 입력만 읽는다.
        ready, _, _ = select.select([sys.stdin], [], [], 0.02)
        return sys.stdin.read(1) if ready else ""

    def scan_callback(self, msg: LaserScan) -> None:
        # 역할: LaserScan을 공통 sector summary로 갱신해 수동 주행 중 전방 충돌을 줄인다.
        self.assist.update_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

    def command_from_key(self, key: str) -> WheelRatioCommand:
        # 역할: 단일 키 입력을 WAVE ROVER 좌/우 PWM 비율 명령으로 변환한다.
        key_lower = key.lower()
        self.last_key_time = time.monotonic()

        if key in {" ", "\x1b", "\x03"} or key_lower == "k":
            # 역할: Space/K/ESC/Ctrl-C는 어떤 상태에서도 즉시 정지 의도로 처리한다.
            return WheelRatioCommand(reason="operator stop")
        if key == "+":
            self.speed_ratio = min(self.max_ratio, self.speed_ratio + 0.02)
            return WheelRatioCommand(reason="speed increased")
        if key == "-":
            self.speed_ratio = max(0.04, self.speed_ratio - 0.02)
            return WheelRatioCommand(reason="speed decreased")
        if key == "[":
            self.turn_gain = max(0.2, self.turn_gain - 0.05)
            return WheelRatioCommand(reason="turn gain decreased")
        if key == "]":
            self.turn_gain = min(1.0, self.turn_gain + 0.05)
            return WheelRatioCommand(reason="turn gain increased")
        if key_lower == "h":
            print(HELP)
            return WheelRatioCommand(reason="help")
        if key_lower in {"p", "r", "g", "n", "b"}:
            # 역할: 순찰 제어 의도 키는 아직 별도 관리자에 연결하지 않고 안전하게 정지로 처리한다.
            self.get_logger().info(f"Intent key '{key_lower}' captured; stopping manual motion.")
            return WheelRatioCommand(reason=f"intent {key_lower}")
        if key_lower not in MOVE_KEYS:
            return WheelRatioCommand(reason="unknown key")

        forward, turn = MOVE_KEYS[key_lower]
        left = self.speed_ratio * forward - self.speed_ratio * self.turn_gain * turn
        right = self.speed_ratio * forward + self.speed_ratio * self.turn_gain * turn
        return self.sanitize(left, right, reason=f"key {key_lower}")

    def sanitize(self, left: float, right: float, reason: str) -> WheelRatioCommand:
        # 역할: 좌/우 PWM 비율을 float 변환 후 -max_ratio~max_ratio로 제한한다.
        left = clamp(finite(float(left)), -self.max_ratio, self.max_ratio)
        right = clamp(finite(float(right)), -self.max_ratio, self.max_ratio)
        return WheelRatioCommand(left, right, reason)

    def wheel_to_drive(self, command: WheelRatioCommand) -> DriveCommand:
        # 역할: WAVE ROVER 좌/우 비율 모델을 Gazebo용 linear/angular 속도 후보로 근사 변환한다.
        if command.is_stop:
            return DriveCommand(source="keyboard", reason=command.reason)
        normalized_linear = (command.left + command.right) / (2.0 * max(self.max_ratio, 1e-6))
        normalized_turn = (command.right - command.left) / (2.0 * max(self.max_ratio, 1e-6))
        return DriveCommand(
            linear=clamp(
                normalized_linear * self.max_linear_speed,
                -self.max_linear_speed,
                self.max_linear_speed,
            ),
            angular=clamp(
                normalized_turn * self.max_angular_speed,
                -self.max_angular_speed,
                self.max_angular_speed,
            ),
            source="keyboard",
            reason=command.reason,
        )

    def drive_to_twist(self, command: DriveCommand) -> Twist:
        # 역할: DriveAssist가 만든 최종 후보 명령을 ROS 표준 Twist 메시지로 변환한다.
        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        return twist

    def drive_to_wheel(self, command: DriveCommand) -> WheelRatioCommand:
        # 역할: 안전 필터 후의 Twist 후보를 다시 실차 serial용 좌/우 비율로 변환한다.
        if command.is_stop:
            return WheelRatioCommand(reason=command.reason)
        normalized_linear = command.linear / max(self.max_linear_speed, 1e-6)
        normalized_turn = command.angular / max(self.max_angular_speed, 1e-6)
        left = (normalized_linear - normalized_turn) * self.max_ratio
        right = (normalized_linear + normalized_turn) * self.max_ratio
        return self.sanitize(left, right, reason=command.reason)

    def publish_command(self, command: WheelRatioCommand, mode: str = "MANUAL") -> None:
        # 역할: 키 입력 명령을 공통 안전 보조 계층에 통과시킨 뒤 ROS/serial로 내보낸다.
        if command.is_stop and mode == "MANUAL":
            stop_words = ("stop", "timeout", "unknown", "intent", "help")
            if any(word in command.reason for word in stop_words):
                mode = "STANDBY"
        self.publish_mode(mode)
        assisted = self.assist.assisted_command(self.wheel_to_drive(command))
        try:
            self.publisher.publish(self.drive_to_twist(assisted))
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().debug(f"ROS Twist publish skipped: {exc}")
        if self.output_mode == "serial_json" and self.serial_writer is not None:
            self.serial_writer.send(self.drive_to_wheel(assisted))

    def publish_stop(self, repeat: int | None = None) -> None:
        # 역할: 노드 종료, timeout, 예외 상황에서 정지 명령을 반복 발행한다.
        self.publish_mode("STANDBY")
        count = self.stop_repeat if repeat is None else repeat
        stop = WheelRatioCommand()
        for _ in range(max(1, count)):
            self.publish_command(stop, mode="STANDBY")
            time.sleep(0.02)

    def publish_mode(self, mode: str) -> None:
        # 역할: 같은 모드 문자열을 반복 송신하지 않으면서 수동/정지 상태를 외부에 알린다.
        if mode == self.last_mode_publish:
            return
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.last_mode_publish = mode

    def spin_keyboard(self) -> None:
        # 역할: 실제 사람이 조작하는 interactive 루프다. 입력이 끊기면 0.3초 내 정지한다.
        print(HELP)
        if self.settings is None:
            raise RuntimeError(
                "interactive keyboard mode requires a TTY; use --scripted-keys for tests"
            )
        period = 1.0 / max(self.command_rate_hz, 1.0)
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok() and self.running:
                key = self.read_key()
                if key:
                    self.latest_command = self.command_from_key(key)
                    if key in {"\x1b", "\x03"}:
                        self.running = False
                elif (
                    self.last_key_time
                    and time.monotonic() - self.last_key_time > self.input_timeout_s
                ):
                    self.latest_command = WheelRatioCommand(reason="keyboard timeout")

                self.publish_command(self.latest_command)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
        finally:
            self.publish_stop()
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            if self.serial_writer is not None:
                self.serial_writer.close()

    def spin_scripted(self, keys: str, interval_s: float) -> None:
        """Run the same key path from a scripted sequence.

        Role:
          - Make Gazebo/CI testing deterministic when no human TTY is present.
          - Exercise command_from_key(), assist filtering, timeout, and stop logic.
        """
        # 역할: Gazebo/CI에서 사람 없이 같은 키 경로를 재현한다.
        print(f"Running scripted Waver keyboard test: {keys!r}")
        try:
            for key in keys:
                self.latest_command = self.command_from_key(key)
                self.publish_command(self.latest_command)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(max(0.02, interval_s))
            timeout_deadline = time.monotonic() + self.input_timeout_s + 0.2
            while rclpy.ok() and time.monotonic() < timeout_deadline:
                if (
                    self.last_key_time
                    and time.monotonic() - self.last_key_time > self.input_timeout_s
                ):
                    self.latest_command = WheelRatioCommand(reason="keyboard timeout")
                self.publish_command(self.latest_command)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(1.0 / max(self.command_rate_hz, 1.0))
        finally:
            self.publish_stop()


def parse_args(argv: list[str] | None) -> tuple[argparse.Namespace, list[str]]:
    # 역할: ROS 인자와 일반 CLI 인자를 분리해 ros2 run 양쪽 사용성을 유지한다.
    parser = argparse.ArgumentParser(description="Waver keyboard controller")
    parser.add_argument(
        "--serial-json",
        action="store_true",
        help="Also send WAVE ROVER T:1 JSON over serial",
    )
    parser.add_argument("--serial-port", default="/dev/ttyTHS0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--scripted-keys",
        default="",
        help="Replay keys for Gazebo/CI tests, e.g. 'wwdk'",
    )
    parser.add_argument("--scripted-interval", type=float, default=0.15)
    raw_args = sys.argv[1:] if argv is None else argv
    known, ros_args = parser.parse_known_args(raw_args)
    if argv is None:
        ros_args = [sys.argv[0], *ros_args]
    return known, ros_args


def main(args=None):
    # 역할: 노드 시작/종료 시 stop burst가 빠지지 않도록 생명주기를 한 곳에서 관리한다.
    cli, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    serial_writer = None
    if cli.serial_json:
        print(
            "WARNING: --serial-json bypasses the safer /cmd_vel serial bridge path. "
            "Use it only for bench fallback tests."
        )
        serial_writer = SerialJsonWriter(cli.serial_port, cli.baudrate, stop_repeat=5)
        serial_writer.connect()

    node = WaverKeyboard(serial_writer)
    if cli.serial_json:
        node.output_mode = "serial_json"

    def _handle_signal(_signum, _frame):
        # 역할: systemd/launch 종료 신호를 받으면 루프가 finally 블록으로 들어가 정지하게 한다.
        node.running = False

    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        if cli.scripted_keys:
            node.spin_scripted(cli.scripted_keys, cli.scripted_interval)
        else:
            node.spin_keyboard()
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

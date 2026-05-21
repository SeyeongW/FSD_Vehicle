#!/usr/bin/env python3
# Copyright 2026 Waver project contributors.
"""Waver visual remote panel for manual and Nav2 mission control.

역할:
  - 작은 Tkinter 창에서 Waver 수동 조작과 자율순찰 시작/정지를 한 곳에 모은다.
  - MANUAL 모드에서는 방향 버튼/키보드 입력을 /waver/manual_cmd_vel 후보로 발행한다.
  - AUTO 버튼은 /waver/mode=AUTO를 발행해 Waver mission/Nav2 백엔드를 시작/재개시킨다.
  - AUTO 주행 중 방향키를 누르면 AUTO 모드는 유지하고 manual 후보만 잠깐 올려
    safety_cmd_mux_node가 수동 override 후 다시 Nav2 경로로 복귀하게 한다.
  - 최종 /cmd_vel은 기본적으로 safety_cmd_mux_node만 발행한다.

주의:
  - 실차에서는 /cmd_vel을 실제 serial bridge 하나만 받아야 한다.
  - 기존 ugv_driver와 waver_cmd_vel_serial_bridge를 동시에 serial에 붙이지 말 것.
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from ugv_tools.waver_drive_assist import AssistConfig, DriveAssist, DriveCommand


@dataclass
class PanelState:
    # 역할: GUI 스레드와 ROS 스레드가 공유하는 현재 조작 상태다.
    mode: str = "STANDBY"
    desired_linear: float = 0.0
    desired_angular: float = 0.0
    active_control: str = "none"
    emergency_stop: bool = False
    speed_limit: float = 0.16
    angular_limit: float = 0.6
    latest_cmd_linear: float = 0.0
    latest_cmd_angular: float = 0.0
    latest_auto_linear: float = 0.0
    latest_auto_angular: float = 0.0
    latest_auto_age: float = 999.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    patrol_state: str = "not started"
    mission_state: str = "unknown"
    safety_state: str = "unknown"
    radar_state: str = "unknown"
    object_goal_state: str = "unknown"
    battery_state: str = "unknown"
    target_class: str = "unknown"
    target_confidence: float = 0.0
    bird_confirmed: bool = False
    auto_status: str = "stopped"


class WaverRemoteNode(Node):
    """ROS side of the visual remote.

    역할:
      - 기본 실차 모드에서는 GUI 입력을 `/waver/manual_cmd_vel` 후보로만 발행한다.
      - 최종 `/cmd_vel`은 `safety_cmd_mux_node`가 단독으로 발행하게 둔다.
      - AUTO 버튼은 `/waver/mode=AUTO`를 발행해 mission/Nav2 백엔드를 시작 또는 재개시킨다.
      - 레거시 Gazebo 단독 실험이 필요할 때만 `publish_direct_cmd_vel:=true`로 직접 `/cmd_vel`을 낸다.
    """

    def __init__(self, state: PanelState, lock: threading.Lock):
        super().__init__("waver_remote_panel")
        # 역할: 실차/Gazebo 양쪽에서 토픽과 안전값을 launch 파라미터로 바꿀 수 있게 한다.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("manual_cmd_vel_topic", "/waver/manual_cmd_vel")
        self.declare_parameter("auto_cmd_vel_topic", "/waver/cmd_vel_nav2")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("patrol_state_topic", "/waver/patrol_state")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("radar_state_topic", "/waver/radar_target_state")
        self.declare_parameter("object_goal_state_topic", "/waver/object_mission_goal_state")
        self.declare_parameter("battery_state_text_topic", "/waver/battery_state_text")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("mission_reset_topic", "/waver/mission_reset")
        self.declare_parameter("speed_limit_topic", "/waver/speed_limit")
        self.declare_parameter("angular_speed_limit_topic", "/waver/angular_speed_limit")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("default_speed", 0.18)
        self.declare_parameter("default_angular", 0.45)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 1.2)
        self.declare_parameter("reverse_speed", 0.08)
        self.declare_parameter("enable_scan_assist", True)
        self.declare_parameter("lidar_required", True)
        self.declare_parameter("hard_stop_distance_m", 0.45)
        self.declare_parameter("slow_down_distance_m", 1.2)
        self.declare_parameter("min_valid_scan_points", 40)
        self.declare_parameter("publish_direct_cmd_vel", False)
        self.declare_parameter("manual_override_returns_to_auto", True)
        self.declare_parameter("auto_mode_strategy", "mission_nav2")
        self.declare_parameter("auto_launch_command", "")
        self.declare_parameter("auto_command", "")
        self.declare_parameter("auto_param_file", "")
        self.declare_parameter("auto_waypoint_file", "")
        self.declare_parameter("auto_waypoints_csv", "")
        self.declare_parameter("auto_loop_count", -1)
        self.declare_parameter("auto_require_scan", True)
        self.declare_parameter("auto_min_valid_scan_points", 40)
        self.declare_parameter("auto_cmd_timeout_s", 0.5)
        self.declare_parameter("auto_use_sim_time", False)
        self.declare_parameter("auto_max_patrol_radius_m", 3.0)
        self.declare_parameter("demo_script", "")
        self.declare_parameter("demo_close_on_finish", False)

        self.state = state
        self.lock = lock
        self.auto_process: Optional[subprocess.Popen] = None
        self.last_mode_publish = ""
        self.last_mode_publish_time = 0.0
        self.last_estop_publish = False
        self.last_speed_limit_publish = -1.0
        self.last_angular_limit_publish = -1.0
        self.publish_direct_cmd_vel = bool(self.get_parameter("publish_direct_cmd_vel").value)
        self.final_cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.manual_cmd_vel_topic = str(self.get_parameter("manual_cmd_vel_topic").value)
        self.cmd_output_topic = (
            self.final_cmd_vel_topic if self.publish_direct_cmd_vel else self.manual_cmd_vel_topic
        )
        self.auto_cmd_vel_topic = str(self.get_parameter("auto_cmd_vel_topic").value)
        self.latest_auto_cmd = Twist()
        self.last_auto_cmd_time = 0.0

        # 역할: GUI/manual 최종 속도 후보도 기존 DriveAssist를 통과시킨다.
        self.assist = DriveAssist(
            AssistConfig(
                max_linear_speed=float(self.get_parameter("max_linear_speed").value),
                max_angular_speed=float(self.get_parameter("max_angular_speed").value),
                max_linear_accel=float(self.get_parameter("max_linear_accel").value),
                max_angular_accel=float(self.get_parameter("max_angular_accel").value),
                reverse_speed=float(self.get_parameter("reverse_speed").value),
                lidar_required=bool(self.get_parameter("lidar_required").value),
                hard_stop_distance_m=float(self.get_parameter("hard_stop_distance_m").value),
                slow_down_distance_m=float(self.get_parameter("slow_down_distance_m").value),
                min_valid_scan_points=int(self.get_parameter("min_valid_scan_points").value),
            )
        )

        with self.lock:
            self.state.speed_limit = float(self.get_parameter("default_speed").value)
            self.state.angular_limit = float(self.get_parameter("default_angular").value)

        # 역할: GUI가 내는 수동 후보 명령, 모드, E-Stop, 속도 제한을 ROS graph에 공개한다.
        self.cmd_pub = self.create_publisher(Twist, self.cmd_output_topic, 10)
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("mode_topic").value),
            10,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            10,
        )
        self.mission_reset_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("mission_reset_topic").value),
            10,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("speed_limit_topic").value),
            10,
        )
        self.angular_limit_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("angular_speed_limit_topic").value),
            10,
        )

        # 역할: 상태창 표시용으로 실제 최종 /cmd_vel, Nav2 후보, odom, mission state를 구독한다.
        self.create_subscription(Twist, self.final_cmd_vel_topic, self.cmd_callback, 10)
        self.create_subscription(Twist, self.auto_cmd_vel_topic, self.auto_cmd_callback, 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("patrol_state_topic").value),
            self.patrol_state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            lambda msg: self.set_text_state("mission_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("safety_state_topic").value),
            lambda msg: self.set_text_state("safety_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("radar_state_topic").value),
            lambda msg: self.set_text_state("radar_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("object_goal_state_topic").value),
            lambda msg: self.set_text_state("object_goal_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("battery_state_text_topic").value),
            lambda msg: self.set_text_state("battery_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_class_topic").value),
            lambda msg: self.set_text_state("target_class", msg.data),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("target_confidence_topic").value),
            self.target_confidence_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("bird_confirmed_topic").value),
            self.bird_confirmed_callback,
            10,
        )
        if bool(self.get_parameter("enable_scan_assist").value):
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self.scan_callback,
                qos_profile_sensor_data,
            )

        rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.publish_tick)
        self.publish_speed_limits(force=True)

    def cmd_callback(self, msg: Twist) -> None:
        # 역할: 현재 /cmd_vel 선속도/각속도를 리모콘 상태창에 표시한다.
        with self.lock:
            self.state.latest_cmd_linear = float(msg.linear.x)
            self.state.latest_cmd_angular = float(msg.angular.z)

    def auto_cmd_callback(self, msg: Twist) -> None:
        # 역할: 자율순찰 노드가 낸 후보 명령을 저장하고, 최종 발행은 패널에서만 수행한다.
        now = time.monotonic()
        with self.lock:
            self.latest_auto_cmd = msg
            self.last_auto_cmd_time = now
            self.state.latest_auto_linear = float(msg.linear.x)
            self.state.latest_auto_angular = float(msg.angular.z)
            self.state.latest_auto_age = 0.0

    def odom_callback(self, msg: Odometry) -> None:
        # 역할: 리모콘 상태창에서 현재 Gazebo/실차 odom 위치를 확인할 수 있게 한다.
        with self.lock:
            self.state.odom_x = float(msg.pose.pose.position.x)
            self.state.odom_y = float(msg.pose.pose.position.y)

    def patrol_state_callback(self, msg: String) -> None:
        # 역할: 자율순찰 노드 상태를 GUI에 그대로 보여준다.
        with self.lock:
            self.state.patrol_state = msg.data

    def set_text_state(self, field_name: str, value: str) -> None:
        # 역할: 여러 상태 문자열 토픽을 같은 패턴으로 GUI 공유 상태에 반영한다.
        with self.lock:
            setattr(self.state, field_name, value)

    def target_confidence_callback(self, msg: Float32) -> None:
        # 역할: 딥러닝/카메라 stub가 낸 목표 분류 confidence를 표시한다.
        with self.lock:
            self.state.target_confidence = float(msg.data)

    def bird_confirmed_callback(self, msg: Bool) -> None:
        # 역할: 조류 확인 여부를 리모콘에서 즉시 볼 수 있게 한다.
        with self.lock:
            self.state.bird_confirmed = bool(msg.data)

    def scan_callback(self, msg: LaserScan) -> None:
        # 역할: manual GUI 조작에도 LiDAR hard stop/slow down을 적용한다.
        self.assist.update_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

    def set_mode(self, mode: str) -> None:
        # 역할: GUI 버튼에서 요청한 모드를 상태와 ROS topic 양쪽에 반영한다.
        with self.lock:
            self.state.mode = mode
            if mode != "MANUAL":
                self.state.desired_linear = 0.0
                self.state.desired_angular = 0.0
                self.state.active_control = "none"
        self.publish_mode(force=True)

    def set_manual_command(self, linear: float, angular: float, label: str) -> None:
        # 역할: 방향 버튼/방향키 입력을 현재 speed slider 값으로 스케일링한다.
        # AUTO 중에는 mode를 AUTO로 유지해 Nav2 goal/path를 살리고, safety mux의 manual override만 사용한다.
        with self.lock:
            keep_auto = (
                self.state.mode == "AUTO"
                and bool(self.get_parameter("manual_override_returns_to_auto").value)
            )
            if not keep_auto:
                self.state.mode = "MANUAL"
            self.state.desired_linear = linear * self.state.speed_limit
            self.state.desired_angular = angular * self.state.angular_limit
            self.state.active_control = label
            self.state.emergency_stop = False
            if keep_auto and (abs(linear) > 1e-5 or abs(angular) > 1e-5):
                self.state.auto_status = "manual override active; AUTO goal retained"

    def stop_motion(self, stop_auto: bool = True) -> None:
        # 역할: 일반 정지 버튼. 자동순찰도 함께 멈추고 0속도를 반복 발행한다.
        if stop_auto:
            self.stop_auto()
        with self.lock:
            keep_auto = (
                not stop_auto
                and self.state.mode == "AUTO"
                and bool(self.get_parameter("manual_override_returns_to_auto").value)
            )
            if not keep_auto:
                self.state.mode = "STANDBY"
            self.state.desired_linear = 0.0
            self.state.desired_angular = 0.0
            self.state.active_control = "stop"
        self.publish_stop_burst()

    def emergency_stop(self) -> None:
        # 역할: 긴급정지. 자동순찰 프로세스를 죽이고 수동 reset 전까지 계속 0속도를 발행한다.
        self.stop_auto()
        with self.lock:
            self.state.mode = "EMERGENCY"
            self.state.emergency_stop = True
            self.state.desired_linear = 0.0
            self.state.desired_angular = 0.0
            self.state.active_control = "E-STOP"
        self.publish_stop_burst()

    def reset_estop(self) -> None:
        # 역할: 긴급정지 latch를 풀되, 바로 움직이지 않고 STANDBY 정지 상태로 둔다.
        with self.lock:
            self.state.mode = "STANDBY"
            self.state.emergency_stop = False
            self.state.active_control = "reset"
        self.publish_stop_burst()

    def start_auto(self) -> None:
        # 역할: AUTO 버튼을 mission/Nav2 백엔드와 연결한다.
        # 기본 실차 모드에서는 subprocess를 띄우지 않고 `/waver/mode=AUTO`만 발행한다.
        if self.auto_process is not None and self.auto_process.poll() is None:
            self.set_mode("AUTO")
            return
        strategy = str(self.get_parameter("auto_mode_strategy").value).strip().lower()
        launch_command = str(self.get_parameter("auto_launch_command").value).strip()
        legacy_command = str(self.get_parameter("auto_command").value).strip()
        command_text = launch_command or (legacy_command if strategy == "legacy_subprocess" else "")
        if strategy != "legacy_subprocess" and not command_text:
            with self.lock:
                self.state.emergency_stop = False
                self.state.desired_linear = 0.0
                self.state.desired_angular = 0.0
                self.state.active_control = "none"
                self.state.auto_status = "AUTO requested: mission/Nav2 backend"
            self.publish_mission_reset_pulse()
            self.set_mode("AUTO")
            return

        if not command_text:
            self.get_logger().warn("AUTO legacy subprocess requested but auto_command is empty")
            with self.lock:
                self.state.auto_status = "AUTO backend missing command"
            self.set_mode("AUTO")
            return

        command = shlex.split(command_text)
        require_scan = str(bool(self.get_parameter("auto_require_scan").value)).lower()
        auto_args = ["--ros-args"]
        auto_param_file = str(self.get_parameter("auto_param_file").value).strip()
        if auto_param_file:
            auto_args.extend(["--params-file", auto_param_file])
        auto_args.extend(
            [
                "-p",
                f"cmd_vel_topic:={self.auto_cmd_vel_topic}",
                "-p",
                f"lidar_required:={require_scan}",
                "-p",
                f"loop_count:={int(self.get_parameter('auto_loop_count').value)}",
                "-p",
                "stuck_timeout_s:=8.0",
                "-p",
                "progress_epsilon_m:=0.01",
                "-p",
                (
                    "min_valid_scan_points:="
                    f"{int(self.get_parameter('auto_min_valid_scan_points').value)}"
                ),
                "-p",
                f"use_sim_time:={str(bool(self.get_parameter('auto_use_sim_time').value)).lower()}",
                "-p",
                (
                    "max_patrol_radius_m:="
                    f"{float(self.get_parameter('auto_max_patrol_radius_m').value)}"
                ),
            ]
        )
        waypoint_file = str(self.get_parameter("auto_waypoint_file").value).strip()
        if waypoint_file:
            auto_args.extend(["-p", f"waypoint_file:={waypoint_file}"])
        waypoints = str(self.get_parameter("auto_waypoints_csv").value)
        if waypoints:
            auto_args.extend(["-p", f"waypoints_csv:={waypoints}"])
        env = os.environ.copy()
        self.auto_process = subprocess.Popen(command + auto_args, env=env)
        with self.lock:
            self.state.auto_status = f"legacy subprocess pid {self.auto_process.pid}"
        self.set_mode("AUTO")

    def stop_auto(self) -> None:
        # 역할: 패널이 직접 띄운 legacy AUTO 프로세스만 종료한다. mission/Nav2 백엔드는 launch가 관리한다.
        proc = self.auto_process
        if proc is None:
            with self.lock:
                self.state.auto_status = "backend controlled by launch"
            return
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.auto_process = None
        with self.lock:
            self.state.auto_status = "stopped"

    def publish_tick(self) -> None:
        # 역할: 20Hz로 현재 모드에 맞는 수동 후보/legacy direct 명령을 발행한다.
        now = time.monotonic()
        with self.lock:
            mode = self.state.mode
            estop = self.state.emergency_stop
            linear = self.state.desired_linear
            angular = self.state.desired_angular
            auto_linear = float(self.latest_auto_cmd.linear.x)
            auto_angular = float(self.latest_auto_cmd.angular.z)
            auto_age = now - self.last_auto_cmd_time if self.last_auto_cmd_time else 999.0
            self.state.latest_auto_age = auto_age

        self.publish_mode()
        self.publish_estop()
        self.publish_speed_limits()
        if estop or mode in {"STANDBY", "EMERGENCY"}:
            self.cmd_pub.publish(Twist())
            return
        if mode == "MANUAL":
            command = self.assist.assisted_command(
                DriveCommand(linear=linear, angular=angular, source="remote_panel")
            )
            msg = Twist()
            msg.linear.x = command.linear
            msg.angular.z = command.angular
            self.cmd_pub.publish(msg)
        elif mode in {"AUTO", "PATROL", "TRACK_ONLY", "RETURN_HOME"}:
            if not self.publish_direct_cmd_vel:
                # 역할: AUTO 중에는 Nav2가 `/waver/cmd_vel_nav2`를 계속 만들고,
                # 리모콘은 사람이 누르는 동안만 `/waver/manual_cmd_vel` 후보를 올린다.
                if abs(linear) > 1e-5 or abs(angular) > 1e-5:
                    command = self.assist.assisted_command(
                        DriveCommand(linear=linear, angular=angular, source="remote_panel_override")
                    )
                    msg = Twist()
                    msg.linear.x = command.linear
                    msg.angular.z = command.angular
                    self.cmd_pub.publish(msg)
                else:
                    self.cmd_pub.publish(Twist())
                return

            timeout_s = float(self.get_parameter("auto_cmd_timeout_s").value)
            if auto_age > timeout_s:
                with self.lock:
                    self.state.auto_status = f"auto command stale {auto_age:.1f}s"
                self.cmd_pub.publish(Twist())
                return
            command = self.assist.assisted_command(
                DriveCommand(linear=auto_linear, angular=auto_angular, source="remote_panel_auto")
            )
            msg = Twist()
            msg.linear.x = command.linear
            msg.angular.z = command.angular
            self.cmd_pub.publish(msg)

    def publish_mode(self, force: bool = False) -> None:
        # 역할: 모드 문자열을 /waver/mode로 발행해 다른 노드가 GUI 상태를 알 수 있게 한다.
        # change-only 발행은 late subscriber가 AUTO를 놓칠 수 있으므로 0.5초마다 heartbeat로 재발행한다.
        with self.lock:
            mode = self.state.mode
        now = time.monotonic()
        if force or mode != self.last_mode_publish or now - self.last_mode_publish_time > 0.5:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
            self.last_mode_publish = mode
            self.last_mode_publish_time = now

    def publish_estop(self) -> None:
        # 역할: 긴급정지 상태를 /waver/emergency_stop으로 발행한다.
        with self.lock:
            estop = self.state.emergency_stop
        if estop != self.last_estop_publish:
            msg = Bool()
            msg.data = estop
            self.estop_pub.publish(msg)
            self.last_estop_publish = estop

    def publish_speed_limits(self, force: bool = False) -> None:
        # 역할: 리모콘 slider 값을 safety_cmd_mux_node의 선속도/각속도 제한 토픽에 반영한다.
        with self.lock:
            speed_limit = float(self.state.speed_limit)
            angular_limit = float(self.state.angular_limit)
        if force or abs(speed_limit - self.last_speed_limit_publish) > 1e-6:
            self.speed_limit_pub.publish(Float32(data=speed_limit))
            self.last_speed_limit_publish = speed_limit
        if force or abs(angular_limit - self.last_angular_limit_publish) > 1e-6:
            self.angular_limit_pub.publish(Float32(data=angular_limit))
            self.last_angular_limit_publish = angular_limit

    def publish_mission_reset_pulse(self) -> None:
        # 역할: HOLD_AT_HOME 등 latch 상태가 있을 때 AUTO 재개 요청을 백엔드에 알려주는 안전한 reset pulse다.
        self.mission_reset_pub.publish(Bool(data=True))
        self.mission_reset_pub.publish(Bool(data=False))

    def publish_stop_burst(self) -> None:
        # 역할: 버튼 클릭 직후에도 지연 없이 정지하도록 zero manual 후보를 여러 번 발행한다.
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def destroy_node(self) -> bool:
        # 역할: GUI 창이 닫혀도 자동순찰 중지와 stop burst를 보장한다.
        self.stop_auto()
        self.publish_stop_burst()
        return super().destroy_node()


class WaverRemotePanel:
    """Tkinter visual remote for Waver.

    역할:
      - 리모콘 버튼, 방향키, 속도 슬라이더, 모드/속도 상태를 시각화한다.
      - 모든 실제 ROS 발행은 WaverRemoteNode에 위임한다.
    """

    def __init__(self, node: WaverRemoteNode, state: PanelState, lock: threading.Lock):
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exc:
            raise RuntimeError("python3-tk is required: sudo apt install python3-tk") from exc

        self.tk = tk
        self.ttk = ttk
        self.node = node
        self.state = state
        self.lock = lock
        self.active_key: Optional[str] = None
        self.closed = False
        self.direction_buttons = {}
        self.status_cards = {}

        # 역할: 리모콘 창의 전체 레이아웃을 만든다.
        self.root = tk.Tk()
        self.root.title("Waver Remote Panel")
        self.root.geometry("1040x900")
        self.root.minsize(900, 780)
        self.root.configure(bg="#0b1117")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        self.mode_var = tk.StringVar(value="STANDBY")
        self.cmd_var = tk.StringVar(value="cmd: 0.00 m/s, 0.00 rad/s")
        self.odom_var = tk.StringVar(value="odom: x=0.00, y=0.00")
        self.patrol_var = tk.StringVar(value="patrol: not started")
        self.mission_var = tk.StringVar(value="mission: unknown")
        self.safety_var = tk.StringVar(value="safety: unknown")
        self.radar_var = tk.StringVar(value="radar: unknown")
        self.object_goal_var = tk.StringVar(value="object goal: unknown")
        self.battery_var = tk.StringVar(value="battery: unknown")
        self.target_var = tk.StringVar(value="target: unknown")
        self.auto_var = tk.StringVar(value="auto: stopped")
        self.hazard_var = tk.StringVar(value="scan: unknown")
        self.auto_cmd_var = tk.StringVar(value="nav2 cmd: 0.00 m/s, 0.00 rad/s")
        self.mode_hint_var = tk.StringVar(value="STANDBY: stopped")
        self.speed_text_var = tk.StringVar(value="speed: 0.12 m/s, turn: 0.45 rad/s")
        self.source_badge_var = tk.StringVar(value="CONTROL: STANDBY")
        self.estop_badge_var = tk.StringVar(value="E-STOP: CLEAR")
        self.scan_badge_var = tk.StringVar(value="SCAN: WAITING")
        self.odom_badge_var = tk.StringVar(value="ODOM: 0.00, 0.00")
        self.speed_var = tk.DoubleVar(value=self.state.speed_limit)
        self.angular_var = tk.DoubleVar(value=self.state.angular_limit)

        self.build_header()
        self.build_mode_buttons()
        self.build_body()
        self.refresh()
        self.start_demo_script()

    def build_header(self) -> None:
        # 역할: 현재 모드를 큰 색상 라벨로 보여준다.
        label = self.tk.Label(
            self.root,
            textvariable=self.mode_var,
            font=("Sans", 32, "bold"),
            fg="white",
            bg="#3949ab",
            height=2,
        )
        label.pack(fill="x", padx=12, pady=(12, 8))
        self.mode_label = label
        self.mode_hint_label = self.tk.Label(
            self.root,
            textvariable=self.mode_hint_var,
            font=("Sans", 13, "bold"),
            fg="#fffde7",
            bg="#0b1117",
            anchor="w",
        )
        self.mode_hint_label.pack(fill="x", padx=18, pady=(0, 4))
        badges = self.tk.Frame(self.root, bg="#0b1117")
        badges.pack(fill="x", padx=12, pady=(0, 6))
        self.source_badge = self.make_badge(badges, self.source_badge_var, "#455a64")
        self.estop_badge = self.make_badge(badges, self.estop_badge_var, "#1b5e20")
        self.scan_badge = self.make_badge(badges, self.scan_badge_var, "#546e7a")
        self.odom_badge = self.make_badge(badges, self.odom_badge_var, "#263238")
        for index, badge in enumerate(
            [self.source_badge, self.estop_badge, self.scan_badge, self.odom_badge]
        ):
            badge.grid(row=0, column=index, sticky="nsew", padx=4)
            badges.grid_columnconfigure(index, weight=1)

    def make_badge(self, parent, variable, color):
        # 역할: 한눈에 읽히는 상태 배지를 만들어 색상과 텍스트를 함께 제공한다.
        return self.tk.Label(
            parent,
            textvariable=variable,
            fg="white",
            bg=color,
            font=("Sans", 12, "bold"),
            padx=10,
            pady=8,
        )

    def build_mode_buttons(self) -> None:
        # 역할: MANUAL/AUTO/STOP/E-STOP 같은 상위 모드 전환 버튼을 만든다.
        frame = self.tk.Frame(self.root, bg="#0b1117")
        frame.pack(fill="x", padx=12, pady=4)
        buttons = [
            ("MANUAL", self.node.set_mode, "MANUAL", "#1565c0"),
            ("AUTO START", self.node.start_auto, None, "#2e7d32"),
            ("AUTO STOP", self.node.stop_motion, True, "#546e7a"),
            ("STOP", self.node.stop_motion, True, "#f9a825"),
            ("E-STOP", self.node.emergency_stop, None, "#b71c1c"),
            ("RESET", self.node.reset_estop, None, "#6a1b9a"),
        ]
        for index, (text, callback, argument, color) in enumerate(buttons):
            button = self.tk.Button(
                frame,
                text=text,
                fg="white",
                bg=color,
                activebackground=color,
                font=("Sans", 11, "bold"),
                height=3,
            )
            if argument is None:
                button.configure(command=callback)
            else:
                button.configure(command=lambda cb=callback, arg=argument: cb(arg))
            button.grid(row=index // 3, column=index % 3, sticky="nsew", padx=4, pady=4)
        for column in range(3):
            frame.grid_columnconfigure(column, weight=1)

    def build_body(self) -> None:
        # 역할: 좌측 조작 영역과 우측 상태 영역을 분리해 현장 가독성을 높인다.
        body = self.tk.Frame(self.root, bg="#0b1117")
        body.pack(fill="both", expand=True, padx=12, pady=8)
        self.left_panel = self.tk.Frame(body, bg="#0b1117")
        self.right_panel = self.tk.Frame(body, bg="#0b1117")
        self.left_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.right_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        body.grid_columnconfigure(0, weight=3)
        body.grid_columnconfigure(1, weight=2)
        body.grid_rowconfigure(0, weight=1)
        self.build_direction_pad()
        self.build_speed_controls()
        self.build_status()

    def build_direction_pad(self) -> None:
        # 역할: 방향 버튼을 누르는 동안만 해당 방향으로 /cmd_vel 후보를 발행한다.
        frame = self.tk.LabelFrame(
            self.left_panel,
            text="Manual Direction / Rudder",
            bg="#0b1117",
            fg="white",
            font=("Sans", 12, "bold"),
        )
        frame.pack(fill="x", pady=(0, 12))
        self.rudder_canvas = self.tk.Canvas(
            frame,
            width=220,
            height=150,
            bg="#101820",
            highlightthickness=1,
            highlightbackground="#37474f",
        )
        self.rudder_canvas.grid(row=0, column=0, columnspan=3, sticky="ew", padx=8, pady=8)
        layout = [
            [
                ("↖", 1.0, 1.0, "forward-left"),
                ("↑", 1.0, 0.0, "forward"),
                ("↗", 1.0, -1.0, "forward-right"),
            ],
            [
                ("←", 0.0, 1.0, "left"),
                ("●", 0.0, 0.0, "stop"),
                ("→", 0.0, -1.0, "right"),
            ],
            [
                ("↙", -1.0, 1.0, "back-left"),
                ("↓", -1.0, 0.0, "back"),
                ("↘", -1.0, -1.0, "back-right"),
            ],
        ]
        for row_index, row in enumerate(layout):
            for col_index, (text, linear, angular, label) in enumerate(row):
                button = self.tk.Button(
                    frame,
                    text=text,
                    width=8,
                    height=3,
                    font=("Sans", 18, "bold"),
                    bg="#263238",
                    fg="white",
                    activebackground="#455a64",
                )
                button.grid(row=row_index + 1, column=col_index, padx=5, pady=5)
                self.direction_buttons[label] = button
                button.bind(
                    "<ButtonPress-1>",
                    lambda _e, l=linear, a=angular, name=label: self.press_direction(l, a, name),
                )
                button.bind("<ButtonRelease-1>", lambda _e: self.release_direction())
        for column in range(3):
            frame.grid_columnconfigure(column, weight=1)

    def build_speed_controls(self) -> None:
        # 역할: 실증 전 단계에서 수동 속도와 회전 속도를 낮게 조정한다.
        frame = self.tk.LabelFrame(
            self.left_panel,
            text="Speed Limit",
            bg="#0b1117",
            fg="white",
            font=("Sans", 12, "bold"),
        )
        frame.pack(fill="x")
        self.tk.Label(
            frame,
            textvariable=self.speed_text_var,
            bg="#0b1117",
            fg="#fff59d",
            font=("Sans", 12, "bold"),
            anchor="w",
        ).pack(fill="x")
        self.tk.Label(frame, text="linear m/s", bg="#0b1117", fg="white").pack(anchor="w")
        self.tk.Scale(
            frame,
            from_=0.03,
            to=0.30,
            resolution=0.01,
            orient="horizontal",
            variable=self.speed_var,
            command=self.on_speed_change,
            bg="#0b1117",
            fg="white",
            highlightthickness=0,
        ).pack(fill="x")
        self.tk.Label(frame, text="angular rad/s", bg="#0b1117", fg="white").pack(anchor="w")
        self.tk.Scale(
            frame,
            from_=0.10,
            to=0.80,
            resolution=0.05,
            orient="horizontal",
            variable=self.angular_var,
            command=self.on_speed_change,
            bg="#0b1117",
            fg="white",
            highlightthickness=0,
        ).pack(fill="x")

    def build_status(self) -> None:
        # 역할: 현재 속도, odom, mission/Nav2/safety 상태를 한눈에 보여준다.
        frame = self.tk.Frame(self.right_panel, bg="#0b1117")
        frame.pack(fill="both", expand=True)
        self.hazard_label = None
        cards = [
            ("FINAL CMD", self.cmd_var),
            ("MISSION", self.mission_var),
            ("SAFETY MUX", self.safety_var),
            ("ODOM", self.odom_var),
            ("PATROL", self.patrol_var),
            ("RADAR", self.radar_var),
            ("OBJECT GOAL", self.object_goal_var),
            ("TARGET", self.target_var),
            ("BATTERY", self.battery_var),
            ("AUTO BACKEND", self.auto_var),
            ("NAV2 CANDIDATE", self.auto_cmd_var),
            ("SCAN SAFETY", self.hazard_var),
        ]
        for title, variable in cards:
            label = self.make_status_card(frame, title, variable)
            if variable is self.hazard_var:
                self.hazard_label = label
        help_text = (
            "Keyboard: Arrow/WASD override, Space/K stop, E emergency, R reset, "
            "P auto. AUTO 중 override 후 키를 떼면 Nav2 경로로 복귀."
        )
        self.tk.Label(
            frame,
            text=help_text,
            anchor="w",
            fg="#b0bec5",
            bg="#0b1117",
            font=("Sans", 9),
        ).pack(fill="x", pady=(8, 0))

    def make_status_card(self, parent, title: str, variable):
        # 역할: 상태 이름과 값을 분리한 카드로 정보 스캔 속도를 높인다.
        card = self.tk.Frame(parent, bg="#17252f", padx=10, pady=7)
        card.pack(fill="x", pady=4)
        self.tk.Label(
            card,
            text=title,
            anchor="w",
            fg="#90caf9",
            bg="#17252f",
            font=("Sans", 9, "bold"),
        ).pack(fill="x")
        value = self.tk.Label(
            card,
            textvariable=variable,
            anchor="w",
            fg="#e0f2f1",
            bg="#17252f",
            font=("Sans", 12, "bold"),
            wraplength=360,
            justify="left",
        )
        value.pack(fill="x")
        self.status_cards[title] = (card, value)
        return card

    def press_direction(self, linear: float, angular: float, label: str) -> None:
        # 역할: 방향 버튼 press 이벤트를 수동 명령으로 바꾼다.
        self.highlight_direction(label)
        if label == "stop":
            self.node.stop_motion(stop_auto=False)
            return
        self.node.set_manual_command(linear, angular, label)

    def release_direction(self) -> None:
        # 역할: 버튼/키를 떼면 수동 명령을 즉시 0으로 만든다.
        self.highlight_direction("")
        self.node.set_manual_command(0.0, 0.0, "released")

    def on_speed_change(self, _value: str) -> None:
        # 역할: slider 값을 ROS 스레드가 사용할 공유 상태로 반영한다.
        with self.lock:
            self.state.speed_limit = float(self.speed_var.get())
            self.state.angular_limit = float(self.angular_var.get())
        self.node.publish_speed_limits(force=True)
        self.speed_text_var.set(
            f"speed: {self.speed_var.get():.2f} m/s, turn: {self.angular_var.get():.2f} rad/s"
        )

    def highlight_direction(self, active_label: str) -> None:
        # 역할: 현재 누른 방향 버튼을 밝게 표시해 조작자가 방향타 상태를 즉시 알 수 있게 한다.
        for label, button in self.direction_buttons.items():
            if label == active_label:
                button.configure(bg="#00acc1", activebackground="#26c6da")
            elif label == "stop":
                button.configure(bg="#b71c1c", activebackground="#d32f2f")
            else:
                button.configure(bg="#263238", activebackground="#455a64")

    def update_rudder(self, linear: float, angular: float, active: str) -> None:
        # 역할: 현재 최종 명령 방향을 작은 벡터 표시로 보여줘 버튼/실제 cmd 차이를 빠르게 확인한다.
        canvas = getattr(self, "rudder_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        width = int(canvas["width"])
        height = int(canvas["height"])
        cx = width // 2
        cy = height // 2
        canvas.create_oval(cx - 46, cy - 46, cx + 46, cy + 46, outline="#455a64", width=2)
        canvas.create_line(cx, 12, cx, height - 12, fill="#263238", width=2)
        canvas.create_line(12, cy, width - 12, cy, fill="#263238", width=2)
        dx = max(-1.0, min(1.0, angular / max(float(self.angular_var.get()), 0.1))) * 60.0
        dy = -max(-1.0, min(1.0, linear / max(float(self.speed_var.get()), 0.03))) * 48.0
        end_x = cx + dx
        end_y = cy + dy
        color = "#00acc1" if abs(linear) > 0.01 or abs(angular) > 0.01 else "#78909c"
        canvas.create_line(cx, cy, end_x, end_y, fill=color, width=5, arrow="last")
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#eceff1", outline="")
        canvas.create_text(
            cx,
            height - 14,
            text=f"rudder: {active}",
            fill="#e0f7fa",
            font=("Sans", 10, "bold"),
        )

    def on_key_press(self, event) -> None:
        # 역할: 리모콘 창에 포커스가 있을 때 방향키/WASD로도 조작한다.
        key = event.keysym.lower()
        if key == self.active_key:
            return
        mapping = {
            "up": (1.0, 0.0, "key-up"),
            "w": (1.0, 0.0, "key-w"),
            "down": (-1.0, 0.0, "key-down"),
            "s": (-1.0, 0.0, "key-s"),
            "left": (0.0, 1.0, "key-left"),
            "a": (0.0, 1.0, "key-a"),
            "right": (0.0, -1.0, "key-right"),
            "d": (0.0, -1.0, "key-d"),
        }
        if key in mapping:
            self.active_key = key
            self.press_direction(*mapping[key])
        elif key in {"space", "k"}:
            self.node.stop_motion(stop_auto=True)
        elif key == "e":
            self.node.emergency_stop()
        elif key == "r":
            self.node.reset_estop()
        elif key == "p":
            self.node.start_auto()

    def on_key_release(self, event) -> None:
        # 역할: 눌렀던 방향키를 떼면 정지한다.
        key = event.keysym.lower()
        if key == self.active_key:
            self.active_key = None
            self.release_direction()

    def start_demo_script(self) -> None:
        # 역할: Gazebo 검증 때 실제 버튼 함수 경로를 자동 호출해 리모콘 클릭 동작을 재현한다.
        script = str(self.node.get_parameter("demo_script").value).strip()
        if not script:
            return
        if script == "manual_smoke":
            steps = [
                (800, lambda: self.press_direction(1.0, 0.0, "forward")),
                (2600, self.release_direction),
                (3200, lambda: self.node.stop_motion(stop_auto=True)),
                (4200, self.close_if_demo_requested),
            ]
        elif script == "manual_auto_smoke":
            steps = [
                (800, lambda: self.press_direction(1.0, 0.0, "forward")),
                (2400, self.release_direction),
                (3000, lambda: self.node.stop_motion(stop_auto=True)),
                (3800, self.node.start_auto),
                (9000, lambda: self.node.stop_motion(stop_auto=True)),
                (10500, self.close_if_demo_requested),
            ]
        elif script == "manual_auto_hold":
            steps = [
                (800, lambda: self.press_direction(1.0, 0.0, "forward")),
                (2400, self.release_direction),
                (3000, lambda: self.node.stop_motion(stop_auto=True)),
                (3800, self.node.start_auto),
                (20000, self.close_if_demo_requested),
            ]
        elif script == "auto_override_smoke":
            # 역할: AUTO를 먼저 켠 뒤 사람이 방향 버튼을 누르는 상황을 재현한다.
            # 기대 동작은 mode=AUTO 유지, `/waver/manual_cmd_vel` 일시 발행,
            # 버튼 release 후 safety mux가 다시 Nav2 후보 명령으로 복귀하는 것이다.
            steps = [
                (900, self.node.start_auto),
                (3800, lambda: self.press_direction(1.0, 0.0, "auto-forward-override")),
                (6800, self.release_direction),
                (8200, lambda: self.press_direction(0.0, -1.0, "auto-right-override")),
                (10800, self.release_direction),
                (15000, self.close_if_demo_requested),
            ]
        else:
            self.node.get_logger().warn(f"Unknown demo_script={script!r}; ignoring")
            return
        self.node.get_logger().info(f"Running Waver remote panel demo_script={script}")
        for delay_ms, callback in steps:
            self.root.after(delay_ms, callback)

    def close_if_demo_requested(self) -> None:
        # 역할: 자동 GUI 검증이 끝난 뒤 테스트 프로세스를 깔끔하게 종료할지 결정한다.
        if bool(self.node.get_parameter("demo_close_on_finish").value):
            self.close()

    def refresh(self) -> None:
        # 역할: ROS 상태를 10Hz로 GUI 라벨에 반영한다.
        if self.closed:
            return
        with self.lock:
            mode = self.state.mode
            estop = self.state.emergency_stop
            cmd_linear = self.state.latest_cmd_linear
            cmd_angular = self.state.latest_cmd_angular
            odom_x = self.state.odom_x
            odom_y = self.state.odom_y
            patrol = self.state.patrol_state
            mission = self.state.mission_state
            safety = self.state.safety_state
            radar = self.state.radar_state
            object_goal = self.state.object_goal_state
            battery = self.state.battery_state
            target_class = self.state.target_class
            target_confidence = self.state.target_confidence
            bird_confirmed = self.state.bird_confirmed
            auto = self.state.auto_status
            auto_linear = self.state.latest_auto_linear
            auto_angular = self.state.latest_auto_angular
            auto_age = self.state.latest_auto_age
            active = self.state.active_control
        self.mode_var.set(f"{mode} {'E-STOP' if estop else ''}".strip())
        hint = {
            "MANUAL": "MANUAL: 패널 수동 후보만 통과, mission/Nav2는 일시 중지 또는 취소",
            "AUTO": "AUTO: Nav2 mission 주행, 방향키 입력은 잠깐 override 후 경로 복귀",
            "STANDBY": "STANDBY: 정지. AUTO 버튼을 누르면 mission/Nav2 백엔드에 AUTO 요청",
            "EMERGENCY": "EMERGENCY: latch stop. RESET 전까지 움직이지 않음",
        }.get(mode, f"{mode}: custom mode")
        self.mode_hint_var.set(hint)
        self.speed_text_var.set(
            f"speed: {self.speed_var.get():.2f} m/s, turn: {self.angular_var.get():.2f} rad/s"
        )
        self.cmd_var.set(
            f"cmd: {cmd_linear:+.2f} m/s, {cmd_angular:+.2f} rad/s, active={active}"
        )
        self.odom_var.set(f"odom: x={odom_x:+.2f}, y={odom_y:+.2f}")
        self.source_badge_var.set(f"CONTROL: {mode}")
        self.estop_badge_var.set("E-STOP: ACTIVE" if estop else "E-STOP: CLEAR")
        self.odom_badge_var.set(f"ODOM: {odom_x:+.2f}, {odom_y:+.2f}")
        self.patrol_var.set(f"patrol: {patrol}")
        self.mission_var.set(f"mission: {mission}")
        self.safety_var.set(f"safety: {safety}")
        self.radar_var.set(f"radar: {radar}")
        self.object_goal_var.set(f"object goal: {object_goal}")
        self.battery_var.set(f"battery: {battery}")
        self.target_var.set(
            f"class={target_class}, conf={target_confidence:.2f}, bird={bird_confirmed}"
        )
        self.auto_var.set(f"auto: {auto}")
        self.auto_cmd_var.set(
            f"nav2 candidate: {auto_linear:+.2f} m/s, {auto_angular:+.2f} rad/s, "
            f"age={auto_age:.1f}s"
        )
        hazard = self.effective_scan_hazard()
        self.hazard_var.set(
            f"scan: {hazard}, front={self.node.assist.scan.front:.2f}m, "
            f"age={self.effective_scan_age():.1f}s"
        )
        color = {
            "MANUAL": "#1565c0",
            "AUTO": "#2e7d32",
            "STANDBY": "#546e7a",
            "EMERGENCY": "#b71c1c",
        }.get(mode, "#3949ab")
        hazard_color = "#1b5e20"
        if "stop" in hazard or "stale" in hazard:
            hazard_color = "#b71c1c"
        elif "slow" in hazard or "degraded" in hazard:
            hazard_color = "#e65100"
        elif "caution" in hazard:
            hazard_color = "#f9a825"
        self.scan_badge_var.set(f"SCAN: {hazard.upper()}")
        source_color = color
        estop_color = "#b71c1c" if estop else "#1b5e20"
        try:
            self.mode_label.configure(bg=color)
            self.source_badge.configure(bg=source_color)
            self.estop_badge.configure(bg=estop_color)
            self.scan_badge.configure(bg=hazard_color)
            if self.hazard_label is not None:
                self.hazard_label.configure(bg=hazard_color)
                for child in self.hazard_label.winfo_children():
                    child.configure(bg=hazard_color, fg="white")
            self.update_rudder(cmd_linear, cmd_angular, active)
            self.root.after(100, self.refresh)
        except self.tk.TclError:
            self.closed = True

    def effective_scan_age(self) -> float:
        # 역할: /scan이 아직 한 번도 안 들어온 상태를 GUI에서 명확히 보이게 한다.
        if self.node.assist.last_scan_time <= 0.0:
            return float("inf")
        return max(0.0, time.monotonic() - self.node.assist.last_scan_time)

    def effective_scan_hazard(self) -> str:
        # 역할: 실차에서 LiDAR 필수인데 scan이 없거나 오래되면 clear로 보이지 않게 보정한다.
        age = self.effective_scan_age()
        if (
            self.node.assist.config.lidar_required
            and age > self.node.assist.config.scan_stale_s
        ):
            return "sensor_stale"
        return self.node.assist.scan.hazard.value

    def run(self) -> None:
        # 역할: Tkinter 이벤트 루프를 시작한다.
        self.root.mainloop()

    def close(self) -> None:
        # 역할: 창 닫기에서도 stop을 먼저 보내고 GUI를 종료한다.
        if self.closed:
            return
        self.closed = True
        self.node.stop_motion(stop_auto=True)
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


def main(args=None):
    # 역할: ROS는 background thread에서 spin하고, Tkinter는 main thread에서 실행한다.
    rclpy.init(args=args)
    state = PanelState()
    lock = threading.Lock()
    node = WaverRemoteNode(state, lock)

    def spin_node() -> None:
        # 역할: GUI 종료나 SIGTERM 중 rclpy shutdown 예외가 사용자 터미널을 더럽히지 않게 한다.
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin_node, daemon=True)
    executor_thread.start()
    panel_holder = {}

    def handle_signal(_signum, _frame) -> None:
        # 역할: timeout/systemd/launch 종료에서도 GUI close 경로로 들어가 stop을 보장한다.
        panel = panel_holder.get("panel")
        if panel is not None:
            panel.close()
        else:
            node.stop_motion(stop_auto=True)

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    try:
        panel = WaverRemotePanel(node, state, lock)
        panel_holder["panel"] = panel
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

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
import math
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
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
    pose_source: str = "odom"
    robot_trace: list[tuple[float, float]] = field(default_factory=list)
    patrol_state: str = "not started"
    mission_state: str = "unknown"
    safety_state: str = "unknown"
    radar_state: str = "unknown"
    object_goal_state: str = "unknown"
    battery_state: str = "unknown"
    target_class: str = "unknown"
    target_confidence: float = 0.0
    bird_confirmed: bool = False
    camera_state: str = "unknown"
    sound_state: str = "unknown"
    gazebo_trial_state: str = "unknown"
    map_apply_state: str = "unknown"
    height_filter_debug: str = "unknown"
    auto_status: str = "stopped"
    odom_yaw: float = 0.0
    map_display_mode: str = "AUTO_MAP"
    map_received: bool = False
    map_frame: str = "map"
    map_width: int = 0
    map_height: int = 0
    map_resolution: float = 0.0
    map_origin_x: float = 0.0
    map_origin_y: float = 0.0
    map_occupied: list[tuple[float, float]] = field(default_factory=list)
    map_sequence: int = 0
    global_path: list[tuple[float, float]] = field(default_factory=list)
    local_path: list[tuple[float, float]] = field(default_factory=list)
    global_path_frame: str = ""
    local_path_frame: str = ""
    active_goal: Optional[tuple[float, float]] = None
    active_goal_frame: str = ""
    object_goal: Optional[tuple[float, float]] = None
    object_goal_frame: str = ""
    lidar_objects: list[tuple[float, float]] = field(default_factory=list)
    lidar_objects_frame: str = ""
    elevated_targets: list[tuple[float, float]] = field(default_factory=list)
    elevated_targets_frame: str = ""
    current_waypoint: Optional[tuple[float, float]] = None
    current_waypoint_frame: str = ""


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
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_display_mode", "auto")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("local_path_topic", "/local_plan")
        self.declare_parameter("active_nav_goal_topic", "/waver/active_nav_goal")
        self.declare_parameter("object_mission_goal_topic", "/waver/object_mission_goal")
        self.declare_parameter("lidar_objects_map_topic", "/waver/lidar_objects_map")
        self.declare_parameter("elevated_dynamic_target_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("current_waypoint_topic", "/waver/current_waypoint")
        self.declare_parameter("patrol_state_topic", "/waver/patrol_state")
        self.declare_parameter("patrol_status_topic", "/waver/patrol_status")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("radar_state_topic", "/waver/radar_target_state")
        self.declare_parameter("object_goal_state_topic", "/waver/object_mission_goal_state")
        self.declare_parameter("height_filter_debug_topic", "/waver/height_filter_debug")
        self.declare_parameter("camera_detection_status_topic", "/waver/classification_state")
        self.declare_parameter("sound_mission_status_topic", "/waver/sound_mission_status")
        self.declare_parameter("gazebo_trial_state_topic", "/waver/gazebo_trial_state")
        self.declare_parameter("map_apply_state_topic", "/waver/map_apply_state")
        self.declare_parameter("battery_state_text_topic", "/waver/battery_state_text")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("mission_command_topic", "/waver/mission_command")
        self.declare_parameter("operator_command_topic", "/waver/operator_command")
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
        self.last_estop_publish_time = 0.0
        self.last_speed_limit_publish = -1.0
        self.last_angular_limit_publish = -1.0
        self.last_speed_limit_publish_time = 0.0
        self.publish_direct_cmd_vel = bool(self.get_parameter("publish_direct_cmd_vel").value)
        self.final_cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.manual_cmd_vel_topic = str(self.get_parameter("manual_cmd_vel_topic").value)
        self.cmd_output_topic = (
            self.final_cmd_vel_topic if self.publish_direct_cmd_vel else self.manual_cmd_vel_topic
        )
        self.auto_cmd_vel_topic = str(self.get_parameter("auto_cmd_vel_topic").value)
        self.latest_auto_cmd = Twist()
        self.last_auto_cmd_time = 0.0
        self.last_amcl_pose_time = 0.0

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
            self.state.map_display_mode = self.normalized_map_mode(
                str(self.get_parameter("map_display_mode").value)
            )

        # 역할: GUI가 내는 수동 후보 명령, 모드, E-Stop, 속도 제한을 ROS graph에 공개한다.
        self.cmd_pub = self.create_publisher(Twist, self.cmd_output_topic, 10)
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("mode_topic").value),
            10,
        )
        self.mission_command_pub = self.create_publisher(
            String,
            str(self.get_parameter("mission_command_topic").value),
            10,
        )
        self.operator_command_pub = self.create_publisher(
            String,
            str(self.get_parameter("operator_command_topic").value),
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
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self.amcl_pose_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("global_path_topic").value),
            self.global_path_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("local_path_topic").value),
            self.local_path_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("active_nav_goal_topic").value),
            self.active_goal_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("object_mission_goal_topic").value),
            self.object_goal_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("lidar_objects_map_topic").value),
            self.lidar_objects_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("elevated_dynamic_target_topic").value),
            self.elevated_targets_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("current_waypoint_topic").value),
            self.current_waypoint_callback,
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
            str(self.get_parameter("patrol_status_topic").value),
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
            str(self.get_parameter("height_filter_debug_topic").value),
            lambda msg: self.set_text_state("height_filter_debug", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("camera_detection_status_topic").value),
            lambda msg: self.set_text_state("camera_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("sound_mission_status_topic").value),
            lambda msg: self.set_text_state("sound_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gazebo_trial_state_topic").value),
            lambda msg: self.set_text_state("gazebo_trial_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("map_apply_state_topic").value),
            lambda msg: self.set_text_state("map_apply_state", msg.data),
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

    @staticmethod
    def normalized_map_mode(raw: str) -> str:
        mode = (raw or "auto").strip().lower()
        if mode in {"slam", "slam_live", "mapping", "live"}:
            return "SLAM_LIVE"
        if mode in {"fixed", "map_fixed", "localization", "patrol", "saved"}:
            return "MAP_FIXED"
        return "AUTO_MAP"

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
        if time.monotonic() - self.last_amcl_pose_time < 1.0:
            return
        with self.lock:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self.state.odom_x = x
            self.state.odom_y = y
            self.state.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self.state.pose_source = "odom"
            self.append_robot_trace_locked(x, y)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        # 역할: 저장 map 기반 Patrol Mode에서는 /amcl_pose를 우선 사용해 map-fixed UI를 맞춘다.
        self.last_amcl_pose_time = time.monotonic()
        with self.lock:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self.state.odom_x = x
            self.state.odom_y = y
            self.state.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self.state.pose_source = "amcl_pose"
            self.append_robot_trace_locked(x, y)

    def append_robot_trace_locked(self, x: float, y: float) -> None:
        trace = self.state.robot_trace
        if trace and math.hypot(x - trace[-1][0], y - trace[-1][1]) < 0.03:
            return
        trace.append((x, y))
        if len(trace) > 1200:
            del trace[: len(trace) - 1200]

    def map_callback(self, msg: OccupancyGrid) -> None:
        # 역할: RViz를 따로 보지 않아도 리모콘 안에서 2D SLAM/map 점유 영역을 확인한다.
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return
        step = max(1, int(max(width, height) / 180))
        occupied: list[tuple[float, float]] = []
        data = msg.data
        max_points = 6000
        for y in range(0, height, step):
            row = y * width
            for x in range(0, width, step):
                value = data[row + x]
                if value > 50:
                    wx = msg.info.origin.position.x + (x + 0.5) * resolution
                    wy = msg.info.origin.position.y + (y + 0.5) * resolution
                    occupied.append((float(wx), float(wy)))
                    if len(occupied) >= max_points:
                        break
            if len(occupied) >= max_points:
                break
        with self.lock:
            self.state.map_received = True
            self.state.map_frame = msg.header.frame_id or "map"
            self.state.map_width = width
            self.state.map_height = height
            self.state.map_resolution = resolution
            self.state.map_origin_x = float(msg.info.origin.position.x)
            self.state.map_origin_y = float(msg.info.origin.position.y)
            self.state.map_occupied = occupied
            self.state.map_sequence += 1

    def global_path_callback(self, msg: Path) -> None:
        # 역할: Nav2/global planner가 만든 전체 경로를 리모콘 지도에 파란 선으로 표시한다.
        points = path_to_xy(msg, limit=900)
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.global_path = points_to_fixed_frame(points, msg.header.frame_id, robot)
            self.state.global_path_frame = fixed_frame_label(msg.header.frame_id)

    def local_path_callback(self, msg: Path) -> None:
        # 역할: 로컬 planner 경로를 리모콘 지도에 초록 선으로 표시한다.
        points = path_to_xy(msg, limit=500)
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.local_path = points_to_fixed_frame(points, msg.header.frame_id, robot)
            self.state.local_path_frame = fixed_frame_label(msg.header.frame_id)

    def active_goal_callback(self, msg: PoseStamped) -> None:
        # 역할: 현재 미션/Nav2 goal을 지도에 표시한다.
        frame_id = msg.header.frame_id
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.active_goal = point_to_fixed_frame(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                frame_id,
                robot,
            )
            self.state.active_goal_frame = fixed_frame_label(frame_id)

    def object_goal_callback(self, msg: PoseStamped) -> None:
        # 역할: LiDAR/radar target mission goal을 지도에 별도 표시한다.
        frame_id = msg.header.frame_id
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.object_goal = point_to_fixed_frame(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                frame_id,
                robot,
            )
            self.state.object_goal_frame = fixed_frame_label(frame_id)

    def lidar_objects_callback(self, msg: PoseArray) -> None:
        # 역할: map/odom 기준 동적 객체 후보를 지도에 표시한다.
        # 혹시 base_link frame 후보가 들어오면 odom pose로 고정 좌표에 투영해
        # 차체 yaw가 바뀌어도 지도 자체가 도는 것처럼 보이지 않게 한다.
        points = [(float(p.position.x), float(p.position.y)) for p in msg.poses[:30]]
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.lidar_objects = points_to_fixed_frame(points, msg.header.frame_id, robot)
            self.state.lidar_objects_frame = fixed_frame_label(msg.header.frame_id)

    def elevated_targets_callback(self, msg: PoseArray) -> None:
        # 역할: height>=3m AND map/odom dynamic 필터를 통과한 최종 target을 지도에 별도 표시한다.
        points = [(float(p.position.x), float(p.position.y)) for p in msg.poses[:20]]
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.elevated_targets = points_to_fixed_frame(points, msg.header.frame_id, robot)
            self.state.elevated_targets_frame = fixed_frame_label(msg.header.frame_id)

    def current_waypoint_callback(self, msg: PoseStamped) -> None:
        # 역할: 현재 순찰 waypoint를 active goal과 구분해서 지도에 표시한다.
        frame_id = msg.header.frame_id
        with self.lock:
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            self.state.current_waypoint = point_to_fixed_frame(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                frame_id,
                robot,
            )
            self.state.current_waypoint_frame = fixed_frame_label(frame_id)

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
        if strategy != "legacy_subprocess":
            if launch_command:
                self.get_logger().warn(
                    "Ignoring auto_launch_command because the operator panel is in "
                    f"{strategy!r} mode. Use legacy_subprocess only for old test launches."
                )
            with self.lock:
                self.state.emergency_stop = False
                self.state.desired_linear = 0.0
                self.state.desired_angular = 0.0
                self.state.active_control = "none"
                self.state.auto_status = "AUTO requested: mission/Nav2 backend"
            self.publish_mission_reset_pulse()
            self.set_mode("AUTO")
            return

        command_text = launch_command or legacy_command
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
                (
                    "use_sim_time:="
                    f"{str(bool(self.get_parameter('auto_use_sim_time').value)).lower()}"
                ),
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

    def send_operator_command(self, command: str) -> None:
        # 역할: 버튼 클릭을 mission backend가 해석할 수 있는 표준 String command로 발행한다.
        # 실차 기본값에서도 /cmd_vel은 직접 만지지 않고 mode/manual 후보와 command 토픽만 낸다.
        normalized = command.strip().upper()
        if not normalized:
            return
        msg = String(data=normalized)
        self.mission_command_pub.publish(msg)
        self.operator_command_pub.publish(msg)
        with self.lock:
            self.state.auto_status = f"operator command: {normalized}"
        if normalized in {"START_PATROL", "RESUME_PATROL", "AUTO_MODE", "GAZEBO_TRIAL_START"}:
            self.start_auto()
        elif normalized == "START_MAPPING":
            with self.lock:
                self.state.map_display_mode = "SLAM_LIVE"
                self.state.auto_status = "mapping requested: live /map"
            self.set_mode("STANDBY")
        elif normalized == "SAVE_MAP":
            with self.lock:
                self.state.auto_status = "save map requested"
        elif normalized in {"LOAD_MAP", "START_LOCALIZATION"}:
            with self.lock:
                self.state.map_display_mode = "MAP_FIXED"
                self.state.auto_status = "fixed map/localization requested"
        elif normalized in {"PAUSE_PATROL", "MANUAL_MODE"}:
            self.set_mode("MANUAL" if normalized == "MANUAL_MODE" else "STANDBY")
        elif normalized == "RETURN_HOME":
            self.set_mode("RETURN_HOME")
        elif normalized in {"STOP", "GAZEBO_TRIAL_STOP"}:
            self.stop_motion(stop_auto=True)
        elif normalized == "EMERGENCY_STOP":
            self.emergency_stop()
        elif normalized == "CLEAR_EMERGENCY_STOP":
            self.reset_estop()

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
                        DriveCommand(
                            linear=linear,
                            angular=angular,
                            source="remote_panel_override",
                        )
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
        # late subscriber가 현재 E-Stop 상태를 놓치지 않도록 heartbeat로도 재발행한다.
        with self.lock:
            estop = self.state.emergency_stop
        now = time.monotonic()
        if estop != self.last_estop_publish or now - self.last_estop_publish_time > 0.5:
            msg = Bool()
            msg.data = estop
            self.estop_pub.publish(msg)
            self.last_estop_publish = estop
            self.last_estop_publish_time = now

    def publish_speed_limits(self, force: bool = False) -> None:
        # 역할: 리모콘 slider 값을 safety_cmd_mux_node의 선속도/각속도 제한 토픽에 반영한다.
        # safety mux가 나중에 켜져도 현재 slider 값을 받을 수 있도록 주기적으로 재발행한다.
        with self.lock:
            speed_limit = float(self.state.speed_limit)
            angular_limit = float(self.state.angular_limit)
        now = time.monotonic()
        stale = now - self.last_speed_limit_publish_time > 0.5
        speed_changed = abs(speed_limit - self.last_speed_limit_publish) > 1e-6
        angular_changed = abs(angular_limit - self.last_angular_limit_publish) > 1e-6
        if force or stale or speed_changed or angular_changed:
            self.speed_limit_pub.publish(Float32(data=speed_limit))
            self.last_speed_limit_publish = speed_limit
            self.angular_limit_pub.publish(Float32(data=angular_limit))
            self.last_angular_limit_publish = angular_limit
            self.last_speed_limit_publish_time = now

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
        self.last_map_draw_time = 0.0

        # 역할: 리모콘 창의 전체 레이아웃을 만든다.
        self.root = tk.Tk()
        self.root.title("Waver Remote Panel")
        self.root.geometry("1240x920")
        self.root.minsize(1040, 820)
        self.root.configure(bg="#0b1117")
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_rowconfigure(2, weight=0)
        self.root.grid_rowconfigure(3, weight=0)
        self.root.grid_rowconfigure(4, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
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
        self.camera_var = tk.StringVar(value="camera: unknown")
        self.sound_var = tk.StringVar(value="sound: unknown")
        self.trial_var = tk.StringVar(value="trial: unknown")
        self.map_apply_var = tk.StringVar(value="map apply: unknown")
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
        label.grid(row=0, column=0, sticky="ew", padx=12, pady=(12, 8))
        self.mode_label = label
        self.mode_hint_label = self.tk.Label(
            self.root,
            textvariable=self.mode_hint_var,
            font=("Sans", 13, "bold"),
            fg="#fffde7",
            bg="#0b1117",
            anchor="w",
        )
        self.mode_hint_label.grid(row=1, column=0, sticky="ew", padx=18, pady=(0, 4))
        badges = self.tk.Frame(self.root, bg="#0b1117")
        badges.grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 6))
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
        # 역할: mission backend가 실제로 해석하는 핵심 command만 노출한다.
        frame = self.tk.Frame(self.root, bg="#0b1117")
        frame.grid(row=3, column=0, sticky="ew", padx=12, pady=4)
        buttons = [
            ("START PATROL", "START_PATROL", "#2e7d32"),
            ("PAUSE", "PAUSE_PATROL", "#546e7a"),
            ("RESUME", "RESUME_PATROL", "#00838f"),
            ("RETURN HOME", "RETURN_HOME", "#5e35b1"),
            ("STOP", "STOP", "#f9a825"),
            ("E-STOP", "EMERGENCY_STOP", "#b71c1c"),
            ("RESET", "CLEAR_EMERGENCY_STOP", "#6a1b9a"),
        ]
        for index, (text, command, color) in enumerate(buttons):
            button = self.tk.Button(
                frame,
                text=text,
                fg="white",
                bg=color,
                activebackground=color,
                font=("Sans", 11, "bold"),
                height=3,
                command=lambda cmd=command: self.node.send_operator_command(cmd),
            )
            button.grid(row=index // 4, column=index % 4, sticky="nsew", padx=4, pady=4)
        for column in range(4):
            frame.grid_columnconfigure(column, weight=1)

    def build_body(self) -> None:
        # 역할: 좌측 조작 영역과 우측 상태 영역을 분리해 현장 가독성을 높인다.
        body = self.tk.Frame(self.root, bg="#0b1117")
        body.grid(row=4, column=0, sticky="nsew", padx=12, pady=8)
        self.root.grid_rowconfigure(4, weight=1)
        self.left_panel = self.tk.Frame(body, bg="#0b1117")
        self.right_panel = self.tk.Frame(body, bg="#0b1117")
        self.left_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.right_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        body.grid_columnconfigure(0, weight=3)
        body.grid_columnconfigure(1, weight=2)
        body.grid_rowconfigure(0, weight=1)
        self.build_map_view()
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

    def build_map_view(self) -> None:
        # 역할: /map, /plan, /local_plan, /odom, object goal을 리모콘 안에 2D로 그린다.
        frame = self.tk.LabelFrame(
            self.left_panel,
            text="Waver Live Map / Nav2 Path",
            bg="#0b1117",
            fg="white",
            font=("Sans", 12, "bold"),
        )
        frame.pack(fill="both", expand=True, pady=(12, 0))
        self.map_canvas = self.tk.Canvas(
            frame,
            height=560,
            bg="#0d151c",
            highlightthickness=1,
            highlightbackground="#37474f",
        )
        self.map_canvas.pack(fill="both", expand=True, padx=8, pady=(8, 4))
        legend = self.tk.Frame(frame, bg="#0b1117")
        legend.pack(fill="x", padx=8, pady=(0, 4))
        for text, color in [
            ("map obstacle", "#90a4ae"),
            ("waver trail", "#ffd54f"),
            ("global path", "#42a5f5"),
            ("local path", "#66bb6a"),
            ("robot", "#ef5350"),
            ("waypoint", "#26c6da"),
            ("nav goal", "#ab47bc"),
            ("cluster", "#ffca28"),
            ("elevated target", "#ff5252"),
        ]:
            item = self.tk.Label(
                legend,
                text=text,
                bg="#0b1117",
                fg=color,
                font=("Sans", 9, "bold"),
                padx=5,
            )
            item.pack(side="left")
        self.map_status_var = self.tk.StringVar(
            value="map: waiting for /map | path: waiting for /plan and /local_plan"
        )
        self.tk.Label(
            frame,
            textvariable=self.map_status_var,
            anchor="w",
            fg="#b0bec5",
            bg="#0b1117",
            font=("Sans", 9, "bold"),
        ).pack(fill="x", padx=8, pady=(0, 8))

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
            ("MAP APPLY", self.map_apply_var),
            ("PATROL", self.patrol_var),
            ("TARGET", self.target_var),
            ("AUTO BACKEND", self.auto_var),
            ("NAV2 CANDIDATE", self.auto_cmd_var),
            ("SCAN SAFETY", self.hazard_var),
        ]
        for title, variable in cards:
            label = self.make_status_card(frame, title, variable)
            if variable is self.hazard_var:
                self.hazard_label = label
        help_text = (
            "Keyboard: Arrow/WASD manual override, Space/K stop, E emergency, "
            "R reset, P start patrol. 지도는 고정이고 Waver 화살표만 회전."
        )
        self.tk.Label(
            frame,
            text=help_text,
            anchor="w",
            fg="#b0bec5",
            bg="#0b1117",
            font=("Sans", 9),
        ).pack(fill="x", pady=(8, 0))

    def build_command_buttons(self, parent) -> None:
        # 역할: mission manager가 구독하는 operator command를 직접 눌러 검증할 수 있게 한다.
        frame = self.tk.LabelFrame(
            parent,
            text="Mission Commands",
            bg="#0b1117",
            fg="white",
            font=("Sans", 11, "bold"),
        )
        frame.pack(fill="x", pady=(0, 8))
        buttons = [
            ("START", "START_PATROL", "#2e7d32"),
            ("PAUSE", "PAUSE_PATROL", "#546e7a"),
            ("RESUME", "RESUME_PATROL", "#00838f"),
            ("HOME", "RETURN_HOME", "#5e35b1"),
            ("STOP", "STOP", "#f9a825"),
            ("E-STOP", "EMERGENCY_STOP", "#b71c1c"),
            ("CLEAR", "CLEAR_EMERGENCY_STOP", "#6a1b9a"),
            ("TARGET", "TARGET_TEST", "#ef6c00"),
            ("SOUND", "SOUND_TEST", "#ad1457"),
            ("MAP", "START_MAPPING", "#0277bd"),
            ("SAVE", "SAVE_MAP", "#00695c"),
            ("LOCALIZE", "START_LOCALIZATION", "#455a64"),
            ("TRIAL ON", "GAZEBO_TRIAL_START", "#558b2f"),
            ("TRIAL OFF", "GAZEBO_TRIAL_STOP", "#795548"),
            ("MANUAL", "MANUAL_MODE", "#1565c0"),
            ("AUTO", "AUTO_MODE", "#2e7d32"),
        ]
        for index, (text, command, color) in enumerate(buttons):
            button = self.tk.Button(
                frame,
                text=text,
                fg="white",
                bg=color,
                activebackground=color,
                font=("Sans", 9, "bold"),
                height=2,
                command=lambda cmd=command: self.node.send_operator_command(cmd),
            )
            button.grid(row=index // 4, column=index % 4, sticky="nsew", padx=3, pady=3)
        for column in range(4):
            frame.grid_columnconfigure(column, weight=1)

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

    def update_map_view(self) -> None:
        # 역할: 패널 안에서 2D SLAM map, Nav2 경로, 로봇 자세, target goal을 빠르게 확인한다.
        canvas = getattr(self, "map_canvas", None)
        if canvas is None:
            return
        now = time.monotonic()
        if now - self.last_map_draw_time < 0.20:
            return
        self.last_map_draw_time = now
        with self.lock:
            map_received = self.state.map_received
            map_frame = self.state.map_frame
            map_width = self.state.map_width
            map_height = self.state.map_height
            map_resolution = self.state.map_resolution
            map_origin_x = self.state.map_origin_x
            map_origin_y = self.state.map_origin_y
            occupied = list(self.state.map_occupied)
            robot = (self.state.odom_x, self.state.odom_y, self.state.odom_yaw)
            robot_trace = list(self.state.robot_trace)
            global_path = list(self.state.global_path)
            local_path = list(self.state.local_path)
            global_path_frame = self.state.global_path_frame
            local_path_frame = self.state.local_path_frame
            active_goal = self.state.active_goal
            object_goal = self.state.object_goal
            current_waypoint = self.state.current_waypoint
            lidar_objects = list(self.state.lidar_objects)
            lidar_objects_frame = self.state.lidar_objects_frame
            elevated_targets = list(self.state.elevated_targets)
            elevated_targets_frame = self.state.elevated_targets_frame
            pose_source = self.state.pose_source
            map_display_mode = self.state.map_display_mode

        width = max(320, int(canvas.winfo_width()))
        height = max(260, int(canvas.winfo_height()))
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#0d151c", outline="")

        points_for_bounds = (
            [(robot[0], robot[1])]
            + robot_trace
            + global_path
            + local_path
            + lidar_objects
            + elevated_targets
        )
        if active_goal is not None:
            points_for_bounds.append(active_goal)
        if object_goal is not None:
            points_for_bounds.append(object_goal)
        if current_waypoint is not None:
            points_for_bounds.append(current_waypoint)
        if map_received and map_width > 0 and map_height > 0 and map_resolution > 0.0:
            min_x = map_origin_x
            min_y = map_origin_y
            max_x = map_origin_x + map_width * map_resolution
            max_y = map_origin_y + map_height * map_resolution
        elif points_for_bounds:
            xs = [p[0] for p in points_for_bounds]
            ys = [p[1] for p in points_for_bounds]
            min_x, max_x = min(xs) - 4.0, max(xs) + 4.0
            min_y, max_y = min(ys) - 4.0, max(ys) + 4.0
        else:
            min_x, max_x = -5.0, 5.0
            min_y, max_y = -5.0, 5.0
        if abs(max_x - min_x) < 1e-3:
            max_x = min_x + 1.0
        if abs(max_y - min_y) < 1e-3:
            max_y = min_y + 1.0

        pad = 18
        scale = min((width - 2 * pad) / (max_x - min_x), (height - 2 * pad) / (max_y - min_y))

        def w2c(x: float, y: float) -> tuple[float, float]:
            cx = pad + (x - min_x) * scale
            cy = height - pad - (y - min_y) * scale
            return cx, cy

        # 배경 grid: map이 없어도 odom 기준 위치와 path 상대관계를 볼 수 있게 한다.
        grid_step_m = 1.0
        start_x = math.floor(min_x / grid_step_m) * grid_step_m
        x = start_x
        while x <= max_x:
            cx, _ = w2c(x, min_y)
            canvas.create_line(cx, pad, cx, height - pad, fill="#17252f")
            x += grid_step_m
        start_y = math.floor(min_y / grid_step_m) * grid_step_m
        y = start_y
        while y <= max_y:
            _, cy = w2c(min_x, y)
            canvas.create_line(pad, cy, width - pad, cy, fill="#17252f")
            y += grid_step_m

        if map_received:
            map_sample = max(1, int(max(map_width, map_height) / 180))
            cell_size = max(1.0, map_resolution * scale * map_sample)
            half = min(4.0, max(1.0, cell_size * 0.5))
            for ox, oy in occupied:
                cx, cy = w2c(ox, oy)
                canvas.create_rectangle(
                    cx - half,
                    cy - half,
                    cx + half,
                    cy + half,
                    fill="#607d8b",
                    outline="",
                )
        else:
            canvas.create_text(
                width / 2,
                34,
                text="Waiting for /map. Odom/path fallback view is active.",
                fill="#ffcc80",
                font=("Sans", 12, "bold"),
            )

        self.draw_polyline(canvas, robot_trace, w2c, "#ffd54f", 2)
        self.draw_polyline(canvas, global_path, w2c, "#42a5f5", 3)
        self.draw_polyline(canvas, local_path, w2c, "#66bb6a", 3)

        for ox, oy in lidar_objects[:20]:
            cx, cy = w2c(ox, oy)
            canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#ffca28", outline="#fff59d")

        for tx, ty in elevated_targets[:12]:
            cx, cy = w2c(tx, ty)
            canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, outline="#ff5252", width=3)
            canvas.create_line(cx - 10, cy, cx + 10, cy, fill="#ff5252", width=2)
            canvas.create_line(cx, cy - 10, cx, cy + 10, fill="#ff5252", width=2)

        if current_waypoint is not None:
            self.draw_cross(canvas, current_waypoint, w2c, "#26c6da", "wp")

        if active_goal is not None:
            self.draw_cross(canvas, active_goal, w2c, "#ab47bc", "goal")
        if object_goal is not None:
            self.draw_cross(canvas, object_goal, w2c, "#ff7043", "target")

        rx, ry, yaw = robot
        rcx, rcy = w2c(rx, ry)
        # 역할: airport map은 넓어서 실제 차체 marker가 너무 작게 보일 수 있다.
        # 로버는 map을 회전시키지 않고, 고정 map 위에서 큰 heading marker와 라벨로 표시한다.
        size = max(18.0, min(34.0, scale * 0.75))
        canvas.create_oval(
            rcx - size * 0.85,
            rcy - size * 0.85,
            rcx + size * 0.85,
            rcy + size * 0.85,
            outline="#ffeb3b",
            width=3,
        )
        nose = (rcx + math.cos(yaw) * size, rcy - math.sin(yaw) * size)
        left = (rcx + math.cos(yaw + 2.5) * size * 0.75, rcy - math.sin(yaw + 2.5) * size * 0.75)
        right = (rcx + math.cos(yaw - 2.5) * size * 0.75, rcy - math.sin(yaw - 2.5) * size * 0.75)
        canvas.create_polygon(nose, left, right, fill="#ef5350", outline="#ffcdd2", width=2)
        canvas.create_oval(rcx - 3, rcy - 3, rcx + 3, rcy + 3, fill="#ffffff", outline="")
        canvas.create_text(
            rcx + size + 6,
            rcy,
            text="UGV_ROVER",
            fill="#ffeb3b",
            anchor="w",
            font=("Sans", 10, "bold"),
        )
        canvas.create_text(
            12,
            12,
            anchor="nw",
            fill="#eceff1",
            font=("Sans", 9, "bold"),
            text=(
                f"{map_display_mode} frame={map_frame if map_received else 'odom'} "
                f"map={'OK' if map_received else 'WAIT'} "
                f"pose={pose_source} "
                f"path={len(global_path)}({global_path_frame or '-'}) "
                f"local={len(local_path)}({local_path_frame or '-'}) "
                f"cluster={len(lidar_objects)}({lidar_objects_frame or '-'}) "
                f"target={len(elevated_targets)}({elevated_targets_frame or '-'})"
            ),
        )
        canvas.create_text(
            width - 12,
            12,
            anchor="ne",
            fill="#b0bec5",
            font=("Sans", 9, "bold"),
            text=(
                f"topic /map:{'OK' if map_received else 'WAIT'}  "
                f"/plan:{'OK' if global_path else 'WAIT'}  "
                f"/local_plan:{'OK' if local_path else 'WAIT'}"
            ),
        )
        self.map_status_var.set(
            f"map={'OK' if map_received else 'waiting'} | "
            f"robot=({rx:+.2f},{ry:+.2f}) yaw={yaw:+.2f} | "
            f"global path={len(global_path)}, local path={len(local_path)} | "
            f"clusters={len(lidar_objects)}, elevated targets={len(elevated_targets)}"
        )

    def draw_polyline(self, canvas, points, w2c, color: str, width: int) -> None:
        # 역할: Path 메시지를 캔버스 선분으로 그린다.
        if len(points) < 2:
            return
        coords: list[float] = []
        for x, y in points:
            cx, cy = w2c(x, y)
            coords.extend([cx, cy])
        canvas.create_line(*coords, fill=color, width=width, smooth=True)

    def draw_cross(self, canvas, point, w2c, color: str, label: str) -> None:
        # 역할: active goal과 object mission goal을 서로 다른 색으로 표시한다.
        cx, cy = w2c(point[0], point[1])
        canvas.create_line(cx - 7, cy, cx + 7, cy, fill=color, width=3)
        canvas.create_line(cx, cy - 7, cx, cy + 7, fill=color, width=3)
        canvas.create_text(
            cx + 10,
            cy - 10,
            text=label,
            fill=color,
            anchor="w",
            font=("Sans", 9, "bold"),
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
        elif script == "estop_smoke":
            # 역할: E-STOP 버튼과 RESET 버튼의 실제 callback 경로를 자동 검증한다.
            steps = [
                (800, lambda: self.press_direction(1.0, 0.0, "forward-before-estop")),
                (2000, self.node.emergency_stop),
                (4200, self.node.reset_estop),
                (5600, self.close_if_demo_requested),
            ]
        elif script == "keyboard_smoke":
            # 역할: 버튼 callback이 아니라 Tk 키보드 이벤트 처리 경로 자체를 검증한다.
            # Up/W/A/D/Space/E/R/P를 차례로 넣어 수동 조작, 정지, E-Stop, reset, AUTO 요청을 확인한다.
            def key_event(keysym: str):
                return type("KeyEvent", (), {"keysym": keysym})()

            steps = [
                (800, lambda: self.on_key_press(key_event("Up"))),
                (1800, lambda: self.on_key_release(key_event("Up"))),
                (2400, lambda: self.on_key_press(key_event("a"))),
                (3300, lambda: self.on_key_release(key_event("a"))),
                (3900, lambda: self.on_key_press(key_event("d"))),
                (4800, lambda: self.on_key_release(key_event("d"))),
                (5400, lambda: self.on_key_press(key_event("space"))),
                (6400, lambda: self.on_key_press(key_event("e"))),
                (7800, lambda: self.on_key_press(key_event("r"))),
                (9000, lambda: self.on_key_press(key_event("p"))),
                (12000, self.close_if_demo_requested),
            ]
        elif script == "command_buttons_smoke":
            steps = [
                (700, lambda: self.node.send_operator_command("START_PATROL")),
                (1700, lambda: self.node.send_operator_command("PAUSE_PATROL")),
                (2700, lambda: self.node.send_operator_command("RESUME_PATROL")),
                (3700, lambda: self.node.send_operator_command("TARGET_TEST")),
                (4700, lambda: self.node.send_operator_command("SOUND_TEST")),
                (5700, lambda: self.node.send_operator_command("RETURN_HOME")),
                (6700, lambda: self.node.send_operator_command("STOP")),
                (7700, lambda: self.node.send_operator_command("EMERGENCY_STOP")),
                (8700, lambda: self.node.send_operator_command("CLEAR_EMERGENCY_STOP")),
                (9800, self.close_if_demo_requested),
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
            camera = self.state.camera_state
            sound = self.state.sound_state
            trial = self.state.gazebo_trial_state
            map_apply = self.state.map_apply_state
            height_filter = self.state.height_filter_debug
            auto = self.state.auto_status
            auto_linear = self.state.latest_auto_linear
            auto_angular = self.state.latest_auto_angular
            auto_age = self.state.latest_auto_age
            active = self.state.active_control
            pose_source = self.state.pose_source
            odom_yaw = self.state.odom_yaw
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
        self.odom_var.set(
            f"pose({pose_source}): x={odom_x:+.2f}, y={odom_y:+.2f}, yaw={odom_yaw:+.2f}"
        )
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
            f"class={target_class}, conf={target_confidence:.2f}, bird={bird_confirmed}\n"
            f"height/dynamic: {height_filter[:120]}"
        )
        self.camera_var.set(f"camera: {camera}")
        self.sound_var.set(f"sound: {sound}")
        self.trial_var.set(f"trial: {trial}")
        self.map_apply_var.set(f"map apply: {map_apply}")
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
            self.update_map_view()
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


def yaw_from_quaternion(q) -> float:
    # 역할: tf_transformations 의존성 없이 odom orientation을 yaw(rad)로 바꾼다.
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def path_to_xy(msg: Path, limit: int) -> list[tuple[float, float]]:
    # 역할: 긴 Nav2 path를 GUI에서 부담 없이 그릴 수 있게 일정 간격으로 줄인다.
    poses = msg.poses
    if not poses:
        return []
    step = max(1, int(len(poses) / max(limit, 1)))
    result = [
        (float(p.pose.position.x), float(p.pose.position.y))
        for p in poses[::step]
    ]
    if poses[-1] is not poses[::step][-1]:
        result.append((float(poses[-1].pose.position.x), float(poses[-1].pose.position.y)))
    return result


def fixed_frame_label(frame_id: str) -> str:
    # 역할: GUI legend에 사람이 읽기 쉬운 frame 이름을 표시한다.
    return (frame_id or "odom").strip() or "odom"


def is_robot_relative_frame(frame_id: str) -> bool:
    # 역할: base_link/local planner frame은 지도 좌표가 아니라 로봇 상대 좌표다.
    # 이 값들을 그대로 그리면 차체 회전 때 경로나 장애물이 지도 위에서 도는 것처럼 보인다.
    normalized = (frame_id or "").strip().lstrip("/")
    return normalized in {
        "base_link",
        "base_footprint",
        "base",
        "robot",
        "laser",
        "lidar",
        "camera_link",
    }


def point_to_fixed_frame(
    x: float,
    y: float,
    frame_id: str,
    robot: tuple[float, float, float],
) -> tuple[float, float]:
    # 역할: robot-relative 좌표를 현재 odom/map 평면에 투영한다.
    # map/odom frame 좌표는 이미 고정 좌표계이므로 그대로 둔다.
    if not is_robot_relative_frame(frame_id):
        return (x, y)
    rx, ry, yaw = robot
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        rx + cos_yaw * x - sin_yaw * y,
        ry + sin_yaw * x + cos_yaw * y,
    )


def points_to_fixed_frame(
    points: list[tuple[float, float]],
    frame_id: str,
    robot: tuple[float, float, float],
) -> list[tuple[float, float]]:
    # 역할: local path/object 후보가 base_link frame으로 들어와도 fixed map canvas에 안정적으로 표시한다.
    if not points:
        return []
    if not is_robot_relative_frame(frame_id):
        return list(points)
    return [point_to_fixed_frame(x, y, frame_id, robot) for x, y in points]


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

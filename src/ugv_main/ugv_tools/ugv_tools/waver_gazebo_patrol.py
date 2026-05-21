#!/usr/bin/env python3
# Copyright 2026 Waver project contributors.
"""Waver assisted waypoint patrol node for ugv_gazebo and low-speed field tests.

Role:
  - Drive waypoint loops using `/odom` and `/cmd_vel`.
  - Behave like a cautious operator: align heading first, slow near turns/goals,
    stop on stale pose, and wait/recover instead of forcing motion.
  - Use optional `/scan` as a local driving-assist layer. When a front obstacle is
    too close, Waver stops, waits, turns toward the clearer side, or backs up only
    when the rear sector is clear.

This is still not "human-level autonomy". It is a conservative waypoint helper for
Gazebo and supervised outdoor tests. For production outdoor navigation, keep Nav2,
costmaps, localization, collision monitor, and a human E-stop operator in the loop.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ugv_tools.waver_drive_assist import (
    AssistConfig,
    DriveAssist,
    DriveCommand,
    HazardLevel,
    clamp,
)


# 역할: odom quaternion을 yaw로 바꿔 단순 P-controller가 사용할 방향값을 만든다.
def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# 역할: 각도 오차를 -pi~pi로 정규화해 좌/우 회전 방향이 튀지 않게 한다.
def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# 역할: 순찰 노드의 현재 판단 상태를 로그와 상태 토픽에 일관되게 표시한다.
class PatrolState(str, Enum):
    WAIT_FOR_ODOM = "WAIT_FOR_ODOM"
    CRUISE = "CRUISE"
    ALIGN_HEADING = "ALIGN_HEADING"
    APPROACH_GOAL = "APPROACH_GOAL"
    OBSTACLE_WAIT = "OBSTACLE_WAIT"
    RECOVERY_TURN = "RECOVERY_TURN"
    RECOVERY_BACKUP = "RECOVERY_BACKUP"
    STUCK_WAIT = "STUCK_WAIT"
    ARRIVED = "ARRIVED"
    COMPLETE = "COMPLETE"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class Waypoint:
    # 역할: Gazebo/odom 기준 시험 웨이포인트 한 점을 표현한다.
    x: float
    y: float
    yaw: float = 0.0
    name: str = "waypoint"


# 역할: Gazebo에서 Waver 자율 순찰 후보를 실험하는 저속 odom 기반 노드다.
class WaverGazeboPatrol(Node):
    """Odometry-based waypoint follower with a local driving-assist policy.

    Role:
      - Generate smooth, low-speed commands between waypoints.
      - Make small local decisions: slow, wait, turn, backup, retry, or stop.
      - Keep the behavior observable through state-change logs.
    """

    def __init__(self):
        super().__init__("waver_gazebo_patrol")
        # 역할: 토픽/웨이포인트/속도/안전 threshold를 launch에서 바로 바꿀 수 있게 선언한다.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter(
            "waypoints",
            ["1.0,0.0,0.0", "1.0,1.0,1.57", "0.0,1.0,3.14", "0.0,0.0,0.0"],
        )
        self.declare_parameter("waypoints_csv", "")
        self.declare_parameter("patrol_center_x", 0.0)
        self.declare_parameter("patrol_center_y", 0.0)
        self.declare_parameter("max_patrol_radius_m", 3.0)
        self.declare_parameter("clamp_waypoints_to_radius", True)
        self.declare_parameter("loop_count", 1)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 1.4)
        self.declare_parameter("reverse_speed", 0.08)
        self.declare_parameter("xy_tolerance", 0.18)
        self.declare_parameter("yaw_tolerance", 0.35)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("append_clicked_goals", True)
        self.declare_parameter("enable_scan_assist", True)
        self.declare_parameter("lidar_required", False)
        self.declare_parameter("hard_stop_distance_m", 0.45)
        self.declare_parameter("slow_down_distance_m", 1.2)
        self.declare_parameter("clear_distance_m", 1.8)
        self.declare_parameter("min_valid_scan_points", 40)
        self.declare_parameter("heading_align_threshold_rad", 0.75)
        self.declare_parameter("approach_distance_m", 0.55)
        self.declare_parameter("blocked_wait_s", 2.0)
        self.declare_parameter("recovery_action_s", 0.8)
        self.declare_parameter("stuck_timeout_s", 8.0)
        self.declare_parameter("progress_epsilon_m", 0.01)
        self.declare_parameter("max_recovery_attempts", 3)
        self.declare_parameter("state_topic", "/waver/patrol_state")
        self.declare_parameter("current_waypoint_topic", "/waver/current_waypoint")

        # 역할: 파라미터를 내부 제어 변수로 확정한다.
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.loop_count = int(self.get_parameter("loop_count").value)
        self.patrol_center_x = float(self.get_parameter("patrol_center_x").value)
        self.patrol_center_y = float(self.get_parameter("patrol_center_y").value)
        self.max_patrol_radius_m = float(self.get_parameter("max_patrol_radius_m").value)
        self.clamp_waypoints_to_radius = bool(
            self.get_parameter("clamp_waypoints_to_radius").value
        )
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.xy_tolerance = float(self.get_parameter("xy_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.append_clicked_goals = bool(self.get_parameter("append_clicked_goals").value)
        self.heading_align_threshold = float(
            self.get_parameter("heading_align_threshold_rad").value
        )
        self.approach_distance = float(self.get_parameter("approach_distance_m").value)
        self.blocked_wait_s = float(self.get_parameter("blocked_wait_s").value)
        self.recovery_action_s = float(self.get_parameter("recovery_action_s").value)
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)
        self.progress_epsilon_m = float(self.get_parameter("progress_epsilon_m").value)
        self.max_recovery_attempts = int(self.get_parameter("max_recovery_attempts").value)

        # 역할: 순찰 명령도 공통 주행 보조 계층을 통과시켜 저속/가속/장애물 제한을 적용한다.
        self.assist = DriveAssist(
            AssistConfig(
                max_linear_speed=self.max_linear_speed,
                max_angular_speed=self.max_angular_speed,
                max_linear_accel=float(self.get_parameter("max_linear_accel").value),
                max_angular_accel=float(self.get_parameter("max_angular_accel").value),
                reverse_speed=float(self.get_parameter("reverse_speed").value),
                hard_stop_distance_m=float(self.get_parameter("hard_stop_distance_m").value),
                slow_down_distance_m=float(self.get_parameter("slow_down_distance_m").value),
                clear_distance_m=float(self.get_parameter("clear_distance_m").value),
                lidar_required=bool(self.get_parameter("lidar_required").value),
                min_valid_scan_points=int(self.get_parameter("min_valid_scan_points").value),
            )
        )

        # 역할: launch 문자열 또는 RViz clicked goal로 들어온 waypoint를 내부 배열로 관리한다.
        waypoint_items = self._waypoint_items()
        self.waypoints = self.parse_waypoints(waypoint_items)
        self.current_index = 0
        self.completed_loops = 0
        self.pose: tuple[float, float, float] | None = None
        self.last_odom_time = self.get_clock().now()
        self.state = PatrolState.WAIT_FOR_ODOM
        self.state_started = time.monotonic()
        self.blocked_since = 0.0
        self.recovery_until = 0.0
        self.recovery_attempts = 0
        self.best_distance = math.inf
        self.last_progress_time = time.monotonic()
        self.last_assist_warn_time = 0.0
        self.active = bool(self.waypoints)

        # 역할: /cmd_vel, /odom, /scan, /goal_pose를 연결하고 모니터링용 상태 토픽을 만든다.
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String

        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.current_waypoint_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("current_waypoint_topic").value),
            10,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.create_subscription(PoseStamped, goal_pose_topic, self.goal_pose_callback, 10)
        if bool(self.get_parameter("enable_scan_assist").value):
            # 역할: Gazebo/실차 LaserScan은 best_effort인 경우가 많아 sensor_data QoS로 맞춘다.
            self.create_subscription(
                LaserScan,
                scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )
        self.timer = self.create_timer(0.05, self.control_tick)

        self.get_logger().info(
            f"Waver assisted patrol loaded {len(self.waypoints)} waypoint(s); "
            f"radius <= {self.max_patrol_radius_m:.2f}m around "
            f"({self.patrol_center_x:.2f}, {self.patrol_center_y:.2f})"
        )

    def _waypoint_items(self) -> list[str]:
        # 역할: launch에서 semicolon CSV를 주면 우선 사용하고, 없으면 YAML 파일/list 파라미터를 사용한다.
        waypoints_csv = str(self.get_parameter("waypoints_csv").value)
        if waypoints_csv:
            return [item.strip() for item in waypoints_csv.split(";") if item.strip()]
        waypoint_file = str(self.get_parameter("waypoint_file").value).strip()
        if waypoint_file:
            return self._waypoint_items_from_file(waypoint_file)
        return list(self.get_parameter("waypoints").value)

    def _waypoint_items_from_file(self, waypoint_file: str) -> list[str]:
        # 역할: 현장 좌표를 코드 수정 없이 YAML 파일에서 불러온다.
        path = os.path.expanduser(waypoint_file)
        if not os.path.isabs(path):
            path = os.path.abspath(path)
        try:
            import yaml
        except ImportError:
            self.get_logger().error(
                "python3-yaml is required to load waypoint_file; "
                "install it or use waypoints_csv instead"
            )
            return []
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().error(f"Cannot read waypoint_file={path}: {exc}")
            return []
        raw_waypoints = data.get("waypoints", [])
        if not isinstance(raw_waypoints, list):
            self.get_logger().error(f"waypoint_file={path} has no list named 'waypoints'")
            return []
        items: list[str] = []
        for index, item in enumerate(raw_waypoints):
            if isinstance(item, dict):
                try:
                    x = float(item["x"])
                    y = float(item["y"])
                    yaw = float(item.get("yaw", item.get("yaw_rad", 0.0)))
                except (KeyError, TypeError, ValueError) as exc:
                    self.get_logger().warn(f"Skipping bad waypoint #{index} in {path}: {exc}")
                    continue
                name = str(item.get("name", f"wp_{index}")).replace(",", "_")
                items.append(f"{x},{y},{yaw},{name}")
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                try:
                    x = float(item[0])
                    y = float(item[1])
                    yaw = float(item[2]) if len(item) > 2 else 0.0
                except (TypeError, ValueError) as exc:
                    self.get_logger().warn(f"Skipping bad waypoint #{index} in {path}: {exc}")
                    continue
                items.append(f"{x},{y},{yaw},wp_{index}")
            else:
                items.append(str(item))
        self.get_logger().info(f"Loaded {len(items)} waypoint item(s) from {path}")
        return items

    def parse_waypoints(self, raw_items: list[str]) -> list[Waypoint]:
        # 역할: "x,y,yaw[,name]" 문자열을 Waypoint 객체로 바꾼다.
        waypoints: list[Waypoint] = []
        for index, item in enumerate(raw_items):
            parts = [part.strip() for part in str(item).split(",")]
            if len(parts) < 2:
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
            except ValueError as exc:
                self.get_logger().warn(f"Skipping malformed waypoint {item!r}: {exc}")
                continue
            name = parts[3] if len(parts) > 3 and parts[3] else f"wp_{index}"
            waypoint = Waypoint(x, y, yaw, name)
            limited = self.limit_waypoint_to_radius(waypoint)
            if limited is not None:
                waypoints.append(limited)
        return waypoints

    def limit_waypoint_to_radius(self, waypoint: Waypoint) -> Waypoint | None:
        # 역할: 실증 전 단계에서 자율순찰 목표가 기준점 반경 3m 밖으로 나가지 못하게 한다.
        if self.max_patrol_radius_m <= 0.0:
            return waypoint
        dx = waypoint.x - self.patrol_center_x
        dy = waypoint.y - self.patrol_center_y
        radius = math.hypot(dx, dy)
        if radius <= self.max_patrol_radius_m:
            return waypoint
        if not self.clamp_waypoints_to_radius:
            self.get_logger().warn(
                f"Rejected {waypoint.name}: {radius:.2f}m exceeds "
                f"max_patrol_radius_m={self.max_patrol_radius_m:.2f}"
            )
            return None
        scale = self.max_patrol_radius_m / max(radius, 1e-6)
        limited = Waypoint(
            self.patrol_center_x + dx * scale,
            self.patrol_center_y + dy * scale,
            waypoint.yaw,
            waypoint.name,
        )
        self.get_logger().warn(
            f"Clamped {waypoint.name} from ({waypoint.x:.2f}, {waypoint.y:.2f}) "
            f"to ({limited.x:.2f}, {limited.y:.2f}) because radius {radius:.2f}m "
            f"exceeds {self.max_patrol_radius_m:.2f}m"
        )
        return limited

    def odom_callback(self, msg: Odometry) -> None:
        # 역할: Gazebo/odom pose를 현재 위치 추정값으로 저장한다.
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose = (pos.x, pos.y, yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w))
        self.last_odom_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        # 역할: 장애물 sector와 TTC를 갱신해 순찰 중 독립 hard stop에 사용한다.
        summary = self.assist.update_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )
        if summary.hazard in {HazardLevel.STOP, HazardLevel.SLOW}:
            self.get_logger().debug(
                f"scan hazard={summary.hazard.value} "
                f"front={summary.front:.2f}m escape={summary.best_escape}"
            )

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        # 역할: RViz 2D Goal로 찍은 위치를 임시 waypoint로 추가해 Gazebo에서 빠르게 시험한다.
        if not self.append_clicked_goals:
            return
        pose = msg.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        waypoint = Waypoint(
            pose.position.x,
            pose.position.y,
            yaw,
            f"clicked_{len(self.waypoints)}",
        )
        limited = self.limit_waypoint_to_radius(waypoint)
        if limited is None:
            return
        waypoint = limited
        self.waypoints.append(waypoint)
        self.active = True
        if self.state == PatrolState.COMPLETE:
            self._set_state(PatrolState.CRUISE, "new clicked waypoint")
        self.get_logger().info(f"Added clicked waypoint {waypoint.x:.2f}, {waypoint.y:.2f}")

    def control_tick(self) -> None:
        # 역할: 20Hz 제어 루프. odom, scan, progress 상태를 보고 다음 안전 명령을 결정한다.
        if not self.active or not self.waypoints:
            self._set_state(PatrolState.COMPLETE, "inactive")
            self.publish_stop()
            return
        if self.pose is None:
            self._set_state(PatrolState.WAIT_FOR_ODOM, "no odom yet")
            self.publish_stop()
            return

        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.odom_timeout_s:
            self._set_state(PatrolState.EMERGENCY_STOP, "odom stale")
            self.publish_stop()
            return

        if self._handle_recovery_action():
            return

        target = self.waypoints[self.current_index]
        self.publish_current_waypoint(target)
        x, y, yaw = self.pose
        dx = target.x - x
        dy = target.y - y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - yaw)
        yaw_error = normalize_angle(target.yaw - yaw)

        self._update_progress(distance)
        if self._is_stuck(distance):
            self._set_state(PatrolState.STUCK_WAIT, "no progress")
            self.publish_stop()
            return

        if distance < self.xy_tolerance:
            if abs(yaw_error) < self.yaw_tolerance:
                self._set_state(PatrolState.ARRIVED, target.name)
                self.advance_waypoint()
                self.publish_stop()
                return
            self._set_state(PatrolState.ALIGN_HEADING, "final yaw")
            angular = clamp(
                1.2 * yaw_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.publish_drive(
                DriveCommand(0.0, angular, "patrol", "final yaw align")
            )
            return

        hazard = self.assist.scan.hazard
        if hazard == HazardLevel.STOP:
            self._handle_blocked_path()
            return

        if abs(heading_error) > self.heading_align_threshold:
            self._set_state(PatrolState.ALIGN_HEADING, "heading before forward")
            angular = clamp(
                1.4 * heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.publish_drive(DriveCommand(0.0, angular, "patrol", "align heading"))
            return

        speed = self._regulated_speed(distance, heading_error)
        state = (
            PatrolState.APPROACH_GOAL
            if distance < self.approach_distance
            else PatrolState.CRUISE
        )
        self._set_state(state, f"distance {distance:.2f}m")
        angular = clamp(
            1.5 * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.publish_drive(
            DriveCommand(speed, angular, "patrol", "regulated waypoint tracking")
        )

    def _regulated_speed(self, distance: float, heading_error: float) -> float:
        # 역할: 목표 접근 거리, heading 오차, scan 위험도에 따라 사람처럼 천천히 감속한다.
        approach_scale = clamp(distance / max(self.approach_distance, 0.1), 0.25, 1.0)
        heading_scale = clamp(
            1.0 - abs(heading_error) / max(self.heading_align_threshold, 0.1),
            0.25,
            1.0,
        )
        caution_hazards = {HazardLevel.CAUTION, HazardLevel.SLOW}
        hazard_scale = 0.65 if self.assist.scan.hazard in caution_hazards else 1.0
        return self.max_linear_speed * approach_scale * heading_scale * hazard_scale

    def _handle_blocked_path(self) -> None:
        # 역할: 전방 hard stop 상황에서 즉시 정지 후 기다리고, 장시간 막히면 짧은 recovery를 시도한다.
        now = time.monotonic()
        if self.blocked_since == 0.0:
            self.blocked_since = now
        hint = self.assist.recovery_hint()
        self._set_state(PatrolState.OBSTACLE_WAIT, f"front blocked, hint={hint}")
        self.publish_stop()
        if now - self.blocked_since < self.blocked_wait_s:
            return
        if self.recovery_attempts >= self.max_recovery_attempts:
            self._set_state(PatrolState.EMERGENCY_STOP, "recovery attempts exceeded")
            self.publish_stop()
            return
        hint = self.assist.recovery_hint()
        self.recovery_attempts += 1
        self.recovery_until = now + self.recovery_action_s
        if hint == "turn_left":
            self._set_state(PatrolState.RECOVERY_TURN, "turn left around obstacle")
            self.publish_drive(
                DriveCommand(0.0, 0.35, "patrol_recovery", "turn left recovery")
            )
        elif hint == "turn_right":
            self._set_state(PatrolState.RECOVERY_TURN, "turn right around obstacle")
            self.publish_drive(
                DriveCommand(0.0, -0.35, "patrol_recovery", "turn right recovery")
            )
        elif hint == "backup":
            self._set_state(PatrolState.RECOVERY_BACKUP, "rear clear")
            self.publish_drive(
                DriveCommand(-0.08, 0.0, "patrol_recovery", "backup recovery")
            )
        else:
            self.recovery_until = 0.0
            self.publish_stop()

    def _handle_recovery_action(self) -> bool:
        # 역할: recovery 동작은 최대 recovery_action_s 동안만 유지하고 이후 정지한다.
        if self.recovery_until <= 0.0:
            return False
        if time.monotonic() >= self.recovery_until:
            self.recovery_until = 0.0
            self.publish_stop()
            return False
        if self.state == PatrolState.RECOVERY_TURN:
            direction = 1.0 if self.assist.recovery_hint() != "turn_right" else -1.0
            self.publish_drive(
                DriveCommand(
                    0.0,
                    0.35 * direction,
                    "patrol_recovery",
                    "timed turn recovery",
                )
            )
            return True
        if self.state == PatrolState.RECOVERY_BACKUP:
            self.publish_drive(
                DriveCommand(-0.08, 0.0, "patrol_recovery", "timed backup recovery")
            )
            return True
        return False

    def _update_progress(self, distance: float) -> None:
        # 역할: 목표까지 거리가 실제로 줄어드는지 추적해 stuck 여부를 판단한다.
        now = time.monotonic()
        if distance + self.progress_epsilon_m < self.best_distance:
            self.best_distance = distance
            self.last_progress_time = now
            if self.assist.scan.hazard != HazardLevel.STOP:
                self.blocked_since = 0.0
                self.recovery_attempts = 0

    def _is_stuck(self, distance: float) -> bool:
        # 역할: 장애물이 아닌데도 목표 접근이 없으면 무리하게 밀지 않고 정지 상태로 전환한다.
        if distance < self.xy_tolerance * 2.0:
            return False
        recovery_states = {
            PatrolState.OBSTACLE_WAIT,
            PatrolState.RECOVERY_TURN,
            PatrolState.RECOVERY_BACKUP,
        }
        if self.state in recovery_states:
            return False
        return time.monotonic() - self.last_progress_time > self.stuck_timeout_s

    def advance_waypoint(self) -> None:
        # 역할: 도착한 waypoint를 넘기고 loop_count 정책에 따라 반복 또는 종료한다.
        reached = self.waypoints[self.current_index]
        self.get_logger().info(f"Reached {reached.name} at {reached.x:.2f}, {reached.y:.2f}")
        self.current_index += 1
        self.best_distance = math.inf
        self.last_progress_time = time.monotonic()
        self.blocked_since = 0.0
        self.recovery_attempts = 0
        if self.current_index < len(self.waypoints):
            self._set_state(PatrolState.CRUISE, "next waypoint")
            return
        self.current_index = 0
        self.completed_loops += 1
        if self.loop_count >= 0 and self.completed_loops >= self.loop_count:
            self.active = False
            self._set_state(PatrolState.COMPLETE, "requested loops complete")
            self.get_logger().info("Completed requested Waver Gazebo patrol loop(s)")
        else:
            self._set_state(PatrolState.CRUISE, "loop restart")

    def publish_drive(self, command: DriveCommand) -> None:
        # 역할: 순찰 후보 명령을 DriveAssist로 필터링한 뒤 /cmd_vel로 발행한다.
        assisted = self.assist.assisted_command(command)
        if command.linear > 0.0 and assisted.is_stop:
            now = time.monotonic()
            if now - self.last_assist_warn_time > 2.0:
                self.last_assist_warn_time = now
                self.get_logger().warn(
                    "DriveAssist stopped patrol command: "
                    f"candidate={command.reason}, result={assisted.reason}, "
                    f"hazard={self.assist.scan.hazard.value}, "
                    f"front={self.assist.scan.front:.2f}m, "
                    f"scan_age={self.assist.scan.age_s:.2f}s"
                )
        msg = Twist()
        msg.linear.x = assisted.linear
        msg.angular.z = assisted.angular
        self.cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        # 역할: 예외/odom timeout/순찰 종료 시 Gazebo와 실차 브리지 모두가 정지하게 한다.
        try:
            self.assist.last_output = DriveCommand()
            self.cmd_pub.publish(Twist())
        except Exception as exc:
            self.get_logger().debug(f"Stop publish skipped during shutdown: {exc}")

    def _set_state(self, state: PatrolState, reason: str) -> None:
        # 역할: 상태 변화만 로그와 /waver/patrol_state로 발행해 실증 중 확인성을 높인다.
        if state == self.state:
            return
        self.state = state
        self.state_started = time.monotonic()
        self.get_logger().info(f"Patrol state -> {state.value}: {reason}")
        self.publish_state(reason)

    def publish_state(self, reason: str = "") -> None:
        # 역할: ros2 topic echo로 순찰 상태와 원인을 바로 확인할 수 있게 한다.
        from std_msgs.msg import String

        msg = String()
        msg.data = self.state.value if not reason else f"{self.state.value}: {reason}"
        self.state_pub.publish(msg)

    def publish_current_waypoint(self, waypoint: Waypoint) -> None:
        # 역할: 현재 목표 waypoint를 /waver/current_waypoint로 발행해 RViz/터미널에서 점검한다.
        from geometry_msgs.msg import PointStamped

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x = waypoint.x
        msg.point.y = waypoint.y
        msg.point.z = 0.0
        self.current_waypoint_pub.publish(msg)

    def destroy_node(self) -> bool:
        # 역할: launch 종료 시에도 최소 5회 stop을 발행한다.
        if rclpy.ok():
            for _ in range(5):
                self.publish_stop()
        return super().destroy_node()


def main(args=None):
    # 역할: ROS2 노드 생명주기를 관리하고 Ctrl-C 시 안전 정지를 보장한다.
    rclpy.init(args=args)
    node = WaverGazeboPatrol()
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

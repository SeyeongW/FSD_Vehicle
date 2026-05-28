#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass
class RobotPose:
    x: float
    y: float
    yaw: float


@dataclass
class TargetSample:
    x: float
    y: float
    z: float
    frame_id: str
    seen_time: float


class LidarTargetFollowNode(Node):
    """Turn/follow controller driven only by LiDAR cluster target coordinates."""

    def __init__(self) -> None:
        super().__init__("lidar_target_follow_node")

        self.declare_parameter("target_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/waver/object_mission_goal")
        self.declare_parameter("state_topic", "/waver/lidar_target_follow_state")
        self.declare_parameter("sound_status_topic", "/waver/sound_mission_status")
        self.declare_parameter("sound_request_topic", "/waver/sound_alert_request")
        self.declare_parameter("enable_cmd_vel_output", False)
        self.declare_parameter("publish_sound_request", True)
        self.declare_parameter("emit_terminal_bell", False)
        self.declare_parameter("min_target_height_m", 3.0)
        self.declare_parameter("max_target_height_m", 8.0)
        self.declare_parameter("warning_distance_m", 3.0)
        self.declare_parameter("follow_start_distance_m", 3.0)
        self.declare_parameter("hold_distance_m", 1.8)
        self.declare_parameter("max_track_distance_m", 15.0)
        self.declare_parameter("target_timeout_s", 1.2)
        self.declare_parameter("track_match_gate_m", 3.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("angle_deadband_rad", 0.06)
        self.declare_parameter("rotate_in_place_threshold_rad", 0.35)
        self.declare_parameter("angular_gain", 1.4)
        self.declare_parameter("linear_gain", 0.28)
        self.declare_parameter("max_angular_speed", 0.65)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("goal_standoff_m", 1.8)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.sound_status_topic = str(self.get_parameter("sound_status_topic").value)
        self.sound_request_topic = str(self.get_parameter("sound_request_topic").value)
        self.enable_cmd_vel_output = bool(self.get_parameter("enable_cmd_vel_output").value)
        self.publish_sound_request_enabled = bool(
            self.get_parameter("publish_sound_request").value
        )
        self.emit_terminal_bell = bool(self.get_parameter("emit_terminal_bell").value)

        self.robot: RobotPose | None = None
        self.target: TargetSample | None = None
        self.last_state = ""
        self.last_sound_status = ""
        self.last_bell_time = 0.0

        self.create_subscription(PoseArray, self.target_topic, self.target_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sound_status_pub = self.create_publisher(String, self.sound_status_topic, 10)
        self.sound_request_pub = self.create_publisher(Bool, self.sound_request_topic, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(period, self.control_tick)

        mode = "direct /cmd_vel" if self.enable_cmd_vel_output else "state/goal only"
        self.get_logger().info(
            f"LiDAR target follow node started: {self.target_topic} -> {mode}, "
            f"warning<={float(self.get_parameter('warning_distance_m').value):.1f}m"
        )

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.robot = RobotPose(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def target_callback(self, msg: PoseArray) -> None:
        now = self.now_sec()
        robot = self.robot
        candidates: list[TargetSample] = []
        for pose in msg.poses:
            if not self.target_height_ok(pose):
                continue
            if robot is not None:
                distance = math.hypot(pose.position.x - robot.x, pose.position.y - robot.y)
                if distance > float(self.get_parameter("max_track_distance_m").value):
                    continue
            candidates.append(
                TargetSample(
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                    msg.header.frame_id or "odom",
                    now,
                )
            )
        if not candidates:
            return
        self.target = self.select_target(candidates)

    def target_height_ok(self, pose: Pose) -> bool:
        z = float(pose.position.z)
        return (
            float(self.get_parameter("min_target_height_m").value)
            <= z
            <= float(self.get_parameter("max_target_height_m").value)
        )

    def select_target(self, candidates: list[TargetSample]) -> TargetSample:
        robot = self.robot
        current = self.target
        gate = float(self.get_parameter("track_match_gate_m").value)
        if current is not None:
            nearby = [
                (math.hypot(c.x - current.x, c.y - current.y), c)
                for c in candidates
                if math.hypot(c.x - current.x, c.y - current.y) <= gate
            ]
            if nearby:
                nearby.sort(key=lambda item: item[0])
                return nearby[0][1]
        if robot is None:
            return candidates[0]
        return min(candidates, key=lambda c: math.hypot(c.x - robot.x, c.y - robot.y))

    def control_tick(self) -> None:
        if self.robot is None:
            self.publish_stop_if_enabled()
            self.publish_status("WAIT_FOR_ODOM", sound_request=False)
            return
        target = self.target
        if target is None:
            self.publish_stop_if_enabled()
            self.publish_status("WAIT_FOR_LIDAR_TARGET", sound_request=False)
            return
        age = self.now_sec() - target.seen_time
        if age > float(self.get_parameter("target_timeout_s").value):
            self.publish_stop_if_enabled()
            self.publish_status(f"TARGET_LOST age={age:.2f}s", sound_request=False)
            return

        robot = self.robot
        dx = target.x - robot.x
        dy = target.y - robot.y
        distance = math.hypot(dx, dy)
        bearing = normalize_angle(math.atan2(dy, dx) - robot.yaw)
        warning_distance = float(self.get_parameter("warning_distance_m").value)
        follow_start = float(self.get_parameter("follow_start_distance_m").value)
        hold_distance = float(self.get_parameter("hold_distance_m").value)
        angular = self.compute_angular(bearing)
        linear = 0.0
        action = "FACE_TARGET"
        sound_request = distance <= warning_distance

        if distance <= hold_distance:
            action = "HOLD_CLOSE_AND_WARN"
            linear = 0.0
        elif distance >= follow_start:
            action = (
                "FOLLOW_LIDAR_COORDINATE_WITH_SOUND"
                if sound_request
                else "FOLLOW_LIDAR_COORDINATE"
            )
            if abs(bearing) <= float(self.get_parameter("rotate_in_place_threshold_rad").value):
                error = distance - hold_distance
                linear = clamp(
                    float(self.get_parameter("linear_gain").value) * error,
                    0.0,
                    float(self.get_parameter("max_linear_speed").value),
                )
                linear *= clamp(1.0 - abs(bearing) / math.pi, 0.25, 1.0)
        elif distance <= warning_distance:
            action = "PROXIMITY_WARN_FACE_TARGET"
            linear = 0.0
        else:
            action = "TRACK_READY_HOLD"

        self.publish_goal(target, robot)
        self.publish_cmd(linear, angular)
        state = (
            f"{action} frame={target.frame_id} target=({target.x:.2f},{target.y:.2f},{target.z:.2f}) "
            f"distance={distance:.2f}m bearing={bearing:.2f}rad cmd=({linear:.2f},{angular:.2f}) "
            f"direct_cmd={self.enable_cmd_vel_output}"
        )
        self.publish_status(state, sound_request=sound_request, distance=distance)

    def compute_angular(self, bearing: float) -> float:
        if abs(bearing) <= float(self.get_parameter("angle_deadband_rad").value):
            return 0.0
        return clamp(
            float(self.get_parameter("angular_gain").value) * bearing,
            -float(self.get_parameter("max_angular_speed").value),
            float(self.get_parameter("max_angular_speed").value),
        )

    def publish_goal(self, target: TargetSample, robot: RobotPose) -> None:
        dx = target.x - robot.x
        dy = target.y - robot.y
        distance = max(math.hypot(dx, dy), 1e-6)
        standoff = min(float(self.get_parameter("goal_standoff_m").value), distance)
        goal_x = target.x - (dx / distance) * standoff
        goal_y = target.y - (dy / distance) * standoff
        yaw = math.atan2(target.y - goal_y, target.x - goal_x)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = target.frame_id
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)

    def publish_cmd(self, linear: float, angular: float) -> None:
        if not self.enable_cmd_vel_output:
            return
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def publish_stop_if_enabled(self) -> None:
        if self.enable_cmd_vel_output and rclpy.ok():
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass

    def publish_status(
        self,
        state: str,
        sound_request: bool,
        distance: float | None = None,
    ) -> None:
        self.state_pub.publish(String(data=state))
        if self.publish_sound_request_enabled:
            self.sound_request_pub.publish(Bool(data=sound_request))

        sound_status = "IDLE_NO_CLOSE_TARGET"
        if sound_request:
            if distance is None:
                sound_status = "SIMULATED_PROXIMITY_SOUND_REQUEST"
            else:
                sound_status = (
                    f"SIMULATED_PROXIMITY_SOUND_REQUEST distance={distance:.2f}m "
                    f"threshold={float(self.get_parameter('warning_distance_m').value):.2f}m"
                )
            if self.emit_terminal_bell:
                now = time.monotonic()
                if now - self.last_bell_time > 1.0:
                    self.last_bell_time = now
                    print("\a", end="", flush=True)
        self.sound_status_pub.publish(String(data=sound_status))

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarTargetFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.publish_stop_if_enabled()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

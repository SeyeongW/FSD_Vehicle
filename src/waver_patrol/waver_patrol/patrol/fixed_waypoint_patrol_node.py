from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from waver_patrol.autonomy_common import (
    clamp,
    load_yaml_file,
    make_twist,
    normalize_angle,
    stop_twist,
    yaw_deg_to_rad,
    yaw_from_quaternion,
)


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    dwell_s: float = 0.0


class FixedWaypointPatrolNode(Node):
    """Repeat fixed waypoints and publish only /waver/cmd_vel_patrol candidates."""

    def __init__(self) -> None:
        super().__init__("fixed_waypoint_patrol_node")
        self.declare_parameter("waypoint_file", "waypoints/waver_bird_patrol_demo.yaml")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/waver/cmd_vel_patrol")
        self.declare_parameter("state_topic", "/waver/patrol_state")
        self.declare_parameter("current_waypoint_topic", "/waver/current_waypoint")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("mode_default", "AUTO")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("loop_count", -1)
        self.declare_parameter("max_linear_speed", 0.14)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("linear_kp", 0.45)
        self.declare_parameter("angular_kp", 1.4)
        self.declare_parameter("xy_tolerance_m", 0.25)
        self.declare_parameter("yaw_tolerance_rad", 0.35)
        self.declare_parameter("heading_gate_rad", 0.75)
        self.declare_parameter("approach_distance_m", 0.8)
        self.declare_parameter("odom_timeout_sec", 0.8)
        self.declare_parameter("timer_hz", 20.0)
        self.declare_parameter("enabled", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop_count = int(self.get_parameter("loop_count").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_kp = float(self.get_parameter("linear_kp").value)
        self.angular_kp = float(self.get_parameter("angular_kp").value)
        self.xy_tolerance = float(self.get_parameter("xy_tolerance_m").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance_rad").value)
        self.heading_gate = float(self.get_parameter("heading_gate_rad").value)
        self.approach_distance = float(self.get_parameter("approach_distance_m").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.mode = str(self.get_parameter("mode_default").value).strip().upper()

        self.waypoints = self._load_waypoints(str(self.get_parameter("waypoint_file").value))
        self.failure_policy = self._load_failure_policy(str(self.get_parameter("waypoint_file").value))
        self.index = 0
        self.completed_loops = 0
        self.pose: tuple[float, float, float] | None = None
        self.last_odom_time = 0.0
        self.dwell_until = 0.0

        self.cmd_pub = self.create_publisher(type(stop_twist()), str(self.get_parameter("cmd_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.wp_pub = self.create_publisher(PointStamped, str(self.get_parameter("current_waypoint_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 20)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), self.mode_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self._publish_state(
            f"READY waypoints={len(self.waypoints)} frame={self.frame_id} "
            "NOTE waypoint frame must match real localization/map/odom origin"
        )

    def _load_waypoints(self, path: str) -> list[Waypoint]:
        data = load_yaml_file(path)
        points = data.get("waypoints", [])
        if not isinstance(points, list):
            points = []
        result: list[Waypoint] = []
        for i, item in enumerate(points):
            if not isinstance(item, dict):
                continue
            result.append(
                Waypoint(
                    name=str(item.get("name", f"wp_{i}")),
                    x=float(item.get("x", 0.0)),
                    y=float(item.get("y", 0.0)),
                    yaw=yaw_deg_to_rad(float(item.get("yaw_deg", 0.0))),
                    dwell_s=float(item.get("dwell_s", 0.0)),
                )
            )
        if not result:
            self.get_logger().warn("No waypoints loaded; fixed patrol will stay stopped")
        return result

    def _load_failure_policy(self, path: str) -> dict:
        try:
            data = load_yaml_file(path)
        except Exception:
            return {}
        policy = data.get("failure_policy", {})
        return policy if isinstance(policy, dict) else {}

    def odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose = (float(pos.x), float(pos.y), yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w))
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9

    def mode_callback(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        self.mode = mode if mode else str(self.get_parameter("mode_default").value).strip().upper()

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.mode in {"EMERGENCY", "DISABLED", "STANDBY", "MANUAL"}:
            self.cmd_pub.publish(stop_twist())
            self._publish_current_waypoint()
            self._publish_state(f"SAFE_IDLE mode={self.mode} publishing_stop_candidate")
            return
        if not self.enabled or not self.waypoints:
            self.cmd_pub.publish(stop_twist())
            self._publish_state("IDLE disabled_or_no_waypoints")
            return
        if self.pose is None or now - self.last_odom_time > self.odom_timeout:
            self.cmd_pub.publish(stop_twist())
            self._publish_state("ODOM_TIMEOUT publishing_stop_candidate")
            return
        if self.loop_count >= 0 and self.completed_loops >= self.loop_count:
            self.cmd_pub.publish(stop_twist())
            self._publish_state("COMPLETE publishing_stop_candidate")
            return
        if now < self.dwell_until:
            self.cmd_pub.publish(stop_twist())
            self._publish_current_waypoint()
            self._publish_state(f"DWELL publishing_stop_candidate policy={self._policy_summary()}")
            return

        target = self.waypoints[self.index]
        x, y, yaw = self.pose
        dx = target.x - x
        dy = target.y - y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)
        yaw_error = normalize_angle(target.yaw - yaw)

        self._publish_current_waypoint()
        if distance <= self.xy_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self._publish_state(f"ARRIVED {target.name}")
            self.dwell_until = now + max(target.dwell_s, 0.0)
            self._advance()
            self.cmd_pub.publish(stop_twist())
            return

        if distance <= self.xy_tolerance:
            linear = 0.0
            angular = clamp(self.angular_kp * yaw_error, -self.max_angular_speed, self.max_angular_speed)
            state = f"ALIGN_FINAL_YAW {target.name}"
        elif abs(heading_error) > self.heading_gate:
            linear = 0.0
            angular = clamp(self.angular_kp * heading_error, -self.max_angular_speed, self.max_angular_speed)
            state = f"ALIGN_TO_WAYPOINT {target.name}"
        else:
            approach_scale = clamp(distance / max(self.approach_distance, 0.1), 0.25, 1.0)
            linear = clamp(self.linear_kp * distance * approach_scale, 0.0, self.max_linear_speed)
            angular = clamp(self.angular_kp * heading_error, -self.max_angular_speed, self.max_angular_speed)
            state = f"GOING_TO_WAYPOINT {target.name} dist={distance:.2f}"
        self._publish_state(state)
        self.cmd_pub.publish(make_twist(linear, angular))

    def _advance(self) -> None:
        self.index += 1
        if self.index >= len(self.waypoints):
            self.index = 0
            self.completed_loops += 1

    def _publish_current_waypoint(self) -> None:
        if not self.waypoints:
            return
        wp = self.waypoints[self.index]
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = wp.x
        msg.point.y = wp.y
        msg.point.z = 0.0
        self.wp_pub.publish(msg)

    def _publish_state(self, text: str) -> None:
        self.state_pub.publish(String(data=text))

    def _policy_summary(self) -> str:
        if not self.failure_policy:
            return "NOT_IMPLEMENTED_BUT_SAFE"
        return "loaded"

    def send_nav2_goal_stub(self, _waypoint: Waypoint) -> bool:
        self.get_logger().debug("Nav2 integration stub is intentionally disabled in this node")
        return False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FixedWaypointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(stop_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

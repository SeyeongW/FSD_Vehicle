from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waver_patrol.autonomy_common import clamp, make_twist, normalize_angle, stop_twist, yaw_from_quaternion


class ReturnHomeControllerNode(Node):
    """Return-home candidate command generator. It never publishes /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("return_home_controller_node")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("return_home_active_topic", "/waver/return_home_active")
        self.declare_parameter("return_goal_topic", "/waver/return_goal")
        self.declare_parameter("cmd_topic", "/waver/cmd_vel_return_home")
        self.declare_parameter("state_topic", "/waver/return_home_state")
        self.declare_parameter("arrived_topic", "/waver/return_home_arrived")
        self.declare_parameter("max_linear_speed", 0.12)
        self.declare_parameter("max_angular_speed", 0.4)
        self.declare_parameter("linear_kp", 0.4)
        self.declare_parameter("angular_kp", 1.2)
        self.declare_parameter("xy_tolerance_m", 0.25)
        self.declare_parameter("yaw_tolerance_rad", 0.4)
        self.declare_parameter("heading_gate_rad", 0.75)
        self.declare_parameter("odom_timeout_sec", 0.8)
        self.declare_parameter("timer_hz", 20.0)

        self.pose: tuple[float, float, float] | None = None
        self.last_odom_time = 0.0
        self.active = False
        self.arrived = False
        self.goal: PoseStamped | None = None

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.arrived_pub = self.create_publisher(Bool, str(self.get_parameter("arrived_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 20)
        self.create_subscription(Bool, str(self.get_parameter("return_home_active_topic").value), self.active_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("return_goal_topic").value), self.goal_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose = (float(pos.x), float(pos.y), yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w))
        self.last_odom_time = self._now()

    def active_callback(self, msg: Bool) -> None:
        self.active = bool(msg.data)
        if not self.active:
            self.arrived = False

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg

    def tick(self) -> None:
        if not self.active:
            self._publish("IDLE", stop_twist(), False)
            return
        if self.arrived:
            self._publish("HOLD_AT_HOME", stop_twist(), True)
            return
        if self.goal is None:
            self._publish("WAITING_FOR_RETURN_GOAL", stop_twist(), False)
            return
        if self.pose is None or self._now() - self.last_odom_time > float(self.get_parameter("odom_timeout_sec").value):
            self._publish("ODOM_TIMEOUT", stop_twist(), False)
            return

        x, y, yaw = self.pose
        gx = float(self.goal.pose.position.x)
        gy = float(self.goal.pose.position.y)
        gori = self.goal.pose.orientation
        gyaw = yaw_from_quaternion(gori.x, gori.y, gori.z, gori.w)
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
        yaw_error = normalize_angle(gyaw - yaw)

        if distance <= float(self.get_parameter("xy_tolerance_m").value) and abs(yaw_error) <= float(self.get_parameter("yaw_tolerance_rad").value):
            self.arrived = True
            self._publish("ARRIVED_HOME", stop_twist(), True)
            return
        if distance <= float(self.get_parameter("xy_tolerance_m").value):
            cmd = make_twist(0.0, clamp(float(self.get_parameter("angular_kp").value) * yaw_error, -self._max_ang(), self._max_ang()))
            self._publish("ALIGN_HOME_YAW", cmd, False)
            return
        if abs(heading_error) > float(self.get_parameter("heading_gate_rad").value):
            cmd = make_twist(0.0, clamp(float(self.get_parameter("angular_kp").value) * heading_error, -self._max_ang(), self._max_ang()))
            self._publish("ALIGN_TO_HOME", cmd, False)
            return
        linear = clamp(float(self.get_parameter("linear_kp").value) * distance, 0.0, self._max_lin())
        angular = clamp(float(self.get_parameter("angular_kp").value) * heading_error, -self._max_ang(), self._max_ang())
        self._publish(f"RETURNING_HOME dist={distance:.2f}", make_twist(linear, angular), False)

    def _publish(self, state: str, cmd: Twist, arrived: bool) -> None:
        self.state_pub.publish(String(data=state))
        self.cmd_pub.publish(cmd)
        self.arrived_pub.publish(Bool(data=arrived))

    def send_nav2_return_goal_stub(self, _goal: PoseStamped) -> bool:
        self.get_logger().debug("Nav2 return-home stub intentionally disabled")
        return False

    def _max_lin(self) -> float:
        return float(self.get_parameter("max_linear_speed").value)

    def _max_ang(self) -> float:
        return float(self.get_parameter("max_angular_speed").value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ReturnHomeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(stop_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

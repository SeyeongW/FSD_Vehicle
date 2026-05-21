from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from waver_patrol.autonomy_common import quaternion_from_yaw


class SimpleSimOdomNode(Node):
    """Desk-test odom simulator. Do not use while real motors are enabled."""

    def __init__(self) -> None:
        super().__init__("simple_sim_odom_node")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("timer_hz", 30.0)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.last_time = self._now()
        self.pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), lambda m: setattr(self, "cmd", m), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("simple_sim_odom_node is for desk tests only; never use it on the real robot")

    def tick(self) -> None:
        now = self._now()
        dt = max(0.0, min(0.1, now - self.last_time))
        self.last_time = now
        self.yaw += float(self.cmd.angular.z) * dt
        self.x += float(self.cmd.linear.x) * math.cos(self.yaw) * dt
        self.y += float(self.cmd.linear.x) * math.sin(self.yaw) * dt
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.child_frame_id = str(self.get_parameter("child_frame_id").value)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = quaternion_from_yaw(self.yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        self.pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleSimOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

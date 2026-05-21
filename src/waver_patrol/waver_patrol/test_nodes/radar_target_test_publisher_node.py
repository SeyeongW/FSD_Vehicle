from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RadarTargetTestPublisherNode(Node):
    """Desk/Gazebo-only fake radar target publisher. Do not use during real driving."""

    def __init__(self) -> None:
        super().__init__("radar_target_test_publisher_node")
        self.declare_parameter("topic", "/radar4d/target_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 2.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.6)
        self.declare_parameter("enable_motion", True)
        self.declare_parameter("motion_radius_m", 0.4)
        self.declare_parameter("motion_angular_rate_radps", 0.5)
        self.declare_parameter("period_sec", 2.0)
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter("topic").value), 10)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(float(self.get_parameter("period_sec").value), self.tick)

    def tick(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        if bool(self.get_parameter("enable_motion").value):
            t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            radius = float(self.get_parameter("motion_radius_m").value)
            rate = float(self.get_parameter("motion_angular_rate_radps").value)
            x += radius * math.cos(rate * t)
            y += radius * math.sin(rate * t)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = float(self.get_parameter("z").value)
        yaw = math.atan2(msg.pose.position.y, msg.pose.position.x)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RadarTargetTestPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

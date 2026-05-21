from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class AerialTargetTestPublisherNode(Node):
    """Fake moving aerial point publisher. Never use during real driving."""

    def __init__(self) -> None:
        super().__init__("aerial_target_test_publisher_node")
        self.declare_parameter("topic", "/waver/object_point")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("timer_hz", 5.0)
        self.t = 0.0
        self.pub = self.create_publisher(PointStamped, str(self.get_parameter("topic").value), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("aerial_target_test_publisher_node is for simulation/test only")

    def tick(self) -> None:
        self.t += 0.2
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        # ROS base_link convention: x forward, y left, z up.
        msg.point.x = 3.0
        msg.point.y = 0.6 * math.sin(self.t)
        msg.point.z = 1.2
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AerialTargetTestPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

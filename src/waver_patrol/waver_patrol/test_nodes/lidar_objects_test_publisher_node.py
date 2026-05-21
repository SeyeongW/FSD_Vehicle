from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class LidarObjectsTestPublisherNode(Node):
    """Fake PoseArray object candidates. Never use during real driving."""

    def __init__(self) -> None:
        super().__init__("lidar_objects_test_publisher_node")
        self.declare_parameter("topic", "/waver/lidar_objects")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("timer_hz", 5.0)
        self.t = 0.0
        self.pub = self.create_publisher(PoseArray, str(self.get_parameter("topic").value), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("lidar_objects_test_publisher_node is for simulation/test only")

    def tick(self) -> None:
        self.t += 0.2
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        pose = Pose()
        # ROS base_link convention: x forward, y left, z up.
        pose.position.x = 2.5
        pose.position.y = 0.8 * math.sin(self.t)
        pose.position.z = 1.0
        msg.poses.append(pose)
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarObjectsTestPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

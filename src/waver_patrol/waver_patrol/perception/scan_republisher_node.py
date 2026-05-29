from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRepublisherNode(Node):
    """Republish a LaserScan to a named role topic.

    Gazebo's rover SDF publishes a single dense laser scan on `/scan`. Mapping
    debug keeps that raw topic intact and mirrors it to `/scan_slam` and
    `/scan_safety` so SLAM and safety checks can audit separate authorities.
    """

    def __init__(self) -> None:
        super().__init__("scan_republisher_node")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.pub = self.create_publisher(LaserScan, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(LaserScan, str(self.get_parameter("input_topic").value), self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRepublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

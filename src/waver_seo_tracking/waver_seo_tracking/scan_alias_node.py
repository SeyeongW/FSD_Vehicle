from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanAliasNode(Node):
    """Republish the Gazebo laser scan into explicit SLAM/safety scan topics."""

    def __init__(self) -> None:
        super().__init__("scan_alias_node")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("slam_topic", "/scan_slam")
        self.declare_parameter("safety_topic", "/scan_safety")
        self.scan_pub = self.create_publisher(LaserScan, str(self.get_parameter("slam_topic").value), qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(LaserScan, str(self.get_parameter("safety_topic").value), qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, "/waver/scan_alias_state", 10)
        self.count = 0
        self.create_subscription(LaserScan, str(self.get_parameter("input_topic").value), self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, msg: LaserScan) -> None:
        self.count += 1
        self.scan_pub.publish(msg)
        self.safety_pub.publish(msg)
        if self.count % 20 == 1:
            self.state_pub.publish(
                String(
                    data=(
                        f"SCAN_ALIAS_OK input={self.get_parameter('input_topic').value} "
                        f"slam={self.get_parameter('slam_topic').value} safety={self.get_parameter('safety_topic').value}"
                    )
                )
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanAliasNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class BatteryTestPublisherNode(Node):
    """Fake critical battery publisher. Never use during real driving."""

    def __init__(self) -> None:
        super().__init__("battery_test_publisher_node")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("voltage_topic", "/voltage")
        self.declare_parameter("percentage", 0.15)
        self.declare_parameter("voltage", 9.2)
        self.declare_parameter("timer_hz", 1.0)
        self.battery_pub = self.create_publisher(BatteryState, str(self.get_parameter("battery_state_topic").value), 10)
        self.voltage_pub = self.create_publisher(Float32, str(self.get_parameter("voltage_topic").value), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 0.2), self.tick)
        self.get_logger().warn("battery_test_publisher_node is for simulation/test only")

    def tick(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.get_parameter("percentage").value)
        msg.voltage = float(self.get_parameter("voltage").value)
        self.battery_pub.publish(msg)
        self.voltage_pub.publish(Float32(data=msg.voltage))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BatteryTestPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

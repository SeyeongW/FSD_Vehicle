from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class FakeCameraClassificationTestPublisherNode(Node):
    """Desk/Gazebo-only fake camera classifier. Never enable during real driving."""

    def __init__(self) -> None:
        super().__init__("fake_camera_classification_test_publisher_node")
        self.declare_parameter("class_topic", "/waver/external_target_class")
        self.declare_parameter("confidence_topic", "/waver/external_target_confidence")
        self.declare_parameter("target_class", "bird")
        self.declare_parameter("confidence", 0.92)
        self.declare_parameter("publish_after_sec", 6.0)
        self.declare_parameter("publish_active", True)
        self.declare_parameter("active_topic", "/waver/aerial_target_active")
        self.class_pub = self.create_publisher(String, str(self.get_parameter("class_topic").value), 10)
        self.conf_pub = self.create_publisher(Float32, str(self.get_parameter("confidence_topic").value), 10)
        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("active_topic").value), 10)
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self.tick)

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start < float(self.get_parameter("publish_after_sec").value):
            return
        if bool(self.get_parameter("publish_active").value):
            # 역할: Gazebo/desk test에서 카메라가 목표를 다시 잡은 상황을 흉내 낸다.
            # 실차 launch에서는 이 test publisher를 켜지 않는다.
            self.active_pub.publish(Bool(data=True))
        self.class_pub.publish(String(data=str(self.get_parameter("target_class").value)))
        self.conf_pub.publish(Float32(data=float(self.get_parameter("confidence").value)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeCameraClassificationTestPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class DeepLearningBridgeStub(Node):
    """Stub for future detector output normalization. No model is loaded here."""

    def __init__(self) -> None:
        super().__init__("deep_learning_bridge_stub")
        self.declare_parameter("external_class_topic", "/waver/external_target_class")
        self.declare_parameter("external_confidence_topic", "/waver/external_target_confidence")
        self.declare_parameter("camera_detection_state_topic", "/waver/camera_detection_state")
        self.declare_parameter("aerial_target_active_topic", "/waver/aerial_target_active")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("classification_state_topic", "/waver/classification_state")
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("bird_class_names", ["bird", "Bird", "BIRD"])
        self.declare_parameter("blocked_sound_classes", ["airplane", "drone", "human", "vehicle", "person", "car"])
        self.declare_parameter("enable_test_classification", False)
        self.declare_parameter("test_mode", False)
        self.declare_parameter("timer_hz", 5.0)
        self.latest_class = "unknown"
        self.latest_confidence = 0.0
        self.camera_state = "UNKNOWN"
        self.target_active = False
        self.class_pub = self.create_publisher(String, str(self.get_parameter("target_class_topic").value), 10)
        self.conf_pub = self.create_publisher(Float32, str(self.get_parameter("target_confidence_topic").value), 10)
        self.bird_pub = self.create_publisher(Bool, str(self.get_parameter("bird_confirmed_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("classification_state_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("external_class_topic").value), self.class_callback, 10)
        self.create_subscription(Float32, str(self.get_parameter("external_confidence_topic").value), self.confidence_callback, 10)
        self.create_subscription(String, str(self.get_parameter("camera_detection_state_topic").value), self.camera_state_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("aerial_target_active_topic").value), self.active_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def class_callback(self, msg: String) -> None:
        self.latest_class = msg.data

    def confidence_callback(self, msg: Float32) -> None:
        self.latest_confidence = float(msg.data)

    def active_callback(self, msg: Bool) -> None:
        self.target_active = bool(msg.data)

    def camera_state_callback(self, msg: String) -> None:
        self.camera_state = msg.data

    def tick(self) -> None:
        if bool(self.get_parameter("test_mode").value) or bool(self.get_parameter("enable_test_classification").value):
            self.latest_class = "bird"
            self.latest_confidence = 0.9
        class_names = {str(name) for name in self.get_parameter("bird_class_names").value}
        blocked = {str(name).lower() for name in self.get_parameter("blocked_sound_classes").value}
        cls = str(self.latest_class)
        confirmed = (
            self.target_active
            and cls in class_names
            and self.latest_confidence >= float(self.get_parameter("confidence_threshold").value)
        )
        self.class_pub.publish(String(data=self.latest_class))
        self.conf_pub.publish(Float32(data=float(self.latest_confidence)))
        self.bird_pub.publish(Bool(data=confirmed))
        if not self.target_active:
            state = "NO_ACTIVE_TARGET bird_confirmed=false"
        elif cls.lower() in blocked:
            state = f"CLASS_BLOCKS_SOUND class={cls} confidence={self.latest_confidence:.3f}"
        elif confirmed:
            state = f"BIRD_CONFIRMED class={cls} confidence={self.latest_confidence:.3f} camera_state={self.camera_state}"
        else:
            state = f"CLASSIFICATION_WAIT_OR_LOW_CONFIDENCE class={cls} confidence={self.latest_confidence:.3f}"
        self.state_pub.publish(String(data=state))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DeepLearningBridgeStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.bird_pub.publish(Bool(data=False))
            node.state_pub.publish(String(data="SHUTDOWN"))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

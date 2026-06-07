from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class SeoBirdYoloNode(Node):
    """Gazebo demo bird classifier.

    This is explicitly a Gazebo fake/YOLO placeholder. It does not run in real
    profile. It publishes the same mission-facing classification topics that a
    YOLO11s adapter will publish after model integration.
    """

    def __init__(self) -> None:
        super().__init__("seo_bird_yolo_node")
        self.declare_parameter("lock_topic", "/waver/dynamic_object_lock")
        self.declare_parameter("camera_aim_request_topic", "/waver/camera_aim_request")
        self.declare_parameter("confidence", 0.92)
        self.declare_parameter("confirm_delay_sec", 0.5)
        self.declare_parameter("state_topic", "/waver/classification_state")
        self.locked = False
        self.aim_requested = False
        self.lock_time = 0.0
        self.class_pub = self.create_publisher(String, "/waver/target_class", 10)
        self.conf_pub = self.create_publisher(Float32, "/waver/target_confidence", 10)
        self.confirmed_pub = self.create_publisher(Bool, "/waver/bird_confirmed", 10)
        self.detected_pub = self.create_publisher(Bool, "/bird_detected", 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(Bool, str(self.get_parameter("lock_topic").value), self.lock_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("camera_aim_request_topic").value), lambda m: setattr(self, "aim_requested", bool(m.data)), 10)
        self.create_timer(0.1, self.tick)

    def lock_callback(self, msg: Bool) -> None:
        locked = bool(msg.data)
        if locked and not self.locked:
            self.lock_time = time.monotonic()
        self.locked = locked

    def tick(self) -> None:
        ready = (
            (self.locked or self.aim_requested)
            and (time.monotonic() - self.lock_time >= float(self.get_parameter("confirm_delay_sec").value))
        )
        if ready:
            self.class_pub.publish(String(data="bird"))
            self.conf_pub.publish(Float32(data=float(self.get_parameter("confidence").value)))
            self.confirmed_pub.publish(Bool(data=True))
            self.detected_pub.publish(Bool(data=True))
            source = "GAZEBO_FAKE_YOLO11S_CLASSIFICATION"
            if self.aim_requested:
                source += " camera_alignment_requested=true"
            self.state_pub.publish(String(data=f"{source} class=bird"))
            return
        self.class_pub.publish(String(data="unknown"))
        self.conf_pub.publish(Float32(data=0.0))
        self.confirmed_pub.publish(Bool(data=False))
        self.detected_pub.publish(Bool(data=False))
        self.state_pub.publish(String(data="WAITING_DYNAMIC_LOCK"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeoBirdYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

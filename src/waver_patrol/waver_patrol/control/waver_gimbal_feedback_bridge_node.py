from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray


class WaverGimbalFeedbackBridgeNode(Node):
    """Bridge camera gimbal command/feedback without enabling real output by default."""

    def __init__(self) -> None:
        super().__init__("waver_gimbal_feedback_bridge_node")
        self.declare_parameter("cmd_topic", "/waver/camera_gimbal_cmd")
        self.declare_parameter("external_feedback_topic", "/external/gimbal/feedback")
        self.declare_parameter("detections_topic", "/waver/bird_detections_2d")
        self.declare_parameter("real_gimbal_output_enabled", False)
        self.declare_parameter("allow_simulated_centered", False)
        self.declare_parameter("max_pan_rad", 1.57)
        self.declare_parameter("min_pan_rad", -1.57)
        self.declare_parameter("max_tilt_rad", 0.8)
        self.declare_parameter("min_tilt_rad", -0.5)
        self.declare_parameter("command_timeout_sec", 0.8)

        self.last_cmd = Vector3()
        self.last_cmd_time = 0.0
        self.feedback = Vector3()
        self.feedback_valid = False

        self.feedback_pub = self.create_publisher(Vector3, "/waver/camera_gimbal_feedback", 10)
        self.centered_pub = self.create_publisher(Bool, "/waver/camera_target_centered", 10)
        self.state_pub = self.create_publisher(String, "/waver/camera_alignment_state", 10)
        self.create_subscription(Vector3, str(self.get_parameter("cmd_topic").value), self.cmd_callback, 10)
        self.create_subscription(Vector3, str(self.get_parameter("external_feedback_topic").value), self.feedback_callback, 10)
        self.create_subscription(Detection2DArray, str(self.get_parameter("detections_topic").value), self.detections_callback, 10)
        self.create_timer(0.1, self.tick)

    def cmd_callback(self, msg: Vector3) -> None:
        self.last_cmd = Vector3(
            x=self.clamp(float(msg.x), float(self.get_parameter("min_pan_rad").value), float(self.get_parameter("max_pan_rad").value)),
            y=self.clamp(float(msg.y), float(self.get_parameter("min_tilt_rad").value), float(self.get_parameter("max_tilt_rad").value)),
            z=float(msg.z),
        )
        self.last_cmd_time = self._now()

    def feedback_callback(self, msg: Vector3) -> None:
        self.feedback = msg
        self.feedback_valid = float(msg.z) > 0.5

    def detections_callback(self, _msg: Detection2DArray) -> None:
        # Future hook: bbox center error can be used here without changing topics.
        return

    def tick(self) -> None:
        age = self._now() - self.last_cmd_time if self.last_cmd_time else math.inf
        if age > float(self.get_parameter("command_timeout_sec").value):
            self.publish(False, "COMMAND_TIMEOUT", self.feedback if self.feedback_valid else Vector3(z=0.0))
            return
        if self.feedback_valid:
            pan_error = abs(float(self.feedback.x) - float(self.last_cmd.x))
            tilt_error = abs(float(self.feedback.y) - float(self.last_cmd.y))
            centered = pan_error < 0.05 and tilt_error < 0.05
            self.publish(centered, f"FEEDBACK {'CENTERED' if centered else 'TRACKING'} pan_error={pan_error:.3f} tilt_error={tilt_error:.3f}", self.feedback)
            return
        if bool(self.get_parameter("allow_simulated_centered").value) and not bool(self.get_parameter("real_gimbal_output_enabled").value):
            self.publish(True, "SIM_ONLY_CENTERED", Vector3(x=self.last_cmd.x, y=self.last_cmd.y, z=1.0))
            return
        self.publish(False, "WAIT_GIMBAL_FEEDBACK", Vector3(x=self.last_cmd.x, y=self.last_cmd.y, z=0.0))

    def publish(self, centered: bool, state: str, feedback: Vector3) -> None:
        self.feedback_pub.publish(feedback)
        self.centered_pub.publish(Bool(data=bool(centered)))
        self.state_pub.publish(String(data=state))

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaverGimbalFeedbackBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

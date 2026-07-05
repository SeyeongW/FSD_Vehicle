from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class BirdMissionSupervisorNode(Node):
    """Fail-closed supervisor for the production bird mission stack."""

    def __init__(self) -> None:
        super().__init__("bird_mission_supervisor_node")
        self.declare_parameter("state_topic", "/waver/bird_mission_supervisor_state")
        self.declare_parameter("zero_cmd_topic", "/waver/manual_cmd_vel")
        self.declare_parameter("critical_stale_sec", 2.0)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.sound_cancel_pub = self.create_publisher(Bool, "/waver/sound_alert_request", 10)
        self.mission_cancel_pub = self.create_publisher(String, "/waver/mission_command", 10)
        self.zero_pub = self.create_publisher(Twist, str(self.get_parameter("zero_cmd_topic").value), 10)
        self.last_safety_time = time.monotonic()
        self.last_fusion_time = 0.0
        self.emergency_stop = False
        self.external_stop = False
        self.create_subscription(String, "/waver/safety_state", self.safety_callback, 10)
        self.create_subscription(String, "/waver/bird_fusion_state", self.fusion_callback, 10)
        self.create_subscription(Bool, "/waver/emergency_stop", lambda m: setattr(self, "emergency_stop", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/external_stop", lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_timer(0.2, self.tick)

    def safety_callback(self, msg: String) -> None:
        self.last_safety_time = time.monotonic()
        text = msg.data.upper()
        if "FATAL" in text or "EMERGENCY" in text:
            self.emergency_stop = True

    def fusion_callback(self, msg: String) -> None:
        self.last_fusion_time = time.monotonic()

    def tick(self) -> None:
        now = time.monotonic()
        safety_stale = now - self.last_safety_time > float(self.get_parameter("critical_stale_sec").value)
        if self.emergency_stop or self.external_stop or safety_stale:
            self.sound_cancel_pub.publish(Bool(data=False))
            self.mission_cancel_pub.publish(String(data="STOP"))
            self.zero_pub.publish(Twist())
            reason = "external_stop" if self.external_stop else "emergency_stop" if self.emergency_stop else "safety_state_stale"
            self.state_pub.publish(String(data=f"SUPERVISOR_SAFE_STOP_SENT reason={reason}"))
            return
        if self.last_fusion_time and now - self.last_fusion_time > 2.0:
            self.state_pub.publish(String(data="SUPERVISOR_DEGRADED fusion_state_stale=true"))
        else:
            self.state_pub.publish(String(data="SUPERVISOR_OK"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BirdMissionSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

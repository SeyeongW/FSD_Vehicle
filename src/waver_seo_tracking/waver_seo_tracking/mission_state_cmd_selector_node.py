from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def normalize_mission_state(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return "UNKNOWN"
    return text.split()[0].strip().upper()


class MissionStateCmdSelectorNode(Node):
    """Select nav or target-tracking candidate command before safety mux."""

    TRACK_STATES = {
        "CAMERA_ALIGN_TO_TARGET",
        "CAMERA_ALIGN_DONE",
        "TARGET_CLASSIFICATION_WAIT",
        "SOUND_TASK_REQUESTED",
        "SOUND_TASK_RUNNING",
    }

    def __init__(self) -> None:
        super().__init__("mission_state_cmd_selector_node")
        self.declare_parameter("nav_cmd_topic", "/waver/cmd_vel_nav2")
        self.declare_parameter("track_cmd_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("output_topic", "/waver/cmd_vel_auto_selected")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.nav_cmd = Twist()
        self.track_cmd = Twist()
        self.nav_time = 0.0
        self.track_time = 0.0
        self.mission_state = "IDLE"
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_topic").value), 10)
        self.state_pub = self.create_publisher(String, "/waver/cmd_selector_state", 10)
        self.create_subscription(Twist, str(self.get_parameter("nav_cmd_topic").value), self.nav_callback, 10)
        self.create_subscription(Twist, str(self.get_parameter("track_cmd_topic").value), self.track_callback, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            lambda m: setattr(self, "mission_state", normalize_mission_state(m.data)),
            10,
        )
        self.create_timer(0.05, self.tick)

    def nav_callback(self, msg: Twist) -> None:
        self.nav_cmd = msg
        self.nav_time = self._now()

    def track_callback(self, msg: Twist) -> None:
        self.track_cmd = msg
        self.track_time = self._now()

    def tick(self) -> None:
        now = self._now()
        timeout = float(self.get_parameter("cmd_timeout_sec").value)
        use_track = self.mission_state in self.TRACK_STATES and now - self.track_time <= timeout
        use_nav = now - self.nav_time <= timeout
        if use_track:
            self.cmd_pub.publish(self.track_cmd)
            self.state_pub.publish(String(data=f"TRACK_CMD_SELECTED mission_state={self.mission_state}"))
        elif use_nav:
            self.cmd_pub.publish(self.nav_cmd)
            self.state_pub.publish(String(data=f"NAV_CMD_SELECTED mission_state={self.mission_state}"))
        else:
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data=f"ZERO_CMD_SELECTED mission_state={self.mission_state}"))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionStateCmdSelectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

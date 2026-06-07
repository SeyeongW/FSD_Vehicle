from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class RemoteUiPatrolAdapterNode(Node):
    """Gazebo-only observer for remote UI patrol commands.

    The current Waver UI already publishes /waver/mission_command. This adapter
    intentionally does not republish by default, avoiding command loops while
    leaving a visible state topic for launch verification.
    """

    def __init__(self) -> None:
        super().__init__("remote_ui_patrol_adapter_node")
        self.declare_parameter("mission_command_topic", "/waver/mission_command")
        self.state_pub = self.create_publisher(String, "/waver/remote_ui_patrol_adapter_state", 10)
        self.create_subscription(String, str(self.get_parameter("mission_command_topic").value), self.command_callback, 10)

    def command_callback(self, msg: String) -> None:
        self.state_pub.publish(String(data=f"UI_COMMAND_OBSERVED command={msg.data.strip().upper()}"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RemoteUiPatrolAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

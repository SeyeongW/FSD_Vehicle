from __future__ import annotations

from waver_patrol.teleop.keyboard_curses import KeyboardTeleopState, run_curses


def main(args: list[str] | None = None) -> None:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
    except ImportError:
        raise RuntimeError("teleop_node requires ROS2; use teleop_keyboard for serial fallback")

    class TeleopPublisher(Node):
        def __init__(self) -> None:
            super().__init__("waver_teleop_node")
            self.publisher = self.create_publisher(Twist, "/waver/manual_cmd_vel", 10)

        def send(self, cmd) -> None:
            msg = Twist()
            msg.linear.x = (cmd.left + cmd.right) / 2.0
            msg.angular.z = (cmd.right - cmd.left) / 2.0
            self.publisher.publish(msg)

    rclpy.init(args=args)
    node = TeleopPublisher()
    try:
        run_curses(node.send, KeyboardTeleopState())
    finally:
        node.send(type("Stop", (), {"left": 0.0, "right": 0.0})())
        node.destroy_node()
        rclpy.shutdown()

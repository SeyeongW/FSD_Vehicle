from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class ObjectMissionGoalTestPublisherNode(Node):
    """Desk/Gazebo-only object mission goal publisher. Never enable on real robot."""

    def __init__(self) -> None:
        super().__init__("object_mission_goal_test_publisher_node")
        self.declare_parameter("goal_topic", "/waver/object_mission_goal")
        self.declare_parameter("active_topic", "/waver/object_mission_goal_active")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 2.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("publish_once_after_sec", 3.0)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("active_topic").value), 10)
        self.sent = False
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.2, self.tick)

    def tick(self) -> None:
        if self.sent:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start < float(self.get_parameter("publish_once_after_sec").value):
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.position.y = float(self.get_parameter("y").value)
        msg.pose.position.z = float(self.get_parameter("z").value)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.active_pub.publish(Bool(data=True))
        self.sent = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObjectMissionGoalTestPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

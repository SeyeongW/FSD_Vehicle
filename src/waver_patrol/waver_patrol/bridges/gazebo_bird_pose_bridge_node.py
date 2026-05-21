from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waver_patrol.autonomy_common import yaw_from_quaternion


class GazeboBirdPoseBridgeNode(Node):
    """Convert ugv_gazebo bird pose into Waver's aerial object interface.

    Role:
      - Subscribe to `bird_manager.py` outputs used by the Gazebo world.
      - Convert world/odom bird pose into a base-relative PointStamped.
      - Publish `/waver/object_point` so the normal aerial motion detector and
        target tracker are tested without letting Gazebo scripts publish `/cmd_vel`.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_bird_pose_bridge_node")
        self.declare_parameter("bird_pose_topic", "/bird/nearest_pose")
        self.declare_parameter("bird_visible_topic", "/bird/visible")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("object_point_topic", "/waver/object_point")
        self.declare_parameter("state_topic", "/waver/gazebo_bird_bridge_state")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_timeout_sec", 0.8)
        self.declare_parameter("bird_timeout_sec", 0.8)
        self.declare_parameter("timer_hz", 10.0)

        self.visible = False
        self.last_bird: PoseStamped | None = None
        self.last_bird_time = 0.0
        self.robot_pose: tuple[float, float, float, float] | None = None
        self.last_odom_time = 0.0

        self.object_pub = self.create_publisher(PointStamped, str(self.get_parameter("object_point_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("bird_pose_topic").value), self.bird_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("bird_visible_topic").value), self.visible_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 20)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def visible_callback(self, msg: Bool) -> None:
        self.visible = bool(msg.data)

    def bird_callback(self, msg: PoseStamped) -> None:
        self.last_bird = msg
        self.last_bird_time = self._now()

    def odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        self.robot_pose = (float(pos.x), float(pos.y), float(pos.z), yaw)
        self.last_odom_time = self._now()

    def tick(self) -> None:
        now = self._now()
        if not self.visible:
            self.state_pub.publish(String(data="NO_BIRD_VISIBLE"))
            return
        if self.last_bird is None or now - self.last_bird_time > float(self.get_parameter("bird_timeout_sec").value):
            self.state_pub.publish(String(data="BIRD_POSE_STALE"))
            return
        if self.robot_pose is None or now - self.last_odom_time > float(self.get_parameter("odom_timeout_sec").value):
            self.state_pub.publish(String(data="ODOM_STALE"))
            return

        rx, ry, rz, yaw = self.robot_pose
        bird = self.last_bird.pose.position
        dx = float(bird.x) - rx
        dy = float(bird.y) - ry
        dz = float(bird.z) - rz

        # ROS base_link convention: x forward, y left, z up.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("base_frame_id").value)
        msg.point.x = forward
        msg.point.y = left
        msg.point.z = dz
        self.object_pub.publish(msg)
        self.state_pub.publish(String(data=f"PUBLISHED_OBJECT_POINT x_forward={forward:.2f} y_left={left:.2f} z={dz:.2f}"))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboBirdPoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

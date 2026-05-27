from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class MappingPathPublisherNode(Node):
    """Publish the robot trace used by the operator panel during live mapping."""

    def __init__(self) -> None:
        super().__init__("mapping_path_publisher_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mapping_active_topic", "/waver/mapping_active")
        self.declare_parameter("path_topic", "/waver/mapping_path")
        self.declare_parameter("pose_topic", "/waver/current_pose")
        self.declare_parameter("state_topic", "/waver/mapping_path_state")
        self.declare_parameter("publish_hz", 4.0)
        self.declare_parameter("append_distance_m", 0.06)
        self.declare_parameter("max_path_points", 2500)
        self.declare_parameter("lookup_timeout_sec", 0.08)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.active = False
        self.path = Path()
        self.last_xy: tuple[float, float] | None = None

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("pose_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("mapping_active_topic").value),
            self.mapping_active_callback,
            10,
        )
        hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self.tick)

    def mapping_active_callback(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self.active:
            self.path = Path()
            self.last_xy = None
        self.active = active

    def tick(self) -> None:
        if not self.active:
            return
        pose = self.lookup_pose()
        if pose is None:
            return
        self.pose_pub.publish(pose)
        xy = (float(pose.pose.position.x), float(pose.pose.position.y))
        append_distance = max(0.0, float(self.get_parameter("append_distance_m").value))
        if self.last_xy is None or math.dist(self.last_xy, xy) >= append_distance:
            self.path.header = pose.header
            self.path.poses.append(pose)
            max_points = max(1, int(self.get_parameter("max_path_points").value))
            if len(self.path.poses) > max_points:
                self.path.poses = self.path.poses[-max_points:]
            self.last_xy = xy
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        self.state_pub.publish(
            String(data=f"OK frame={self.path.header.frame_id} points={len(self.path.poses)}")
        )

    def lookup_pose(self) -> PoseStamped | None:
        base_frame = str(self.get_parameter("base_frame").value)
        timeout = Duration(seconds=max(0.0, float(self.get_parameter("lookup_timeout_sec").value)))
        for frame in (str(self.get_parameter("map_frame").value), str(self.get_parameter("odom_frame").value)):
            try:
                transform = self.tf_buffer.lookup_transform(frame, base_frame, rclpy.time.Time(), timeout=timeout)
            except TransformException:
                continue
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame
            pose.pose.position.x = float(transform.transform.translation.x)
            pose.pose.position.y = float(transform.transform.translation.y)
            pose.pose.position.z = float(transform.transform.translation.z)
            pose.pose.orientation = transform.transform.rotation
            return pose
        self.state_pub.publish(String(data=f"TF_FAIL base={base_frame}"))
        return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingPathPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

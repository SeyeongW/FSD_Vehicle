from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


class SeoCameraTiltJointNode(Node):
    """Publish logical camera alignment state, optionally with Gazebo joint commands."""

    def __init__(self) -> None:
        super().__init__("seo_camera_tilt_joint_node")
        self.declare_parameter("target_topic", "/waver/lidar_target_pose_base")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("transform_timeout_sec", 0.05)
        self.declare_parameter("joint_topic", "/set_joint_trajectory")
        self.declare_parameter("joint_name", "pt_link1_to_pt_link2")
        self.declare_parameter("tilt_axis_sign", -1.0)
        self.declare_parameter("min_tilt_rad", -0.7)
        self.declare_parameter("max_tilt_rad", 0.7)
        self.declare_parameter("center_tolerance_rad", 0.15)
        self.declare_parameter("alignment_mode", "logical")
        self.declare_parameter("publish_joint_trajectory", False)
        self.target: PoseStamped | None = None
        self.request_active = False
        self.tf_buffer = Buffer() if Buffer is not None else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.tf_buffer is not None else None
        self.joint_pub = self.create_publisher(JointTrajectory, str(self.get_parameter("joint_topic").value), 10)
        self.centered_pub = self.create_publisher(Bool, "/waver/camera_target_centered", 10)
        self.state_pub = self.create_publisher(String, "/waver/camera_alignment_state", 10)
        self.bearing_pub = self.create_publisher(Float32, "/bird_visual_bearing", 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("target_topic").value), self.target_callback, 10)
        self.create_subscription(Bool, "/waver/camera_aim_request", lambda m: setattr(self, "request_active", bool(m.data)), 10)
        self.create_timer(0.1, self.tick)

    def target_callback(self, msg: PoseStamped) -> None:
        self.target = msg

    def tick(self) -> None:
        if self.target is None or not self.request_active:
            self.centered_pub.publish(Bool(data=False))
            return
        target = self.target_in_base()
        if target is None:
            self.centered_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data="CAMERA_ALIGNMENT TF_FAIL centered=false"))
            return
        x = float(target.pose.position.x)
        y = float(target.pose.position.y)
        z = float(target.pose.position.z)
        bearing = math.atan2(y, x)
        elevation = math.atan2(z, max(math.hypot(x, y), 1e-6))
        tilt = float(self.get_parameter("tilt_axis_sign").value) * elevation
        tilt = max(float(self.get_parameter("min_tilt_rad").value), min(float(self.get_parameter("max_tilt_rad").value), tilt))
        if bool(self.get_parameter("publish_joint_trajectory").value):
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = [str(self.get_parameter("joint_name").value)]
            point = JointTrajectoryPoint()
            point.positions = [tilt]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 200_000_000
            traj.points = [point]
            self.joint_pub.publish(traj)
        self.bearing_pub.publish(Float32(data=float(bearing)))
        centered = x > 0.0 and abs(bearing) <= float(self.get_parameter("center_tolerance_rad").value)
        self.centered_pub.publish(Bool(data=centered))
        mode = str(self.get_parameter("alignment_mode").value)
        self.state_pub.publish(String(data=f"CAMERA_ALIGNMENT mode={mode} centered={centered} bearing={bearing:.3f} tilt_cmd={tilt:.3f} joint_publish={bool(self.get_parameter('publish_joint_trajectory').value)}"))

    def target_in_base(self) -> PoseStamped | None:
        target = self.target
        if target is None:
            return None
        base_frame = str(self.get_parameter("base_frame").value)
        if not target.header.frame_id or target.header.frame_id == base_frame:
            return target
        if self.tf_buffer is None or do_transform_pose_stamped is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                target.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=float(self.get_parameter("transform_timeout_sec").value)),
            )
            return do_transform_pose_stamped(target, transform)
        except (TransformException, Exception):
            return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeoCameraTiltJointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

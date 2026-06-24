from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

try:
    import tf2_geometry_msgs  # noqa: F401
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


class CameraGimbalControllerNode(Node):
    """Aim a camera/gimbal at a 3D target pose.

    The default backend is topic-only simulation. It publishes pan/tilt commands
    and alignment state, but it does not drive real hardware unless an external
    node consumes `/waver/camera_gimbal_cmd` and explicit hardware enable is set
    elsewhere.
    """

    def __init__(self) -> None:
        super().__init__("camera_gimbal_controller_node")
        self.declare_parameter("camera_mount_frame", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pan_axis_sign", 1.0)
        self.declare_parameter("tilt_axis_sign", 1.0)
        self.declare_parameter("max_pan_rad", 1.57)
        self.declare_parameter("min_pan_rad", -1.57)
        self.declare_parameter("max_tilt_rad", 0.8)
        self.declare_parameter("min_tilt_rad", -0.5)
        self.declare_parameter("pan_tolerance_rad", 0.05)
        self.declare_parameter("tilt_tolerance_rad", 0.05)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("alignment_timeout_sec", 5.0)
        self.declare_parameter("use_robot_body_fallback", True)
        self.declare_parameter("fallback_cmd_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("hardware_backend", "topic_only")
        self.declare_parameter("real_gimbal_output_enabled", False)

        self.target_pose: PoseStamped | None = None
        self.request_active = False
        self.request_start_time = 0.0
        self.last_pan = 0.0
        self.last_tilt = 0.0

        self.cmd_pub = self.create_publisher(Vector3, "/waver/camera_gimbal_cmd", 10)
        self.state_pub = self.create_publisher(String, "/waver/camera_alignment_state", 10)
        self.centered_pub = self.create_publisher(Bool, "/waver/camera_target_centered", 10)
        self.bearing_pub = self.create_publisher(Float32, "/waver/camera_target_bearing", 10)
        self.elevation_pub = self.create_publisher(Float32, "/waver/camera_target_elevation", 10)
        self.fallback_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("fallback_cmd_topic").value),
            10,
        )

        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.create_subscription(PoseStamped, "/waver/camera_aim_target_pose", self.target_callback, 10)
        self.create_subscription(Bool, "/waver/camera_aim_request", self.request_callback, 10)
        rate = max(float(self.get_parameter("command_rate_hz").value), 0.5)
        self.create_timer(1.0 / rate, self.tick)

    def target_callback(self, msg: PoseStamped) -> None:
        self.target_pose = msg

    def request_callback(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self.request_active:
            self.request_start_time = self._now()
        self.request_active = active
        if not active:
            self.publish_centered(False, "IDLE")

    def tick(self) -> None:
        if not self.request_active:
            return
        if self.target_pose is None:
            self.publish_centered(False, "WAIT_TARGET_POSE")
            return
        if self._now() - self.request_start_time > float(self.get_parameter("alignment_timeout_sec").value):
            self.publish_centered(False, "CAMERA_ALIGN_FAILED timeout=true")
            return
        target = self.transform_target_to_base(self.target_pose)
        if target is None:
            self.publish_centered(False, f"CAMERA_ALIGN_FAILED no_tf frame={self.target_pose.header.frame_id}")
            return

        x = float(target.pose.position.x)
        y = float(target.pose.position.y)
        z = float(target.pose.position.z)
        horizontal = max(math.hypot(x, y), 1e-6)
        bearing = math.atan2(y, x)
        elevation = math.atan2(z, horizontal)

        pan = float(self.get_parameter("pan_axis_sign").value) * bearing
        tilt = float(self.get_parameter("tilt_axis_sign").value) * elevation
        clamped_pan = self.clamp(
            pan,
            float(self.get_parameter("min_pan_rad").value),
            float(self.get_parameter("max_pan_rad").value),
        )
        clamped_tilt = self.clamp(
            tilt,
            float(self.get_parameter("min_tilt_rad").value),
            float(self.get_parameter("max_tilt_rad").value),
        )
        self.last_pan = clamped_pan
        self.last_tilt = clamped_tilt
        self.cmd_pub.publish(Vector3(x=clamped_pan, y=clamped_tilt, z=0.0))
        self.bearing_pub.publish(Float32(data=float(bearing)))
        self.elevation_pub.publish(Float32(data=float(elevation)))

        clipped = abs(clamped_pan - pan) > 1e-6 or abs(clamped_tilt - tilt) > 1e-6
        topic_only = not bool(self.get_parameter("real_gimbal_output_enabled").value)
        centered = not clipped and topic_only
        if centered:
            self.publish_centered(
                True,
                (
                    "SIMULATED_ALIGNMENT TOPIC_ONLY real_gimbal_output_enabled=false centered=true "
                    f"pan_rad={clamped_pan:.3f} tilt_rad={clamped_tilt:.3f} "
                    f"bearing_rad={bearing:.3f} elevation_rad={elevation:.3f}"
                ),
            )
            self.publish_fallback_cmd(0.0)
            return

        if clipped and bool(self.get_parameter("use_robot_body_fallback").value):
            self.publish_fallback_cmd(bearing)
            self.publish_centered(
                False,
                (
                    "ROBOT_BODY_FALLBACK_ACTIVE centered=false "
                    f"bearing_rad={bearing:.3f} elevation_rad={elevation:.3f}"
                ),
            )
        else:
            self.publish_centered(
                False,
                (
                    "COMMANDING_GIMBAL centered=false "
                    f"pan_rad={clamped_pan:.3f} tilt_rad={clamped_tilt:.3f}"
                ),
            )

    def transform_target_to_base(self, msg: PoseStamped) -> PoseStamped | None:
        base_frame = str(self.get_parameter("base_frame").value)
        if not msg.header.frame_id or msg.header.frame_id == base_frame:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = base_frame
            out.pose = msg.pose
            return out
        if self.tf_buffer is None or do_transform_pose_stamped is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, msg.header.frame_id, rclpy.time.Time())
            return do_transform_pose_stamped(msg, transform)
        except TransformException as exc:
            self.get_logger().warn(f"camera aim TF failed: {exc}")
            return None

    def publish_centered(self, centered: bool, state: str) -> None:
        self.centered_pub.publish(Bool(data=bool(centered)))
        self.state_pub.publish(String(data=state))

    def publish_fallback_cmd(self, bearing: float) -> None:
        cmd = Twist()
        cmd.angular.z = max(min(float(bearing), 0.3), -0.3)
        self.fallback_pub.publish(cmd)

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraGimbalControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

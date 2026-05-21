from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from waver_patrol.autonomy_common import clamp, make_twist, stop_twist


class TargetBodyTrackerNode(Node):
    """Turn the robot body toward an active aerial target.

    ROS 기준:
      - `angular.z > 0` 은 좌회전이다.
      - `angular.z < 0` 은 우회전이다.

    base_link 기준:
      - x = 전방, y = 좌측, z = 상방이다.
    """

    def __init__(self) -> None:
        super().__init__("target_body_tracker_node")
        self.declare_parameter("target_topic", "/waver/aerial_target")
        self.declare_parameter("active_topic", "/waver/aerial_target_active")
        self.declare_parameter("cmd_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("state_topic", "/waver/target_tracking_state")
        self.declare_parameter("direction_topic", "/waver/target_direction")
        self.declare_parameter("centered_topic", "/waver/target_centered")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_tf_transform", True)
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("lateral_axis", "y", axis_descriptor)
        self.declare_parameter("depth_axis", "x", axis_descriptor)
        self.declare_parameter("positive_lateral_is_left", True)
        self.declare_parameter("center_tolerance_rad", 0.08)
        self.declare_parameter("min_target_depth_m", 0.15)
        self.declare_parameter("angular_kp", 1.1)
        self.declare_parameter("max_angular_speed", 0.40)
        self.declare_parameter("max_angular_delta_per_tick", 0.035)
        self.declare_parameter("angular_smoothing_alpha", 0.35)
        self.declare_parameter("forward_when_aligned", False)
        self.declare_parameter("forward_speed", 0.05)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("timer_hz", 20.0)

        self.active = False
        self.target: PointStamped | None = None
        self.last_target_time = 0.0
        self.last_angular = 0.0
        self.smoothed_angular = 0.0
        self.tf_buffer = None
        self.tf_listener = None
        if bool(self.get_parameter("use_tf_transform").value):
            try:
                import tf2_geometry_msgs  # noqa: F401
                import tf2_ros

                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            except Exception as exc:
                self.get_logger().warn(f"TF setup failed; target tracking will stop on TF use: {exc}")

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.direction_pub = self.create_publisher(Float32, str(self.get_parameter("direction_topic").value), 10)
        self.centered_pub = self.create_publisher(Bool, str(self.get_parameter("centered_topic").value), 10)
        self.create_subscription(PointStamped, str(self.get_parameter("target_topic").value), self.target_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("active_topic").value), self.active_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def active_callback(self, msg: Bool) -> None:
        self.active = bool(msg.data)

    def target_callback(self, msg: PointStamped) -> None:
        self.target = msg
        self.last_target_time = self._now()

    def tick(self) -> None:
        if not self.active:
            self._publish(stop_twist(), "IDLE target_inactive", 0.0, True)
            return
        if self.target is None or self._now() - self.last_target_time > float(self.get_parameter("target_timeout_sec").value):
            self._publish(stop_twist(), "TARGET_TIMEOUT publishing_stop_candidate", 0.0, False)
            return

        target = self._target_in_base_frame(self.target)
        if target is None:
            self._publish(stop_twist(), "TF_TIMEOUT publishing_stop_candidate", 0.0, False)
            return

        lateral = self._axis_value(target, self._axis_param("lateral_axis", "y"))
        depth = self._axis_value(target, self._axis_param("depth_axis", "x"))
        if not bool(self.get_parameter("positive_lateral_is_left").value):
            lateral *= -1.0
        if not math.isfinite(lateral) or not math.isfinite(depth) or depth < float(self.get_parameter("min_target_depth_m").value):
            self._publish(stop_twist(), f"BAD_TARGET_GEOMETRY depth={depth:.2f}", 0.0, False)
            return

        bearing_rad = math.atan2(lateral, depth)
        centered = abs(bearing_rad) <= float(self.get_parameter("center_tolerance_rad").value)
        raw_angular = clamp(float(self.get_parameter("angular_kp").value) * bearing_rad, -self._max_ang(), self._max_ang())
        alpha = clamp(float(self.get_parameter("angular_smoothing_alpha").value), 0.0, 1.0)
        self.smoothed_angular = alpha * raw_angular + (1.0 - alpha) * self.smoothed_angular
        delta = clamp(self.smoothed_angular - self.last_angular, -self._max_delta(), self._max_delta())
        angular = self.last_angular + delta
        self.last_angular = angular
        linear = float(self.get_parameter("forward_speed").value) if centered and bool(self.get_parameter("forward_when_aligned").value) else 0.0
        state = (
            f"TARGET_CENTERED bearing_rad={bearing_rad:.3f}"
            if centered
            else (
                f"TARGET_LEFT_TURN_LEFT bearing_rad={bearing_rad:.3f}"
                if bearing_rad > 0.0
                else f"TARGET_RIGHT_TURN_RIGHT bearing_rad={bearing_rad:.3f}"
            )
        )
        self._publish(make_twist(linear, angular), state, bearing_rad, centered)

    def _target_in_base_frame(self, msg: PointStamped):
        base_frame = str(self.get_parameter("base_frame").value)
        if not bool(self.get_parameter("use_tf_transform").value) or msg.header.frame_id in {"", base_frame}:
            return msg.point
        if self.tf_buffer is None:
            return None
        try:
            transformed = self.tf_buffer.transform(
                msg,
                base_frame,
                timeout=Duration(seconds=0.05),
            )
            return transformed.point
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed from {msg.header.frame_id} to {base_frame}: {exc}")
            return None

    def _publish(self, cmd: Twist, state: str, direction_rad: float, centered: bool) -> None:
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        self.direction_pub.publish(Float32(data=float(direction_rad)))
        self.centered_pub.publish(Bool(data=bool(centered)))

    @staticmethod
    def _axis_value(point, axis: str) -> float:
        return float(getattr(point, axis, 0.0))

    def _axis_param(self, name: str, default: str) -> str:
        """Return a valid axis name even if ROS YAML parsed bare `y` as true."""
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return default
        text = str(value).strip()
        if text in {"x", "y", "z"}:
            return text
        self.get_logger().warn(f"Invalid {name}={text!r}; using {default}")
        return default

    def _max_ang(self) -> float:
        return float(self.get_parameter("max_angular_speed").value)

    def _max_delta(self) -> float:
        return float(self.get_parameter("max_angular_delta_per_tick").value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetBodyTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(stop_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

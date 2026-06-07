from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import tf2_geometry_msgs  # noqa: F401
    import tf2_ros
except Exception:  # pragma: no cover
    tf2_ros = None


OBSERVATION_STATES = {
    "TARGET_REACHED",
    "CAMERA_ALIGN_TO_TARGET",
    "CAMERA_ALIGN_DONE",
    "TARGET_CLASSIFICATION_WAIT",
    "TARGET_CLASSIFIED_BIRD",
    "SOUND_TASK_REQUESTED",
    "SOUND_TASK_RUNNING",
    "WAIT_TARGET_DEPARTURE",
}


class TargetObservationBodyTrackerNode(Node):
    """Body-yaw tracker gated to target observation states only."""

    def __init__(self) -> None:
        super().__init__("target_observation_body_tracker_node")
        self.declare_parameter("target_topic", "/waver/aerial_target")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("moving_target_valid_topic", "/waver/moving_target_valid")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("cmd_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("target_timeout_sec", 0.6)
        self.declare_parameter("center_tolerance_rad", 0.08)
        self.declare_parameter("angular_kp", 0.8)
        self.declare_parameter("max_angular_speed", 0.18)
        self.declare_parameter("timer_hz", 20.0)

        self.target: PointStamped | None = None
        self.last_target_time = 0.0
        self.mission_state = "IDLE"
        self.mode = "STANDBY"
        self.moving_valid = False
        self.estop = False

        self.tf_buffer = None
        self.tf_listener = None
        if bool(self.get_parameter("use_tf_transform").value) and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.centered_pub = self.create_publisher(Bool, "/waver/body_target_centered", 10)
        self.state_pub = self.create_publisher(String, "/waver/body_tracking_state", 10)
        self.create_subscription(PointStamped, str(self.get_parameter("target_topic").value), self.target_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_state_topic").value), lambda m: setattr(self, "mission_state", m.data.strip().upper()), 10)
        self.create_subscription(Bool, str(self.get_parameter("moving_target_valid_topic").value), lambda m: setattr(self, "moving_valid", bool(m.data)), 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), lambda m: setattr(self, "mode", m.data.strip().upper()), 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def target_callback(self, msg: PointStamped) -> None:
        self.target = msg
        self.last_target_time = self._now()

    def tick(self) -> None:
        cmd, centered, state = self.compute_command()
        self.cmd_pub.publish(cmd)
        self.centered_pub.publish(Bool(data=bool(centered)))
        self.state_pub.publish(String(data=state))

    def compute_command(self) -> tuple[Twist, bool, str]:
        zero = Twist()
        if self.estop:
            return zero, False, "EMERGENCY_STOP zero"
        if self.mode in {"MANUAL", "STANDBY", "EMERGENCY"}:
            return zero, False, f"MODE_BLOCKED mode={self.mode}"
        if self.mission_state not in OBSERVATION_STATES:
            return zero, False, f"MISSION_STATE_BLOCKED state={self.mission_state}"
        if not self.moving_valid:
            return zero, False, "TARGET_INVALID moving_target_valid=false"
        if self.target is None:
            return zero, False, "WAIT_TARGET"
        age = self._now() - self.last_target_time
        if age > float(self.get_parameter("target_timeout_sec").value):
            return zero, False, f"TARGET_TIMEOUT age_sec={age:.2f}"
        target = self.target_in_base_frame(self.target)
        if target is None:
            return zero, False, "TF_FAIL"
        bearing = math.atan2(float(target.point.y), max(float(target.point.x), 1e-6))
        centered = abs(bearing) <= float(self.get_parameter("center_tolerance_rad").value)
        cmd = Twist()
        if not centered:
            angular = float(self.get_parameter("angular_kp").value) * bearing
            limit = float(self.get_parameter("max_angular_speed").value)
            cmd.angular.z = max(-limit, min(limit, angular))
        state = f"{'CENTERED' if centered else 'TRACKING'} bearing_rad={bearing:.3f}"
        return cmd, centered, state

    def target_in_base_frame(self, msg: PointStamped) -> PointStamped | None:
        base = str(self.get_parameter("base_frame").value)
        if not bool(self.get_parameter("use_tf_transform").value) or msg.header.frame_id in {"", base}:
            out = PointStamped()
            out.header = msg.header
            out.header.frame_id = base
            out.point = msg.point
            return out
        if self.tf_buffer is None:
            return None
        try:
            return self.tf_buffer.transform(msg, base, timeout=Duration(seconds=0.05))
        except Exception:
            return None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetObservationBodyTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

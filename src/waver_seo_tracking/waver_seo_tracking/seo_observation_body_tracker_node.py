from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


def normalize_mission_state(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return "UNKNOWN"
    return text.split()[0].strip().upper()


class SeoObservationBodyTrackerNode(Node):
    """Body-yaw tracker that publishes only a candidate command topic."""

    TRACK_STATES = {
        "CAMERA_ALIGN_TO_TARGET",
        "CAMERA_ALIGN_DONE",
        "TARGET_CLASSIFICATION_WAIT",
        "SOUND_TASK_REQUESTED",
        "SOUND_TASK_RUNNING",
    }

    def __init__(self) -> None:
        super().__init__("seo_observation_body_tracker_node")
        self.declare_parameter("target_topic", "/waver/lidar_target_pose_base")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("transform_timeout_sec", 0.05)
        self.declare_parameter("cmd_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("angular_gain", 0.8)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("min_turn_speed", 0.18)
        self.declare_parameter("bearing_deadband_rad", 0.08)
        self.target: PoseStamped | None = None
        self.mission_state = "IDLE"
        self.tf_buffer = Buffer() if Buffer is not None else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.tf_buffer is not None else None
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.state_pub = self.create_publisher(String, "/waver/body_tracking_state", 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("target_topic").value), self.target_callback, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            lambda m: setattr(self, "mission_state", normalize_mission_state(m.data)),
            10,
        )
        self.create_timer(0.05, self.tick)

    def target_callback(self, msg: PoseStamped) -> None:
        self.target = msg

    def tick(self) -> None:
        cmd = Twist()
        if self.target is None or self.mission_state not in self.TRACK_STATES:
            self.cmd_pub.publish(cmd)
            return
        target = self.target_in_base()
        if target is None:
            self.cmd_pub.publish(cmd)
            self.state_pub.publish(String(data=f"BODY_TRACK state={self.mission_state} tf_fail=true angular=0.000"))
            return
        x = float(target.pose.position.x)
        y = float(target.pose.position.y)
        bearing = math.atan2(y, x)
        if abs(bearing) > float(self.get_parameter("bearing_deadband_rad").value):
            raw = float(self.get_parameter("angular_gain").value) * bearing
            limit = float(self.get_parameter("max_angular_speed").value)
            min_turn = min(abs(float(self.get_parameter("min_turn_speed").value)), limit)
            bounded = max(-limit, min(limit, raw))
            if abs(bounded) < min_turn:
                bounded = math.copysign(min_turn, bounded if bounded != 0.0 else bearing)
            cmd.angular.z = bounded
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=f"BODY_TRACK state={self.mission_state} bearing={bearing:.3f} angular={cmd.angular.z:.3f}"))

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
    node = SeoObservationBodyTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

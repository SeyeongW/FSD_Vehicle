from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waver_patrol.mission.mission_utils import is_finite_pose, offset_goal_from_target, pose_stamped

try:
    import tf2_geometry_msgs  # noqa: F401
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


@dataclass
class Candidate:
    pose: PoseStamped
    source: str
    moving: bool = False
    bird_confirmed: bool = False


class TargetGoalManagerNode(Node):
    """Convert radar/LiDAR/camera object candidates into 2D Nav2 mission goals.

    2D LaserScan has no z/height channel and must not be used for aerial-object
    selection. This node accepts only PoseStamped, PointStamped, or PoseArray
    object candidates that already carry 3D coordinates.
    """

    def __init__(self) -> None:
        super().__init__("target_goal_manager_node")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("min_height_m", 0.30)
        self.declare_parameter("min_depth_m", 0.50)
        self.declare_parameter("max_depth_m", 30.0)
        self.declare_parameter("min_target_distance_m", 0.8)
        self.declare_parameter("max_target_distance_m", 25.0)
        self.declare_parameter("require_motion", True)
        self.declare_parameter("require_bird_confirmed", False)
        self.declare_parameter("allow_radar_without_bird_confirmed", True)
        self.declare_parameter("goal_offset_distance_m", 1.5)
        self.declare_parameter("goal_publish_cooldown_sec", 3.0)
        self.declare_parameter("target_hold_sec", 1.0)
        self.declare_parameter("target_lost_timeout_sec", 2.0)
        self.declare_parameter("goal_yaw_policy", "FACE_TARGET")

        self.goal_pub = self.create_publisher(PoseStamped, "/waver/object_mission_goal", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/object_mission_goal_active", 10)
        self.state_pub = self.create_publisher(String, "/waver/object_mission_goal_state", 10)

        if bool(self.get_parameter("use_tf_transform").value) and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.radar_active = False
        self.aerial_active = False
        self.bird_confirmed = False
        self.last_goal_time = -1e9
        self.last_candidate_time = 0.0

        self.create_subscription(PoseStamped, "/waver/radar_target_goal", self.radar_goal_callback, 10)
        self.create_subscription(Bool, "/waver/radar_target_active", lambda m: setattr(self, "radar_active", bool(m.data)), 10)
        self.create_subscription(PointStamped, "/waver/aerial_target", self.aerial_point_callback, 10)
        self.create_subscription(Bool, "/waver/aerial_target_active", lambda m: setattr(self, "aerial_active", bool(m.data)), 10)
        self.create_subscription(PointStamped, "/waver/object_point", self.object_point_callback, 10)
        self.create_subscription(PoseArray, "/waver/lidar_objects", self.lidar_objects_callback, 10)
        self.create_subscription(PoseArray, "/lidar/detections", self.lidar_objects_callback, 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_timer(0.5, self.timeout_tick)

    def radar_goal_callback(self, msg: PoseStamped) -> None:
        self.accept_candidate(Candidate(msg, "radar", moving=True, bird_confirmed=self.bird_confirmed))

    def aerial_point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        self.accept_candidate(Candidate(pose, "aerial_target", moving=self.aerial_active, bird_confirmed=self.bird_confirmed))

    def object_point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        self.accept_candidate(Candidate(pose, "object_point", moving=True, bird_confirmed=self.bird_confirmed))

    def lidar_objects_callback(self, msg: PoseArray) -> None:
        best: Candidate | None = None
        best_depth = math.inf
        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = pose
            depth = math.hypot(pose.position.x, pose.position.y)
            if depth < best_depth:
                best_depth = depth
                best = Candidate(stamped, "lidar_pose_array", moving=self.aerial_active, bird_confirmed=self.bird_confirmed)
        if best is None:
            self.state_pub.publish(String(data="NO_LIDAR_OBJECTS"))
            return
        self.accept_candidate(best)

    def accept_candidate(self, candidate: Candidate) -> None:
        now = self._now()
        reason = self.rejection_reason(candidate)
        if reason:
            self.state_pub.publish(String(data=f"REJECTED source={candidate.source} reason={reason}"))
            self.active_pub.publish(Bool(data=False))
            return
        if now - self.last_goal_time < float(self.get_parameter("goal_publish_cooldown_sec").value):
            self.state_pub.publish(String(data=f"COOLDOWN source={candidate.source}"))
            self.active_pub.publish(Bool(data=True))
            return
        transformed = self.transform_to_global(candidate.pose)
        if transformed is None:
            self.state_pub.publish(String(data=f"TF_FAILED source={candidate.source} frame={candidate.pose.header.frame_id}"))
            self.active_pub.publish(Bool(data=False))
            return
        goal = offset_goal_from_target(
            transformed,
            float(self.get_parameter("goal_offset_distance_m").value),
            str(self.get_parameter("goal_yaw_policy").value),
        )
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)
        self.active_pub.publish(Bool(data=True))
        self.state_pub.publish(
            String(
                data=(
                    f"ACCEPTED source={candidate.source} frame={goal.header.frame_id} "
                    f"goal_x={goal.pose.position.x:.3f} goal_y={goal.pose.position.y:.3f}"
                )
            )
        )
        self.last_goal_time = now
        self.last_candidate_time = now

    def rejection_reason(self, candidate: Candidate) -> str:
        pose = candidate.pose
        if not pose.header.frame_id:
            return "missing_frame_id"
        if not is_finite_pose(pose):
            return "nan_inf_pose"
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        z = float(pose.pose.position.z)
        depth = math.hypot(x, y)
        if z < float(self.get_parameter("min_height_m").value):
            return "height_low"
        if depth < float(self.get_parameter("min_depth_m").value):
            return "depth_low"
        if depth > float(self.get_parameter("max_depth_m").value):
            return "depth_high"
        if depth < float(self.get_parameter("min_target_distance_m").value):
            return "target_too_close"
        if depth > float(self.get_parameter("max_target_distance_m").value):
            return "target_too_far"
        if bool(self.get_parameter("require_motion").value) and not candidate.moving:
            if candidate.source != "radar" or not bool(self.get_parameter("allow_radar_without_bird_confirmed").value):
                return "motion_not_confirmed"
        if bool(self.get_parameter("require_bird_confirmed").value) and not candidate.bird_confirmed:
            if candidate.source != "radar" or not bool(self.get_parameter("allow_radar_without_bird_confirmed").value):
                return "bird_not_confirmed"
        return ""

    def transform_to_global(self, msg: PoseStamped) -> PoseStamped | None:
        global_frame = str(self.get_parameter("global_frame").value)
        if msg.header.frame_id == global_frame or not bool(self.get_parameter("use_tf_transform").value):
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = msg.header.frame_id or global_frame
            out.pose = msg.pose
            return out
        if self.tf_buffer is None or do_transform_pose_stamped is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(global_frame, msg.header.frame_id, rclpy.time.Time())
            return do_transform_pose_stamped(msg, transform)
        except TransformException as exc:
            self.get_logger().warn(f"Target goal TF failed: {exc}")
            return None

    def timeout_tick(self) -> None:
        if self.last_candidate_time == 0.0:
            return
        if self._now() - self.last_candidate_time > float(self.get_parameter("target_lost_timeout_sec").value):
            self.active_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data="TARGET_LOST_TIMEOUT"))
            self.last_candidate_time = 0.0

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetGoalManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

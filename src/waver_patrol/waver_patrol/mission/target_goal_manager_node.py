from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
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
    fusion_valid: bool = False


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
        self.declare_parameter("require_3d_fusion_valid", False)
        self.declare_parameter("enforce_mode_gate", False)
        self.declare_parameter("allowed_trigger_modes", ["PATROL", "AUTO", "BIRD_CONFIRMED"])
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("battery_safety_state_topic", "/waver/battery_safety_state")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("bird_target_valid_topic", "/waver/bird_target_valid")
        self.declare_parameter("bird_target_pose_map_topic", "/waver/bird_target_pose_map")
        self.declare_parameter("goal_offset_distance_m", 1.5)
        self.declare_parameter("goal_publish_cooldown_sec", 3.0)
        self.declare_parameter("target_hold_sec", 1.0)
        self.declare_parameter("target_lost_timeout_sec", 2.0)
        self.declare_parameter("target_pose_stale_timeout_sec", 3.0)
        self.declare_parameter("robot_pose_stale_timeout_sec", 2.0)
        self.declare_parameter("goal_yaw_policy", "FACE_TARGET")
        self.declare_parameter("robot_pose_topic", "/amcl_pose")
        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("require_robot_pose_for_goal", True)
        self.declare_parameter("lidar_objects_topic", "/waver/lidar_objects")
        self.declare_parameter("lidar_detections_topic", "/lidar/detections")
        self.declare_parameter("lidar_objects_map_topic", "/waver/lidar_objects_map")
        self.declare_parameter("elevated_dynamic_targets_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("subscribe_raw_lidar_objects", False)
        self.declare_parameter("subscribe_lidar_objects_map", False)
        self.declare_parameter("target_goal_state_topic", "/waver/target_goal_state")

        self.goal_pub = self.create_publisher(PoseStamped, "/waver/object_mission_goal", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/object_mission_goal_active", 10)
        self.state_pub = self.create_publisher(String, "/waver/object_mission_goal_state", 10)
        self.target_goal_state_pub = self.create_publisher(String, str(self.get_parameter("target_goal_state_topic").value), 10)

        if bool(self.get_parameter("use_tf_transform").value) and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.radar_active = False
        self.aerial_active = False
        self.bird_confirmed = False
        self.bird_target_valid = False
        self.mode = "STANDBY"
        self.safety_state = "UNKNOWN"
        self.battery_safety_state = "UNKNOWN"
        self.estop = False
        self.external_stop = False
        self.last_goal_time = -1e9
        self.last_candidate_time = 0.0
        self.robot_pose: PoseStamped | None = None
        self.robot_pose_time = 0.0

        self.create_subscription(PoseStamped, "/waver/radar_target_goal", self.radar_goal_callback, 10)
        self.create_subscription(Bool, "/waver/radar_target_active", lambda m: setattr(self, "radar_active", bool(m.data)), 10)
        self.create_subscription(PointStamped, "/waver/aerial_target", self.aerial_point_callback, 10)
        self.create_subscription(Bool, "/waver/aerial_target_active", lambda m: setattr(self, "aerial_active", bool(m.data)), 10)
        self.create_subscription(PointStamped, "/waver/object_point", self.object_point_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self.robot_pose_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("robot_odom_topic").value),
            self.robot_odom_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("elevated_dynamic_targets_topic").value),
            self.elevated_dynamic_targets_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("bird_target_pose_map_topic").value),
            self.bird_target_pose_callback,
            10,
        )
        if bool(self.get_parameter("subscribe_raw_lidar_objects").value):
            self.create_subscription(
                PoseArray,
                str(self.get_parameter("lidar_objects_topic").value),
                self.lidar_objects_callback,
                10,
            )
            self.create_subscription(
                PoseArray,
                str(self.get_parameter("lidar_detections_topic").value),
                self.lidar_objects_callback,
                10,
            )
        if bool(self.get_parameter("subscribe_lidar_objects_map").value):
            self.create_subscription(
                PoseArray,
                str(self.get_parameter("lidar_objects_map_topic").value),
                self.lidar_objects_callback,
                10,
            )
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("bird_target_valid_topic").value), lambda m: setattr(self, "bird_target_valid", bool(m.data)), 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), lambda m: setattr(self, "mode", m.data.strip().upper()), 10)
        self.create_subscription(String, str(self.get_parameter("safety_state_topic").value), lambda m: setattr(self, "safety_state", m.data.strip().upper()), 10)
        self.create_subscription(String, str(self.get_parameter("battery_safety_state_topic").value), lambda m: setattr(self, "battery_safety_state", m.data.strip().upper()), 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), lambda m: setattr(self, "external_stop", bool(m.data)), 10)
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

    def robot_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.robot_pose = pose
        self.robot_pose_time = self._now()

    def robot_odom_callback(self, msg: Odometry) -> None:
        if self.robot_pose is not None:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.robot_pose = pose
        self.robot_pose_time = self._now()

    def bird_target_pose_callback(self, msg: PoseStamped) -> None:
        self.accept_candidate(
            Candidate(
                msg,
                "bird_3d_fusion",
                moving=self.bird_target_valid,
                bird_confirmed=self.bird_confirmed,
                fusion_valid=self.bird_target_valid,
            )
        )

    def elevated_dynamic_targets_callback(self, msg: PoseArray) -> None:
        # 역할: height>=3m AND ego-motion-compensated dynamic 필터를 통과한 객체만 mission goal 후보로 받는다.
        # raw /waver/lidar_objects는 최종 target 판단에 쓰지 않고, 이 토픽이 mission trigger의 우선 입력이다.
        if not msg.poses:
            return
        best: Candidate | None = None
        best_depth = math.inf
        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = pose
            depth = math.hypot(pose.position.x, pose.position.y)
            if depth < best_depth:
                best_depth = depth
                best = Candidate(
                    stamped,
                    "elevated_dynamic_target",
                    moving=True,
                    bird_confirmed=self.bird_confirmed,
                    fusion_valid=self.bird_target_valid,
                )
        if best is not None:
            self.accept_candidate(best)

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
            self._publish_state(f"REJECTED source={candidate.source} reason={reason}")
            self.active_pub.publish(Bool(data=False))
            return
        if now - self.last_goal_time < float(self.get_parameter("goal_publish_cooldown_sec").value):
            self._publish_state(f"COOLDOWN source={candidate.source}")
            self.active_pub.publish(Bool(data=True))
            return
        transformed = self.transform_to_global(candidate.pose)
        if transformed is None:
            self._publish_state(f"TF_FAILED source={candidate.source} frame={candidate.pose.header.frame_id}")
            self.active_pub.publish(Bool(data=False))
            return
        robot_pose = self.robot_pose
        if robot_pose is not None and robot_pose.header.frame_id != transformed.header.frame_id:
            robot_pose = self.transform_to_global(robot_pose)
        if robot_pose is None and bool(self.get_parameter("require_robot_pose_for_goal").value):
            self._publish_state(f"REJECTED source={candidate.source} reason=robot_pose_unavailable")
            self.active_pub.publish(Bool(data=False))
            return
        goal = offset_goal_from_target(
            transformed,
            float(self.get_parameter("goal_offset_distance_m").value),
            str(self.get_parameter("goal_yaw_policy").value),
            robot_pose=robot_pose,
        )
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)
        self.active_pub.publish(Bool(data=True))
        self._publish_state(
            (
                f"ACCEPTED source={candidate.source} frame={goal.header.frame_id} "
                f"goal_x={goal.pose.position.x:.3f} goal_y={goal.pose.position.y:.3f}"
            )
        )
        self.last_goal_time = now
        self.last_candidate_time = now

    def rejection_reason(self, candidate: Candidate) -> str:
        if self.estop:
            return "emergency_stop_active"
        if self.external_stop:
            return "external_stop_active"
        if bool(self.get_parameter("enforce_mode_gate").value):
            allowed = {str(v).upper() for v in self.get_parameter("allowed_trigger_modes").value}
            if self.mode not in allowed:
                return f"mode_not_allowed:{self.mode}"
        if any(token in self.safety_state for token in ("EMERGENCY", "STOP", "FAULT")) and "MAPPING" not in self.safety_state:
            return f"safety_state_blocks:{self.safety_state}"
        if any(token in self.battery_safety_state for token in ("CRITICAL", "STALE_STOP", "BATTERY_STALE_STOP")):
            return f"battery_blocks:{self.battery_safety_state}"
        pose = candidate.pose
        if not pose.header.frame_id:
            return "missing_frame_id"
        if not is_finite_pose(pose):
            return "nan_inf_pose"
        if pose.header.stamp.sec or pose.header.stamp.nanosec:
            stamp = float(pose.header.stamp.sec) + float(pose.header.stamp.nanosec) * 1e-9
            if self._now() - stamp > float(self.get_parameter("target_pose_stale_timeout_sec").value):
                return "target_pose_stale"
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
        if bool(self.get_parameter("require_3d_fusion_valid").value) and not candidate.fusion_valid:
            return "bird_3d_fusion_not_valid"
        if bool(self.get_parameter("require_robot_pose_for_goal").value):
            if self.robot_pose is None:
                return "robot_pose_unavailable"
            if self._now() - self.robot_pose_time > float(self.get_parameter("robot_pose_stale_timeout_sec").value):
                return "robot_pose_stale"
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
            self._publish_state("TARGET_LOST_TIMEOUT")
            self.last_candidate_time = 0.0

    def _publish_state(self, text: str) -> None:
        msg = String(data=text)
        self.state_pub.publish(msg)
        self.target_goal_state_pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetGoalManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        # 역할: launch shutdown 경계에서 rclpy wait set이 먼저 닫히는 경우가 있다.
        # 이때 남는 context-invalid 예외는 정상 종료로 처리하고, 그 외 예외는 보존한다.
        if rclpy.ok() and "context is not valid" not in str(exc):
            raise
    finally:
        if rclpy.ok():
            try:
                node.active_pub.publish(Bool(data=False))
            except Exception:
                pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()

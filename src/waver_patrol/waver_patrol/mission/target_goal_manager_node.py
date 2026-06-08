from __future__ import annotations

import json
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
    z_valid: bool = True
    dynamic_valid: bool = False
    bird_confirmed: bool = False
    fusion_valid: bool = False
    target_class: str = "none"
    target_confidence: float = 0.0
    mission_type: str = "INSPECTION_ONLY"


@dataclass
class InspectedTargetMemory:
    x: float
    y: float
    z: float
    stamp: float
    count: int = 1


class TargetGoalManagerNode(Node):
    """Convert radar/LiDAR/camera object candidates into 2D Nav2 mission goals.

    2D LaserScan has no z/height channel and must not be used for aerial-object
    selection. This node accepts only PoseStamped, PointStamped, or PoseArray
    object candidates that already carry 3D coordinates.
    """

    TERMINAL_INSPECTION_STATES = {
        "SOUND_TASK_DONE",
        "SOUND_TASK_BLOCKED_BY_CLASS",
        "SOUND_TASK_TIMEOUT",
        "TARGET_NOT_BIRD",
        "TARGET_CLASSIFIED_DRONE",
        "TARGET_CLASSIFIED_UNKNOWN",
        "TARGET_CLASSIFIED_IRRELEVANT",
        "CAMERA_ALIGN_FAILED",
        "TARGET_LOST_RECOVERY",
        "RETURN_TO_INTERRUPTED_WAYPOINT",
        "RESUME_PATROL",
    }

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
        self.declare_parameter("mission_target_policy", "lidar_first_inspection")
        self.declare_parameter("allow_lidar_dynamic_without_bird_confirmed_for_inspection", True)
        self.declare_parameter("require_bird_confirmed_for_inspection_goal", False)
        self.declare_parameter("require_bird_confirmed_for_sound", True)
        self.declare_parameter("require_3d_lidar_z_valid_for_inspection", True)
        self.declare_parameter("require_dynamic_valid_for_inspection", True)
        self.declare_parameter("require_camera_classification_before_sound", True)
        self.declare_parameter("deterrence_classes", ["bird"])
        self.declare_parameter("non_deterrence_classes", ["drone", "unknown", "irrelevant", "none"])
        self.declare_parameter("inspection_goal_offset_distance_m", 2.0)
        self.declare_parameter("classification_standoff_distance_m", 2.0)
        self.declare_parameter("target_arrival_tolerance_xy_m", 0.5)
        self.declare_parameter("inspection_cooldown_sec", 8.0)
        self.declare_parameter("max_inspection_retries_per_target", 1)
        self.declare_parameter("inspected_target_lockout_sec", 120.0)
        self.declare_parameter("inspected_target_radius_m", 1.5)
        self.declare_parameter("enforce_mode_gate", False)
        self.declare_parameter("allowed_trigger_modes", ["PATROL", "AUTO", "BIRD_CANDIDATE"])
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
        self.declare_parameter("allowed_goal_x_min_m", -1.0e9)
        self.declare_parameter("allowed_goal_x_max_m", 1.0e9)
        self.declare_parameter("allowed_goal_y_min_m", -1.0e9)
        self.declare_parameter("allowed_goal_y_max_m", 1.0e9)

        self.goal_pub = self.create_publisher(PoseStamped, "/waver/object_mission_goal", 10)
        self.goal_debug_pub = self.create_publisher(String, "/waver/object_mission_goal_debug", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/object_mission_goal_active", 10)
        self.state_pub = self.create_publisher(String, "/waver/object_mission_goal_state", 10)
        self.target_goal_state_pub = self.create_publisher(String, str(self.get_parameter("target_goal_state_topic").value), 10)
        self.inspection_target_pub = self.create_publisher(PoseStamped, "/waver/inspection_target_pose_map", 10)
        self.inspection_active_pub = self.create_publisher(Bool, "/waver/inspection_target_active", 10)
        self.inspection_state_pub = self.create_publisher(String, "/waver/inspection_target_state", 10)
        self.inspection_reason_pub = self.create_publisher(String, "/waver/inspection_goal_reason", 10)

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
        self.target_class = "none"
        self.target_confidence = 0.0
        self.last_goal_time = -1e9
        self.last_candidate_time = 0.0
        self.robot_pose: PoseStamped | None = None
        self.robot_pose_time = 0.0
        self.inspected_targets: list[InspectedTargetMemory] = []
        self.pending_inspection_target: PoseStamped | None = None

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
        self.create_subscription(String, "/waver/target_class", lambda m: setattr(self, "target_class", m.data.strip().lower()), 10)
        self.create_subscription(Bool, str(self.get_parameter("bird_target_valid_topic").value), lambda m: setattr(self, "bird_target_valid", bool(m.data)), 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), lambda m: setattr(self, "mode", m.data.strip().upper()), 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(String, str(self.get_parameter("safety_state_topic").value), lambda m: setattr(self, "safety_state", m.data.strip().upper()), 10)
        self.create_subscription(String, str(self.get_parameter("battery_safety_state_topic").value), lambda m: setattr(self, "battery_safety_state", m.data.strip().upper()), 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_timer(0.5, self.timeout_tick)

    def radar_goal_callback(self, msg: PoseStamped) -> None:
        self.accept_candidate(Candidate(msg, "radar", moving=True, dynamic_valid=True, bird_confirmed=self.bird_confirmed))

    def aerial_point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        self.accept_candidate(
            Candidate(
                pose,
                "aerial_target",
                moving=self.aerial_active,
                dynamic_valid=self.aerial_active,
                bird_confirmed=self.bird_confirmed,
                target_class=self.target_class,
            )
        )

    def object_point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        self.accept_candidate(Candidate(pose, "object_point", moving=True, dynamic_valid=True, bird_confirmed=self.bird_confirmed))

    def robot_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.robot_pose = pose
        self.robot_pose_time = self._now()

    def robot_odom_callback(self, msg: Odometry) -> None:
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
                dynamic_valid=self.bird_target_valid,
                bird_confirmed=self.bird_confirmed,
                fusion_valid=self.bird_target_valid,
                target_class=self.target_class,
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
                    z_valid=pose.position.z >= float(self.get_parameter("min_height_m").value),
                    dynamic_valid=True,
                    bird_confirmed=self.bird_confirmed,
                    fusion_valid=self.bird_target_valid,
                    target_class=self.target_class,
                    mission_type="INSPECTION_ONLY",
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
                best = Candidate(
                    stamped,
                    "lidar_pose_array",
                    moving=self.aerial_active,
                    z_valid=pose.position.z >= float(self.get_parameter("min_height_m").value),
                    dynamic_valid=self.aerial_active,
                    bird_confirmed=self.bird_confirmed,
                    target_class=self.target_class,
                    mission_type="INSPECTION_ONLY",
                )
        if best is None:
            self.state_pub.publish(String(data="NO_LIDAR_OBJECTS"))
            return
        self.accept_candidate(best)

    def accept_candidate(self, candidate: Candidate) -> None:
        now = self._now()
        reason = self.rejection_reason(candidate)
        if reason:
            candidate.mission_type = self.reject_mission_type(reason)
            self._publish_state(f"REJECT source={candidate.source} mission_type={candidate.mission_type} {reason}")
            self.active_pub.publish(Bool(data=False))
            self.inspection_active_pub.publish(Bool(data=False))
            return
        cooldown = float(self.get_parameter("inspection_cooldown_sec").value) if self.is_lidar_inspection_candidate(candidate) else float(self.get_parameter("goal_publish_cooldown_sec").value)
        if now - self.last_goal_time < cooldown:
            self._publish_state(f"REJECT cooldown source={candidate.source} remaining_sec={cooldown - (now - self.last_goal_time):.2f}")
            self.active_pub.publish(Bool(data=True))
            self.inspection_active_pub.publish(Bool(data=True))
            return
        transformed = self.transform_to_global(candidate.pose)
        if transformed is None:
            candidate.mission_type = "REJECTED_NO_TF"
            self._publish_state(f"REJECT no_tf source={candidate.source} frame={candidate.pose.header.frame_id}")
            self.active_pub.publish(Bool(data=False))
            self.inspection_active_pub.publish(Bool(data=False))
            return
        if self.outside_allowed_goal_area(transformed):
            candidate.mission_type = "REJECTED_FORBIDDEN_ZONE"
            self._publish_state(
                f"REJECT forbidden_zone source={candidate.source} "
                f"x={transformed.pose.position.x:.2f} y={transformed.pose.position.y:.2f}"
            )
            self.active_pub.publish(Bool(data=False))
            self.inspection_active_pub.publish(Bool(data=False))
            return
        inspected_reject = self.recently_inspected_rejection_reason(transformed)
        if inspected_reject:
            self._publish_state(f"REJECT source={candidate.source} mission_type=REJECTED_ALREADY_INSPECTED {inspected_reject}")
            self.active_pub.publish(Bool(data=False))
            self.inspection_active_pub.publish(Bool(data=False))
            return
        robot_pose = self.robot_pose
        if robot_pose is not None and robot_pose.header.frame_id != transformed.header.frame_id:
            robot_pose = self.transform_to_global(robot_pose)
        if robot_pose is None and bool(self.get_parameter("require_robot_pose_for_goal").value):
            self._publish_state(f"REJECT robot_pose_unavailable source={candidate.source}")
            self.active_pub.publish(Bool(data=False))
            self.inspection_active_pub.publish(Bool(data=False))
            return
        offset_distance = (
            float(self.get_parameter("inspection_goal_offset_distance_m").value)
            if self.is_lidar_inspection_candidate(candidate)
            else float(self.get_parameter("goal_offset_distance_m").value)
        )
        goal = offset_goal_from_target(
            transformed,
            offset_distance,
            str(self.get_parameter("goal_yaw_policy").value),
            robot_pose=robot_pose,
        )
        goal.header.stamp = self.get_clock().now().to_msg()
        self.publish_goal_debug(candidate, transformed, goal, robot_pose, offset_distance)
        self.goal_pub.publish(goal)
        self.active_pub.publish(Bool(data=True))
        self.inspection_target_pub.publish(transformed)
        self.inspection_active_pub.publish(Bool(data=True))
        self.pending_inspection_target = transformed
        height = transformed.pose.position.z
        distance = self.robot_relative_range_xy(transformed)
        if distance is None:
            distance = math.hypot(transformed.pose.position.x, transformed.pose.position.y)
        reason_text = (
            f"ACCEPT_INSPECTION_GOAL source={candidate.source} height={height:.2f} range={distance:.2f} "
            f"dynamic={candidate.dynamic_valid} z_valid={candidate.z_valid} bird_confirmed={candidate.bird_confirmed} "
            f"target_class={candidate.target_class} offset_m={offset_distance:.2f}"
        )
        self._publish_state(
            (
                f"{reason_text} frame={goal.header.frame_id} "
                f"goal_x={goal.pose.position.x:.3f} goal_y={goal.pose.position.y:.3f}"
            )
        )
        self.last_goal_time = now
        self.last_candidate_time = now

    def publish_goal_debug(
        self,
        candidate: Candidate,
        target: PoseStamped,
        goal: PoseStamped,
        robot_pose: PoseStamped | None,
        offset_distance: float,
    ) -> None:
        target_x = float(target.pose.position.x)
        target_y = float(target.pose.position.y)
        target_z = float(target.pose.position.z)
        goal_x = float(goal.pose.position.x)
        goal_y = float(goal.pose.position.y)
        goal_z = float(goal.pose.position.z)
        robot_x = math.nan
        robot_y = math.nan
        if robot_pose is not None:
            robot_x = float(robot_pose.pose.position.x)
            robot_y = float(robot_pose.pose.position.y)
        goal_to_target = math.hypot(goal_x - target_x, goal_y - target_y)
        payload = {
            "time_sec": self._now(),
            "goal_role": "TARGET_INSPECTION" if self.is_lidar_inspection_candidate(candidate) else "RADAR_TARGET",
            "source": candidate.source,
            "candidate_provenance": "lidar" if candidate.source in {"elevated_dynamic_target", "aerial_target", "lidar_pose_array"} else candidate.source,
            "target_frame": target.header.frame_id,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "goal_frame": goal.header.frame_id,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_z": goal_z,
            "offset_distance_m": offset_distance,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "goal_to_target_xy_m": goal_to_target,
            "goal_yaw_policy": str(self.get_parameter("goal_yaw_policy").value),
            "candidate_track_id": -1,
            "moving": candidate.moving,
            "dynamic_valid": candidate.dynamic_valid,
            "bird_confirmed": candidate.bird_confirmed,
        }
        self.goal_debug_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def mission_state_callback(self, msg: String) -> None:
        state = (msg.data or "").strip().split()[0].upper() if (msg.data or "").strip() else "UNKNOWN"
        if state in self.TERMINAL_INSPECTION_STATES and self.pending_inspection_target is not None:
            self.remember_inspected_target(self.pending_inspection_target)
            self._publish_state(f"INSPECTION_MEMORY_COMMITTED terminal_state={state}")
            self.pending_inspection_target = None

    def rejection_reason(self, candidate: Candidate) -> str:
        if self.estop:
            return "safety_state_blocks=emergency_stop_active"
        if self.external_stop:
            return "safety_state_blocks=external_stop_active"
        if bool(self.get_parameter("enforce_mode_gate").value):
            allowed = {str(v).upper() for v in self.get_parameter("allowed_trigger_modes").value}
            if self.mode not in allowed:
                return f"mode_not_allowed mode={self.mode} allowed={sorted(allowed)}"
        if any(token in self.safety_state for token in ("EMERGENCY", "STOP", "FAULT")) and "MAPPING" not in self.safety_state:
            return f"safety_state_blocks={self.safety_state}"
        if any(token in self.battery_safety_state for token in ("CRITICAL", "STALE_STOP", "BATTERY_STALE_STOP")):
            return f"battery_blocks={self.battery_safety_state}"
        pose = candidate.pose
        if not pose.header.frame_id:
            return "no_tf missing_frame_id"
        if not is_finite_pose(pose):
            return "invalid_pose nan_inf_pose"
        if pose.header.stamp.sec or pose.header.stamp.nanosec:
            stamp = float(pose.header.stamp.sec) + float(pose.header.stamp.nanosec) * 1e-9
            age = self._now() - stamp
            if age > float(self.get_parameter("target_pose_stale_timeout_sec").value):
                return f"target_pose_stale age={age:.2f}"
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        z = float(pose.pose.position.z)
        depth = self.robot_relative_range_xy(pose)
        if depth is None:
            depth = math.hypot(x, y)
        min_height = float(self.get_parameter("min_height_m").value)
        lidar_first = self.is_lidar_inspection_candidate(candidate)
        if bool(self.get_parameter("require_3d_lidar_z_valid_for_inspection").value) and lidar_first and not candidate.z_valid:
            return f"height_low z={z:.2f} threshold={min_height:.2f}"
        if z < min_height:
            return f"height_low z={z:.2f} threshold={min_height:.2f}"
        if depth < float(self.get_parameter("min_depth_m").value):
            return f"depth_low depth={depth:.2f} threshold={float(self.get_parameter('min_depth_m').value):.2f}"
        if depth > float(self.get_parameter("max_depth_m").value):
            return f"depth_high depth={depth:.2f} threshold={float(self.get_parameter('max_depth_m').value):.2f}"
        if depth < float(self.get_parameter("min_target_distance_m").value):
            return f"target_too_close range={depth:.2f} threshold={float(self.get_parameter('min_target_distance_m').value):.2f}"
        if depth > float(self.get_parameter("max_target_distance_m").value):
            return f"target_too_far range={depth:.2f} threshold={float(self.get_parameter('max_target_distance_m').value):.2f}"
        if bool(self.get_parameter("require_dynamic_valid_for_inspection").value) and lidar_first and not candidate.dynamic_valid:
            return "dynamic_not_confirmed"
        if bool(self.get_parameter("require_motion").value) and not candidate.moving:
            if candidate.source != "radar" or not bool(self.get_parameter("allow_radar_without_bird_confirmed").value):
                return "dynamic_not_confirmed"
        require_bird_for_inspection = bool(self.get_parameter("require_bird_confirmed_for_inspection_goal").value)
        legacy_require_bird = bool(self.get_parameter("require_bird_confirmed").value)
        lidar_bird_bypass = (
            lidar_first
            and str(self.get_parameter("mission_target_policy").value) == "lidar_first_inspection"
            and bool(self.get_parameter("allow_lidar_dynamic_without_bird_confirmed_for_inspection").value)
        )
        if (require_bird_for_inspection or legacy_require_bird) and not candidate.bird_confirmed and not lidar_bird_bypass:
            if candidate.source != "radar" or not bool(self.get_parameter("allow_radar_without_bird_confirmed").value):
                return "bird_not_confirmed_for_inspection"
        if bool(self.get_parameter("require_3d_fusion_valid").value) and not candidate.fusion_valid and not lidar_bird_bypass:
            return "bird_3d_fusion_not_valid_for_inspection"
        if bool(self.get_parameter("require_robot_pose_for_goal").value):
            if self.robot_pose is None:
                return "robot_pose_unavailable"
            age = self._now() - self.robot_pose_time
            if age > float(self.get_parameter("robot_pose_stale_timeout_sec").value):
                return f"robot_pose_stale age={age:.2f}"
        return ""

    def recently_inspected_rejection_reason(self, target: PoseStamped) -> str:
        self.prune_inspected_targets()
        max_retries = max(1, int(self.get_parameter("max_inspection_retries_per_target").value))
        radius = float(self.get_parameter("inspected_target_radius_m").value)
        now = self._now()
        tx = float(target.pose.position.x)
        ty = float(target.pose.position.y)
        tz = float(target.pose.position.z)
        for memory in self.inspected_targets:
            distance_xy = math.hypot(tx - memory.x, ty - memory.y)
            distance_z = abs(tz - memory.z)
            if distance_xy <= radius and distance_z <= max(radius, 1.0) and memory.count >= max_retries:
                age = max(0.0, now - memory.stamp)
                return (
                    f"target_already_inspected age={age:.1f}s count={memory.count} "
                    f"distance_xy={distance_xy:.2f} radius={radius:.2f}"
                )
        return ""

    def remember_inspected_target(self, target: PoseStamped) -> None:
        self.prune_inspected_targets()
        radius = float(self.get_parameter("inspected_target_radius_m").value)
        now = self._now()
        tx = float(target.pose.position.x)
        ty = float(target.pose.position.y)
        tz = float(target.pose.position.z)
        for memory in self.inspected_targets:
            if math.hypot(tx - memory.x, ty - memory.y) <= radius and abs(tz - memory.z) <= max(radius, 1.0):
                memory.x = tx
                memory.y = ty
                memory.z = tz
                memory.stamp = now
                memory.count += 1
                return
        self.inspected_targets.append(InspectedTargetMemory(tx, ty, tz, now))

    def prune_inspected_targets(self) -> None:
        lockout = max(0.0, float(self.get_parameter("inspected_target_lockout_sec").value))
        if lockout <= 0.0:
            self.inspected_targets.clear()
            return
        now = self._now()
        self.inspected_targets = [
            memory for memory in self.inspected_targets if now - memory.stamp <= lockout
        ]

    def is_lidar_inspection_candidate(self, candidate: Candidate) -> bool:
        return candidate.source in {"elevated_dynamic_target", "aerial_target", "lidar_pose_array"}

    def robot_relative_range_xy(self, target: PoseStamped) -> float | None:
        if self.robot_pose is None:
            return None
        robot = self.robot_pose
        target_frame = target.header.frame_id or str(self.get_parameter("global_frame").value)
        robot_frame = robot.header.frame_id or str(self.get_parameter("global_frame").value)
        target_pose = target
        robot_pose = robot
        if target_frame != robot_frame:
            target_pose = self.transform_to_global(target)
            robot_pose = self.transform_to_global(robot)
            if target_pose is None or robot_pose is None or target_pose.header.frame_id != robot_pose.header.frame_id:
                return None
        return math.hypot(
            float(target_pose.pose.position.x) - float(robot_pose.pose.position.x),
            float(target_pose.pose.position.y) - float(robot_pose.pose.position.y),
        )

    def outside_allowed_goal_area(self, pose: PoseStamped) -> bool:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        return not (
            float(self.get_parameter("allowed_goal_x_min_m").value) <= x <= float(self.get_parameter("allowed_goal_x_max_m").value)
            and float(self.get_parameter("allowed_goal_y_min_m").value) <= y <= float(self.get_parameter("allowed_goal_y_max_m").value)
        )

    @staticmethod
    def reject_mission_type(reason: str) -> str:
        if "height_low" in reason:
            return "REJECTED_LOW_HEIGHT"
        if "dynamic_not_confirmed" in reason or "motion_not_confirmed" in reason:
            return "REJECTED_NO_DYNAMIC"
        if "no_tf" in reason or "frame" in reason:
            return "REJECTED_NO_TF"
        if "target_too_close" in reason:
            return "REJECTED_TOO_CLOSE"
        if "target_too_far" in reason or "depth_high" in reason:
            return "REJECTED_TOO_FAR"
        if "static" in reason:
            return "REJECTED_STATIC"
        return "REJECTED_NO_TF" if "tf" in reason.lower() else "REJECTED_STATIC"

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
        self.inspection_state_pub.publish(msg)
        self.inspection_reason_pub.publish(msg)

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

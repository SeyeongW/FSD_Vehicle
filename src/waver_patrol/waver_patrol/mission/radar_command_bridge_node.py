from __future__ import annotations

import json
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from waver_patrol.mission.mission_utils import is_finite_number, is_finite_pose, pose_stamped

try:
    import tf2_geometry_msgs  # noqa: F401
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover - ROS install dependent
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


@dataclass
class RadarMetrics:
    range_m: float = math.nan
    bearing_rad: float = math.nan
    doppler_mps: float = math.nan
    confidence: float = math.nan


class RadarCommandBridgeNode(Node):
    """Normalize future 4D radar commands into Waver mission goal topics.

    역할:
      - 실제 4D radar driver를 구현하지 않는다.
      - PoseStamped/PointStamped/PoseArray/JSON 형태의 외부 target을 검증해
        `/waver/radar_target_goal` 후보로 내보낸다.
      - range, bearing, Doppler, confidence는 논문 로그용 표준 토픽으로 분리한다.
    """

    def __init__(self) -> None:
        super().__init__("radar_command_bridge_node")
        self.declare_parameter("target_pose_topic", "/radar4d/target_pose")
        self.declare_parameter("target_point_topic", "/radar4d/target_point")
        self.declare_parameter("objects_topic", "/radar4d/objects")
        self.declare_parameter("target_json_topic", "/radar4d/target_json")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("min_target_distance_m", 0.8)
        self.declare_parameter("max_target_distance_m", 40.0)
        self.declare_parameter("min_target_height_m", 0.2)
        self.declare_parameter("max_target_height_m", 20.0)
        self.declare_parameter("min_radar_confidence", 0.50)
        self.declare_parameter("require_positive_confidence", True)
        self.declare_parameter("goal_publish_cooldown_sec", 3.0)
        self.declare_parameter("radar_command_timeout_sec", 2.0)
        self.declare_parameter("reject_nan_inf", True)
        self.declare_parameter("reject_missing_frame_id", True)

        self.goal_pub = self.create_publisher(PoseStamped, "/waver/radar_target_goal", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/radar_target_active", 10)
        self.state_pub = self.create_publisher(String, "/waver/radar_target_state", 10)
        self.range_pub = self.create_publisher(Float32, "/waver/radar_target_range_m", 10)
        self.bearing_pub = self.create_publisher(Float32, "/waver/radar_target_bearing_rad", 10)
        self.doppler_pub = self.create_publisher(Float32, "/waver/radar_target_doppler_mps", 10)
        self.conf_pub = self.create_publisher(Float32, "/waver/radar_target_confidence", 10)

        if bool(self.get_parameter("use_tf_transform").value) and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.last_publish_time = -1e9
        self.last_target_time = 0.0
        self.create_subscription(PoseStamped, str(self.get_parameter("target_pose_topic").value), self.pose_callback, 10)
        self.create_subscription(PointStamped, str(self.get_parameter("target_point_topic").value), self.point_callback, 10)
        self.create_subscription(PoseArray, str(self.get_parameter("objects_topic").value), self.objects_callback, 10)
        self.create_subscription(String, str(self.get_parameter("target_json_topic").value), self.json_callback, 10)
        self.create_timer(0.5, self.timeout_tick)

    def pose_callback(self, msg: PoseStamped) -> None:
        metrics = self.metrics_from_pose(msg)
        self.accept_pose(msg, metrics, "pose")

    def point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        metrics = self.metrics_from_pose(pose)
        self.accept_pose(pose, metrics, "point")

    def objects_callback(self, msg: PoseArray) -> None:
        best_pose: PoseStamped | None = None
        best_metrics = RadarMetrics()
        best_range = math.inf
        for pose in msg.poses:
            candidate = PoseStamped()
            candidate.header = msg.header
            candidate.pose = pose
            metrics = self.metrics_from_pose(candidate)
            if math.isfinite(metrics.range_m) and metrics.range_m < best_range:
                best_range = metrics.range_m
                best_pose = candidate
                best_metrics = metrics
        if best_pose is None:
            self.publish_state("REJECTED_EMPTY_OBJECTS")
            self.active_pub.publish(Bool(data=False))
            return
        self.accept_pose(best_pose, best_metrics, "pose_array_nearest")

    def json_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_state(f"REJECTED_JSON_PARSE_ERROR reason={exc}")
            self.active_pub.publish(Bool(data=False))
            return
        frame_id = str(data.get("frame_id", self.get_parameter("base_frame").value))
        if all(key in data for key in ("x", "y")):
            pose = pose_stamped(frame_id, float(data["x"]), float(data["y"]), 0.0, float(data.get("z", 0.0)))
        elif "range_m" in data and "azimuth_rad" in data:
            r = float(data["range_m"])
            az = float(data["azimuth_rad"])
            el = float(data.get("elevation_rad", 0.0))
            pose = pose_stamped(frame_id, r * math.cos(el) * math.cos(az), r * math.cos(el) * math.sin(az), 0.0, r * math.sin(el))
        else:
            self.publish_state("REJECTED_JSON_MISSING_POSITION")
            self.active_pub.publish(Bool(data=False))
            return
        metrics = RadarMetrics(
            range_m=float(data.get("range_m", math.hypot(pose.pose.position.x, pose.pose.position.y))),
            bearing_rad=float(data.get("azimuth_rad", math.atan2(pose.pose.position.y, pose.pose.position.x))),
            doppler_mps=float(data.get("doppler_mps", math.nan)),
            confidence=float(data.get("confidence", math.nan)),
        )
        self.accept_pose(pose, metrics, "json")

    def accept_pose(self, msg: PoseStamped, metrics: RadarMetrics, source: str) -> None:
        now = self._now()
        reason = self.rejection_reason(msg, metrics)
        if reason:
            self.publish_state(f"REJECTED source={source} reason={reason}")
            self.active_pub.publish(Bool(data=False))
            return
        if now - self.last_publish_time < float(self.get_parameter("goal_publish_cooldown_sec").value):
            self.publish_state(f"COOLDOWN source={source}")
            self.active_pub.publish(Bool(data=True))
            return
        transformed = self.transform_to_global(msg)
        if transformed is None:
            self.publish_state(f"TF_FAILED source={source} frame={msg.header.frame_id}")
            self.active_pub.publish(Bool(data=False))
            return
        transformed.header.stamp = self.get_clock().now().to_msg()
        self.last_publish_time = now
        self.last_target_time = now
        self.goal_pub.publish(transformed)
        self.active_pub.publish(Bool(data=True))
        self.publish_metrics(metrics)
        self.publish_state(
            f"ACCEPTED source={source} frame={transformed.header.frame_id} "
            f"range_m={metrics.range_m:.3f} bearing_rad={metrics.bearing_rad:.3f}"
        )

    def rejection_reason(self, msg: PoseStamped, metrics: RadarMetrics) -> str:
        if bool(self.get_parameter("reject_missing_frame_id").value) and not msg.header.frame_id:
            return "missing_frame_id"
        if bool(self.get_parameter("reject_nan_inf").value) and not is_finite_pose(msg):
            return "nan_inf_pose"
        distance = metrics.range_m
        if not math.isfinite(distance):
            distance = math.hypot(msg.pose.position.x, msg.pose.position.y)
        if distance < float(self.get_parameter("min_target_distance_m").value):
            return "too_close"
        if distance > float(self.get_parameter("max_target_distance_m").value):
            return "too_far"
        z = float(msg.pose.position.z)
        if z < float(self.get_parameter("min_target_height_m").value):
            return "height_low"
        if z > float(self.get_parameter("max_target_height_m").value):
            return "height_high"
        if bool(self.get_parameter("require_positive_confidence").value):
            if not math.isfinite(metrics.confidence) or metrics.confidence < float(self.get_parameter("min_radar_confidence").value):
                return "confidence_low"
        return ""

    def metrics_from_pose(self, msg: PoseStamped) -> RadarMetrics:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        return RadarMetrics(range_m=math.sqrt(x * x + y * y + z * z), bearing_rad=math.atan2(y, x), confidence=math.nan)

    def transform_to_global(self, msg: PoseStamped) -> PoseStamped | None:
        global_frame = str(self.get_parameter("global_frame").value)
        if not bool(self.get_parameter("use_tf_transform").value) or msg.header.frame_id == global_frame:
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
            self.get_logger().warn(f"Radar target TF failed: {exc}")
            return None

    def timeout_tick(self) -> None:
        if self.last_target_time == 0.0:
            return
        if self._now() - self.last_target_time > float(self.get_parameter("radar_command_timeout_sec").value):
            self.active_pub.publish(Bool(data=False))
            self.publish_state("RADAR_TARGET_TIMEOUT")
            self.last_target_time = 0.0

    def publish_metrics(self, metrics: RadarMetrics) -> None:
        self.range_pub.publish(Float32(data=float(metrics.range_m if is_finite_number(metrics.range_m) else -1.0)))
        self.bearing_pub.publish(Float32(data=float(metrics.bearing_rad if is_finite_number(metrics.bearing_rad) else 0.0)))
        self.doppler_pub.publish(Float32(data=float(metrics.doppler_mps if is_finite_number(metrics.doppler_mps) else 0.0)))
        self.conf_pub.publish(Float32(data=float(metrics.confidence if is_finite_number(metrics.confidence) else -1.0)))

    def publish_state(self, state: str) -> None:
        self.state_pub.publish(String(data=state))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RadarCommandBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

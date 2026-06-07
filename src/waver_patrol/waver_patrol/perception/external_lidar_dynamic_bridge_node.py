from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

try:
    import tf2_geometry_msgs  # noqa: F401
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None


@dataclass
class LidarBridgeTarget:
    point: PointStamped
    height_m: float
    range_m: float
    velocity_mps: float
    valid: bool
    state: str


class ExternalLidarDynamicBridgeNode(Node):
    """Adapt an external dynamic LiDAR target stream into Waver mission topics.

    This node is intentionally a bridge only. It never publishes `/cmd_vel` and
    never sends Nav2 goals. Target height and target range are published as
    separate scalar topics so the height>=3m gate is not confused with distance.
    """

    def __init__(self) -> None:
        super().__init__("external_lidar_dynamic_bridge_node")
        self.declare_parameter("input_lidar_target_topic", "/external/lidar/target")
        self.declare_parameter("input_lidar_target_pose_topic", "/external/lidar/target_pose")
        self.declare_parameter("input_lidar_objects_topic", "/external/lidar/objects")
        self.declare_parameter("input_target_frame", "map")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("use_tf_transform", False)
        self.declare_parameter("min_target_height_m", 3.0)
        self.declare_parameter("min_target_range_m", 0.5)
        self.declare_parameter("max_target_range_m", 30.0)
        self.declare_parameter("target_stale_timeout_sec", 1.0)
        self.declare_parameter("input_has_velocity", False)
        self.declare_parameter("input_has_target_id", False)

        self.pose_array_pub = self.create_publisher(PoseArray, "/waver/lidar_objects_map", 10)
        self.target_pub = self.create_publisher(PointStamped, "/waver/aerial_target", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/aerial_target_active", 10)
        self.moving_pub = self.create_publisher(Bool, "/waver/moving_target_valid", 10)
        self.height_pub = self.create_publisher(Float32, "/waver/lidar_target_height_m", 10)
        self.range_pub = self.create_publisher(Float32, "/waver/lidar_target_range_m", 10)
        self.velocity_pub = self.create_publisher(Float32, "/waver/lidar_target_velocity_mps", 10)
        self.state_pub = self.create_publisher(String, "/waver/lidar_target_state", 10)

        self.tf_buffer = None
        self.tf_listener = None
        if bool(self.get_parameter("use_tf_transform").value) and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_target: PointStamped | None = None
        self.last_target_time = 0.0
        self.last_target_wall_time = 0.0
        self.last_velocity = 0.0

        self.create_subscription(
            PointStamped,
            str(self.get_parameter("input_lidar_target_topic").value),
            self.target_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_lidar_target_pose_topic").value),
            self.target_pose_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("input_lidar_objects_topic").value),
            self.objects_callback,
            10,
        )
        self.create_timer(0.2, self.timeout_tick)

    def target_pose_callback(self, msg: PoseStamped) -> None:
        point = PointStamped()
        point.header = msg.header
        point.point = msg.pose.position
        self.target_callback(point)

    def objects_callback(self, msg: PoseArray) -> None:
        out = PoseArray()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or str(self.get_parameter("input_target_frame").value)
        out.poses = list(msg.poses)
        best = self._best_pose(msg)
        if best is not None:
            point = PointStamped()
            point.header = out.header
            point.point = best.position
            self.target_callback(point)
        self.pose_array_pub.publish(out)

    def target_callback(self, msg: PointStamped) -> None:
        target = self.evaluate_target(msg)
        if target is None:
            return
        self.last_target = target.point
        self.last_target_time = self._now()
        self.last_target_wall_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_velocity = target.velocity_mps
        self.target_pub.publish(target.point)
        self.active_pub.publish(Bool(data=target.valid))
        self.moving_pub.publish(Bool(data=target.valid))
        self.height_pub.publish(Float32(data=float(target.height_m)))
        self.range_pub.publish(Float32(data=float(target.range_m)))
        self.velocity_pub.publish(Float32(data=float(target.velocity_mps)))
        self.state_pub.publish(String(data=target.state))

    def evaluate_target(self, msg: PointStamped) -> LidarBridgeTarget | None:
        point = self._transform_point(msg)
        if point is None:
            self._publish_inactive("TF_FAIL")
            return None
        height = float(point.point.z)
        target_range = math.hypot(float(point.point.x), float(point.point.y))
        velocity = self._estimate_velocity(point)
        valid, state = self._target_validity(height, target_range)
        return LidarBridgeTarget(point, height, target_range, velocity, valid, state)

    def _target_validity(self, height: float, target_range: float) -> tuple[bool, str]:
        min_height = float(self.get_parameter("min_target_height_m").value)
        min_range = float(self.get_parameter("min_target_range_m").value)
        max_range = float(self.get_parameter("max_target_range_m").value)
        if not all(math.isfinite(v) for v in (height, target_range)):
            return False, "INVALID_NAN"
        if height < min_height:
            return False, f"CANDIDATE height_low height_m={height:.2f} threshold={min_height:.2f}"
        if target_range < min_range:
            return False, f"CANDIDATE range_low range_m={target_range:.2f} threshold={min_range:.2f}"
        if target_range > max_range:
            return False, f"CANDIDATE range_high range_m={target_range:.2f} threshold={max_range:.2f}"
        return True, f"LOCKED height_m={height:.2f} range_m={target_range:.2f}"

    def _transform_point(self, msg: PointStamped) -> PointStamped | None:
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or str(self.get_parameter("input_target_frame").value)
        out.point = msg.point
        target_frame = str(self.get_parameter("output_frame").value)
        if not bool(self.get_parameter("use_tf_transform").value) or out.header.frame_id == target_frame:
            out.header.frame_id = out.header.frame_id or target_frame
            return out
        if self.tf_buffer is None:
            return None
        try:
            return self.tf_buffer.transform(out, target_frame, timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(f"external lidar bridge TF failed: {exc}")
            return None

    def _estimate_velocity(self, point: PointStamped) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_target is None or self.last_target.header.frame_id != point.header.frame_id:
            return 0.0
        dt = now - self.last_target_wall_time
        if dt <= 1e-3 or dt > 2.0:
            return 0.0
        dx = float(point.point.x - self.last_target.point.x)
        dy = float(point.point.y - self.last_target.point.y)
        dz = float(point.point.z - self.last_target.point.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz) / dt

    @staticmethod
    def _best_pose(msg: PoseArray) -> Pose | None:
        best = None
        best_range = math.inf
        for pose in msg.poses:
            distance = math.hypot(float(pose.position.x), float(pose.position.y))
            if distance < best_range:
                best = pose
                best_range = distance
        return best

    def timeout_tick(self) -> None:
        if self.last_target_time == 0.0:
            self._publish_inactive("NO_TARGET")
            return
        age = self._now() - self.last_target_time
        if age > float(self.get_parameter("target_stale_timeout_sec").value):
            self._publish_inactive(f"LOST age_sec={age:.2f}")

    def _publish_inactive(self, state: str) -> None:
        self.active_pub.publish(Bool(data=False))
        self.moving_pub.publish(Bool(data=False))
        self.state_pub.publish(String(data=state))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExternalLidarDynamicBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

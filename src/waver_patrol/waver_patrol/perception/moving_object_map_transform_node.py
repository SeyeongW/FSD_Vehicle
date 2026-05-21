from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import tf2_geometry_msgs  # noqa: F401
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover - depends on ROS install
    Buffer = None
    TransformException = Exception
    TransformListener = None
    do_transform_pose_stamped = None


@dataclass
class TransformResult:
    frame_id: str
    poses: list[PoseStamped]
    skipped: int = 0
    reason: str = ""


class MovingObjectMapTransformNode(Node):
    """Transform moving LiDAR object coordinates into the shared SLAM frame.

    역할:
      - 기존 LiDAR 객체 후보 토픽을 삭제하지 않고 구독만 한다.
      - LiDAR/raw sensor frame 좌표를 tf2로 map 또는 odom 기준 좌표로 변환한다.
      - 최종 /cmd_vel, Nav2 goal, 4D radar command는 발행하지 않는다.
      - 출력은 Waver/RC카가 판단 데이터로 구독 가능한 공통 좌표계 PoseArray/PointStamped다.
    """

    def __init__(self) -> None:
        super().__init__("moving_object_map_transform_node")
        self.declare_parameter("input_type", "pose_array")
        self.declare_parameter("input_topic", "/waver/lidar_objects")
        self.declare_parameter("output_pose_array_topic", "/waver/lidar_objects_map")
        self.declare_parameter("output_point_topic", "/waver/lidar_object_point_map")
        self.declare_parameter("debug_marker_topic", "/waver/moving_objects_map_marker")
        self.declare_parameter("state_topic", "/waver/moving_object_map_transform_state")
        self.declare_parameter("lidar_frame", "")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("fallback_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("transform_timeout_sec", 0.2)
        self.declare_parameter("publish_debug_marker", True)
        self.declare_parameter("max_objects", 80)
        self.declare_parameter("z_policy", "keep")
        self.declare_parameter("marker_lifetime_sec", 0.5)

        self.pose_pub = self.create_publisher(PoseArray, str(self.get_parameter("output_pose_array_topic").value), 10)
        self.point_pub = self.create_publisher(PointStamped, str(self.get_parameter("output_point_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("debug_marker_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)

        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None
            self.get_logger().warn("tf2 is unavailable; moving object map transform will skip all inputs")

        input_topic = str(self.get_parameter("input_topic").value)
        input_type = str(self.get_parameter("input_type").value).strip().lower()
        if input_type in {"pose_array", "posearray"}:
            self.create_subscription(PoseArray, input_topic, self.pose_array_callback, 10)
        elif input_type in {"point_stamped", "pointstamped", "point"}:
            self.create_subscription(PointStamped, input_topic, self.point_callback, 10)
        elif input_type in {"marker_array", "markerarray", "markers"}:
            self.create_subscription(MarkerArray, input_topic, self.marker_array_callback, 10)
        else:
            self.get_logger().warn(f"Unsupported input_type={input_type}; defaulting to PoseArray on {input_topic}")
            self.create_subscription(PoseArray, input_topic, self.pose_array_callback, 10)

        self.get_logger().info(
            f"Transforming moving objects from {input_topic} ({input_type}) to "
            f"{self.get_parameter('target_frame').value} with fallback {self.get_parameter('fallback_frame').value}"
        )

    def pose_array_callback(self, msg: PoseArray) -> None:
        result = self._transform_pose_array(msg)
        self._publish_result(result, source_type="PoseArray")

    def point_callback(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        frame_id = self._source_frame(msg.header.frame_id)
        if not frame_id:
            self._publish_state("SKIP missing_frame source=PointStamped")
            return
        pose.header.frame_id = frame_id
        transformed = self._transform_pose_with_fallback(pose)
        if transformed is None:
            self._publish_state(f"TF_SKIP source=PointStamped frame={frame_id}")
            return
        self._publish_result(TransformResult(transformed.header.frame_id, [transformed]), source_type="PointStamped")

    def marker_array_callback(self, msg: MarkerArray) -> None:
        poses: list[PoseStamped] = []
        skipped = 0
        max_objects = int(self.get_parameter("max_objects").value)
        for marker in msg.markers[:max_objects]:
            if marker.action == Marker.DELETE:
                continue
            frame_id = self._source_frame(marker.header.frame_id)
            if not frame_id:
                skipped += 1
                continue
            pose = PoseStamped()
            pose.header = marker.header
            pose.header.frame_id = frame_id
            pose.pose = marker.pose
            transformed = self._transform_pose_with_fallback(pose)
            if transformed is None:
                skipped += 1
                continue
            poses.append(transformed)
        frame_id = poses[0].header.frame_id if poses else str(self.get_parameter("target_frame").value)
        self._publish_result(TransformResult(frame_id, poses, skipped), source_type="MarkerArray")

    def _transform_pose_array(self, msg: PoseArray) -> TransformResult:
        frame_id = self._source_frame(msg.header.frame_id)
        if not frame_id:
            return TransformResult(str(self.get_parameter("target_frame").value), [], len(msg.poses), "missing_frame")
        poses: list[PoseStamped] = []
        skipped = 0
        max_objects = int(self.get_parameter("max_objects").value)
        for pose in msg.poses[:max_objects]:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.header.frame_id = frame_id
            stamped.pose = pose
            transformed = self._transform_pose_with_fallback(stamped)
            if transformed is None:
                skipped += 1
                continue
            poses.append(transformed)
        out_frame = poses[0].header.frame_id if poses else str(self.get_parameter("target_frame").value)
        return TransformResult(out_frame, poses, skipped)

    def _transform_pose_with_fallback(self, msg: PoseStamped) -> PoseStamped | None:
        for target_frame in self._target_frames():
            transformed = self._transform_pose(msg, target_frame)
            if transformed is not None:
                self._apply_z_policy(transformed)
                return transformed
        return None

    def _transform_pose(self, msg: PoseStamped, target_frame: str) -> PoseStamped | None:
        source_frame = self._source_frame(msg.header.frame_id)
        if not source_frame or not target_frame:
            return None
        if source_frame == target_frame:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = target_frame
            out.pose = msg.pose
            return out
        if self.tf_buffer is None or do_transform_pose_stamped is None:
            return None
        try:
            stamp = rclpy.time.Time() if bool(self.get_parameter("use_latest_tf").value) else rclpy.time.Time.from_msg(msg.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=float(self.get_parameter("transform_timeout_sec").value)),
            )
            out = do_transform_pose_stamped(msg, transform)
            out.header.frame_id = target_frame
            return out
        except TransformException as exc:
            self.get_logger().warn(f"TF unavailable {source_frame}->{target_frame}: {exc}")
            return None

    def _publish_result(self, result: TransformResult, source_type: str) -> None:
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = result.frame_id
        pose_array.poses = [pose.pose for pose in result.poses]
        self.pose_pub.publish(pose_array)
        if result.poses:
            point = PointStamped()
            point.header = pose_array.header
            point.point = result.poses[0].pose.position
            self.point_pub.publish(point)
        if bool(self.get_parameter("publish_debug_marker").value):
            self.marker_pub.publish(self._make_markers(pose_array))
        self._publish_state(
            f"OK source={source_type} frame={pose_array.header.frame_id} "
            f"count={len(pose_array.poses)} skipped={result.skipped} reason={result.reason}"
        )

    def _make_markers(self, msg: PoseArray) -> MarkerArray:
        markers = MarkerArray()
        lifetime = Duration(seconds=float(self.get_parameter("marker_lifetime_sec").value)).to_msg()
        for i, pose in enumerate(msg.poses):
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = "waver_moving_objects_map"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.color.r = 0.1
            sphere.color.g = 0.9
            sphere.color.b = 0.2
            sphere.color.a = 0.85
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            text = Marker()
            text.header = msg.header
            text.ns = "waver_moving_objects_map_text"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.9
            text.text = f"obj{i} ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
            text.lifetime = lifetime
            markers.markers.append(text)
        return markers

    def _target_frames(self) -> list[str]:
        target = str(self.get_parameter("target_frame").value).strip() or "map"
        fallback = str(self.get_parameter("fallback_frame").value).strip()
        frames = [target]
        if fallback and fallback not in frames:
            frames.append(fallback)
        return frames

    def _source_frame(self, frame_id: str) -> str:
        frame = frame_id.strip() if frame_id else ""
        if frame:
            return frame
        return str(self.get_parameter("lidar_frame").value).strip()

    def _apply_z_policy(self, msg: PoseStamped) -> None:
        policy = str(self.get_parameter("z_policy").value).strip().lower()
        if policy == "zero":
            msg.pose.position.z = 0.0
        elif policy == "finite_or_zero" and not math.isfinite(float(msg.pose.position.z)):
            msg.pose.position.z = 0.0

    def _publish_state(self, text: str) -> None:
        self.state_pub.publish(String(data=text))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MovingObjectMapTransformNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

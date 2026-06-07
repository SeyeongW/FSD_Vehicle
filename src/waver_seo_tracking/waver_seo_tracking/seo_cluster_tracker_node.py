from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from sklearn.cluster import DBSCAN
except Exception:  # pragma: no cover
    DBSCAN = None

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
class TargetTrack:
    pose: PoseStamped
    previous_pose: PoseStamped | None = None
    last_seen: float = 0.0
    seen_count: int = 0
    motion_count: int = 0
    speed_mps: float = 0.0
    moving: bool = False
    source: str = "unknown"


def distance_xy(a: PoseStamped, b: PoseStamped) -> float:
    return math.hypot(
        float(a.pose.position.x) - float(b.pose.position.x),
        float(a.pose.position.y) - float(b.pose.position.y),
    )


class SeoClusterTrackerNode(Node):
    """Gazebo-only dynamic target tracker adapted from the SEO branch.

    The node never publishes /cmd_vel. It turns point cloud clusters, or the
    Gazebo bird pose fallback when the Livox plugin is unavailable, into the
    Waver mission target topics consumed by target_goal_manager_node.
    """

    def __init__(self) -> None:
        super().__init__("seo_cluster_tracker_node")
        self.declare_parameter("pointcloud_topic", "/mid360_PointCloud2")
        self.declare_parameter("bird_pose_topic", "/bird/nearest_pose")
        self.declare_parameter("use_gazebo_bird_pose_fallback", True)
        self.declare_parameter("detector_mode", "ground_truth")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("pointcloud_frame_alias", "livox")
        self.declare_parameter("roi_min_range", 0.3)
        self.declare_parameter("roi_max_range", 15.0)
        self.declare_parameter("target_min_height_m", 3.0)
        self.declare_parameter("target_max_height_m", 8.0)
        self.declare_parameter("dbscan_eps", 0.55)
        self.declare_parameter("dbscan_min_samples", 4)
        self.declare_parameter("min_cluster_points", 5)
        self.declare_parameter("max_input_points", 12000)
        self.declare_parameter("track_match_dist_m", 2.5)
        self.declare_parameter("move_threshold_m", 0.05)
        self.declare_parameter("min_motion_frames", 2)
        self.declare_parameter("target_timeout_s", 1.5)
        self.declare_parameter("hold_last_target_sec", 1.0)
        self.declare_parameter("output_smoothing_alpha", 0.35)
        self.declare_parameter("prefer_gazebo_fallback_sec", 0.8)
        self.declare_parameter("state_topic", "/waver/lidar_tracking_state")

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.track: TargetTrack | None = None
        self.last_pc_time = 0.0
        self.last_fallback_time = 0.0
        self.last_publish_time = 0.0
        self.last_raw_count = 0
        self.last_roi_count = 0
        self.last_cluster_count = 0
        self.last_tf_ok = False
        self.odom: Odometry | None = None

        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.objects_pub = self.create_publisher(PoseArray, "/waver/elevated_dynamic_targets", 10)
        self.lock_pub = self.create_publisher(Bool, "/waver/dynamic_object_lock", 10)
        self.lock_state_pub = self.create_publisher(String, "/waver/dynamic_object_lock_state", 10)
        self.moving_pub = self.create_publisher(Bool, "/waver/moving_target_valid", 10)
        self.target_base_pub = self.create_publisher(PoseStamped, "/waver/lidar_target_pose_base", 10)
        self.target_odom_pub = self.create_publisher(PoseStamped, "/waver/lidar_target_pose_odom", 10)
        self.bird_target_pub = self.create_publisher(PointStamped, "/bird_target", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/cluster_markers", 10)

        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("bird_pose_topic").value),
            self.bird_pose_callback,
            10,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_timer(0.2, self.timeout_tick)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.last_pc_time = self._now()
        target = self.target_from_pointcloud(msg)
        if target is None:
            self.publish_state("WAITING_CLUSTER pointcloud_received=true")
            return
        self.update_and_publish_track(target, "lidar_cluster")

    def bird_pose_callback(self, msg: PoseStamped) -> None:
        mode = self.detector_mode()
        if mode == "lidar":
            self.last_fallback_time = self._now()
            self.publish_state("GT_LOG_ONLY /bird/nearest_pose ignored_for_decision=true")
            return
        if not bool(self.get_parameter("use_gazebo_bird_pose_fallback").value):
            return
        self.last_fallback_time = self._now()
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = msg.header.frame_id or "odom"
        pose.pose = msg.pose
        if pose.header.frame_id == "world":
            pose.header.frame_id = "odom"
        self.update_and_publish_track(pose, "gazebo_bird_pose_fallback")

    def target_from_pointcloud(self, msg: PointCloud2) -> PoseStamped | None:
        source_frame = self.normalize_pointcloud_frame(msg.header.frame_id)
        points = []
        raw_count = 0
        roi_min = float(self.get_parameter("roi_min_range").value)
        roi_max = float(self.get_parameter("roi_max_range").value)
        z_min = float(self.get_parameter("target_min_height_m").value)
        z_max = float(self.get_parameter("target_max_height_m").value)
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            raw_count += 1
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            r = math.hypot(x, y)
            if roi_min <= r <= roi_max and z_min <= z <= z_max:
                points.append((x, y, z))
        self.last_raw_count = raw_count
        self.last_roi_count = len(points)
        self.last_cluster_count = 0
        self.last_tf_ok = False
        min_points = int(self.get_parameter("min_cluster_points").value)
        if len(points) < min_points:
            self.publish_state(f"NO_CLUSTER raw={raw_count} roi={len(points)}")
            return None
        max_input = int(self.get_parameter("max_input_points").value)
        if max_input > 0 and len(points) > max_input:
            stride = max(1, math.ceil(len(points) / max_input))
            points = points[::stride]
        arr = np.asarray(points, dtype=np.float32)
        if DBSCAN is not None:
            labels = DBSCAN(
                eps=float(self.get_parameter("dbscan_eps").value),
                min_samples=int(self.get_parameter("dbscan_min_samples").value),
            ).fit(arr).labels_
            best_cluster = None
            for label in sorted(set(labels)):
                if label == -1:
                    continue
                cluster = arr[labels == label]
                if best_cluster is None or len(cluster) > len(best_cluster):
                    best_cluster = cluster
            if best_cluster is None or len(best_cluster) < min_points:
                self.publish_state(f"NO_DBSCAN_CLUSTER roi={len(points)}")
                return None
            self.last_cluster_count = len(best_cluster)
            centroid = np.median(best_cluster, axis=0)
        else:
            self.last_cluster_count = len(arr)
            centroid = np.median(arr, axis=0)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = source_frame
        pose.pose.position.x = float(centroid[0])
        pose.pose.position.y = float(centroid[1])
        pose.pose.position.z = float(centroid[2])
        pose.pose.orientation.w = 1.0
        transformed = self.transform_pose(pose, self.target_frame)
        self.last_tf_ok = transformed is not None
        return transformed or pose

    def normalize_pointcloud_frame(self, frame_id: str) -> str:
        frame = (frame_id or "").strip().lstrip("/")
        alias = str(self.get_parameter("pointcloud_frame_alias").value).strip().lstrip("/")
        if not frame:
            return alias or "livox"
        if "::" in frame:
            parts = [p for p in frame.split("::") if p]
            if alias and alias in parts:
                return alias
            if parts:
                return parts[-1]
        return frame

    def update_and_publish_track(self, pose: PoseStamped, source: str) -> None:
        now = self._now()
        detector_mode = self.detector_mode()
        if detector_mode == "lidar" and source == "gazebo_bird_pose_fallback":
            self.publish_state("GT_LOG_ONLY source=gazebo_bird_pose_fallback ignored_for_decision=true")
            return
        prefer_fallback_window = max(0.0, float(self.get_parameter("prefer_gazebo_fallback_sec").value))
        if (
            detector_mode in {"ground_truth", "fused"}
            and source != "gazebo_bird_pose_fallback"
            and bool(self.get_parameter("use_gazebo_bird_pose_fallback").value)
            and self.track is not None
            and self.track.source == "gazebo_bird_pose_fallback"
            and now - self.last_fallback_time <= prefer_fallback_window
        ):
            self.publish_state(
                "HOLD_LAST_TARGET suppress=lidar_cluster reason=fallback_recent "
                f"age={now - self.last_fallback_time:.2f}"
            )
            return

        if self.track is None or distance_xy(self.track.pose, pose) > float(self.get_parameter("track_match_dist_m").value):
            self.track = TargetTrack(pose=pose, last_seen=now, seen_count=1, motion_count=1, source=source)
        else:
            dt = max(1e-3, now - self.track.last_seen)
            raw_motion = distance_xy(self.track.pose, pose)
            smoothed_pose = self.smooth_pose(self.track.pose, pose)
            motion = distance_xy(self.track.pose, smoothed_pose)
            self.track.previous_pose = self.track.pose
            self.track.pose = smoothed_pose
            self.track.last_seen = now
            self.track.seen_count += 1
            self.track.speed_mps = raw_motion / dt
            self.track.source = source
            if raw_motion >= float(self.get_parameter("move_threshold_m").value):
                self.track.motion_count += 1
        # Gazebo bird pose fallback already comes from a moving actor manager.
        # Use repeated observations as the lock gate so a slow bird is not
        # treated as static solely because the pose delta is below the motion
        # threshold during a short alignment window.
        if source == "gazebo_bird_pose_fallback":
            moving = self.track.seen_count >= int(self.get_parameter("min_motion_frames").value)
        else:
            moving = self.track.motion_count >= int(self.get_parameter("min_motion_frames").value)
        self.track.moving = bool(moving)
        self.publish_target(self.track.pose, moving, source)

    def smooth_pose(self, previous: PoseStamped, current: PoseStamped) -> PoseStamped:
        alpha = float(self.get_parameter("output_smoothing_alpha").value)
        alpha = max(0.0, min(1.0, alpha))
        if alpha >= 0.999 or previous.header.frame_id != current.header.frame_id:
            return current
        out = PoseStamped()
        out.header = current.header
        out.pose = current.pose
        out.pose.position.x = (1.0 - alpha) * previous.pose.position.x + alpha * current.pose.position.x
        out.pose.position.y = (1.0 - alpha) * previous.pose.position.y + alpha * current.pose.position.y
        out.pose.position.z = (1.0 - alpha) * previous.pose.position.z + alpha * current.pose.position.z
        return out

    def publish_target(self, pose: PoseStamped, moving: bool, source: str) -> None:
        pose.header.stamp = self.get_clock().now().to_msg()
        self.last_publish_time = self._now()
        detector_mode = self.detector_mode()
        decision_allowed = source != "gazebo_bird_pose_fallback" or detector_mode in {"ground_truth", "fused"}
        dynamic_decision = bool(moving) and decision_allowed
        arr = PoseArray()
        arr.header = pose.header
        arr.poses.append(pose.pose)
        if dynamic_decision:
            self.objects_pub.publish(arr)
        self.lock_pub.publish(Bool(data=dynamic_decision))
        self.moving_pub.publish(Bool(data=dynamic_decision))
        self.lock_state_pub.publish(
            String(
                data=(
                    f"LOCKED source={source} moving={dynamic_decision} "
                    f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f} z={pose.pose.position.z:.2f} "
                    f"decision_allowed={decision_allowed}"
                )
            )
        )
        if pose.header.frame_id == self.target_frame:
            self.target_odom_pub.publish(pose)
        base_pose = self.transform_pose(pose, str(self.get_parameter("base_frame").value))
        if base_pose is None:
            base_pose = self.odom_pose_to_base_fallback(pose)
        if base_pose is not None:
            self.target_base_pub.publish(base_pose)
        point = PointStamped()
        point.header = pose.header
        point.point = pose.pose.position
        if dynamic_decision:
            self.bird_target_pub.publish(point)
            self.publish_marker(pose)
        self.publish_state(
            f"TARGET_OK source={source} moving={dynamic_decision} speed={self.track.speed_mps if self.track else 0.0:.2f}"
        )

    def publish_marker(self, pose: PoseStamped) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "waver_seo_dynamic_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def timeout_tick(self) -> None:
        if self.track is None:
            self.lock_pub.publish(Bool(data=False))
            self.moving_pub.publish(Bool(data=False))
            return
        age = self._now() - self.track.last_seen
        hold_sec = max(0.0, float(self.get_parameter("hold_last_target_sec").value))
        if age <= hold_sec:
            self.publish_target(self.track.pose, self.track.moving, f"last_value_hold age={age:.2f}")
            return
        if age > float(self.get_parameter("target_timeout_s").value):
            self.lock_pub.publish(Bool(data=False))
            self.moving_pub.publish(Bool(data=False))
            self.lock_state_pub.publish(String(data=f"TARGET_LOST age={age:.2f}"))
            self.publish_state(f"TARGET_LOST age={age:.2f}")
            self.track = None

    def transform_pose(self, msg: PoseStamped, target_frame: str) -> PoseStamped | None:
        if not target_frame or msg.header.frame_id == target_frame:
            return msg
        if self.tf_buffer is None or do_transform_pose_stamped is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, msg.header.frame_id, rclpy.time.Time())
            return do_transform_pose_stamped(msg, tf)
        except TransformException:
            return None

    def odom_pose_to_base_fallback(self, msg: PoseStamped) -> PoseStamped | None:
        if self.odom is None:
            return None
        if (msg.header.frame_id or "") not in {"odom", "world"}:
            return None
        robot = self.odom.pose.pose
        target = msg.pose.position
        q = robot.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        dx = target.x - robot.position.x
        dy = target.y - robot.position.y
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(self.get_parameter("base_frame").value)
        out.pose.orientation.w = 1.0
        out.pose.position.x = c * dx - s * dy
        out.pose.position.y = s * dx + c * dy
        out.pose.position.z = target.z - robot.position.z
        return out

    def publish_state(self, text: str) -> None:
        mode = self.detector_mode()
        fallback_allowed = mode in {"ground_truth", "fused"} and bool(self.get_parameter("use_gazebo_bird_pose_fallback").value)
        decision_allowed = not (mode == "lidar" and "gazebo_bird_pose_fallback" in text)
        state = (
            f"{text} detector_mode={mode} provenance={'gazebo_gt' if 'fallback' in text or 'GT_' in text else 'lidar'} "
            f"raw={self.last_raw_count} roi={self.last_roi_count} cluster={self.last_cluster_count} "
            f"tf_ok={self.last_tf_ok} fallback_allowed={fallback_allowed} decision_allowed={decision_allowed}"
        )
        self.state_pub.publish(String(data=state))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 if self.get_clock().now().nanoseconds else time.monotonic()

    def detector_mode(self) -> str:
        mode = str(self.get_parameter("detector_mode").value).strip().lower()
        if mode not in {"ground_truth", "lidar", "fused"}:
            self.get_logger().warn(f"Unknown detector_mode={mode!r}; using lidar")
            return "lidar"
        return mode


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeoClusterTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

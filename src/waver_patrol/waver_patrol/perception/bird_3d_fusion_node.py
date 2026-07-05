from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2D, Detection2DArray

try:
    import message_filters
except Exception:  # pragma: no cover
    message_filters = None

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None


class Bird3DFusionNode(Node):
    """Fuse bird 2D detections with PointCloud2 to produce validated 3D bird targets."""

    def __init__(self) -> None:
        super().__init__("bird_3d_fusion_node")
        self.declare_parameter("detections_topic", "/waver/bird_detections_2d")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("pointcloud_topic", "/mid360_PointCloud2")
        self.declare_parameter("moving_target_valid_topic", "/waver/moving_target_valid")
        self.declare_parameter("dynamic_targets_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("pose_base_topic", "/waver/bird_target_pose_base")
        self.declare_parameter("pose_map_topic", "/waver/bird_target_pose_map")
        self.declare_parameter("valid_topic", "/waver/bird_target_valid")
        self.declare_parameter("state_topic", "/waver/bird_fusion_state")
        self.declare_parameter("marker_topic", "/waver/bird_target_marker")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("min_points_in_bbox", 5)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("min_height_m", 3.0)
        self.declare_parameter("max_height_m", 30.0)
        self.declare_parameter("ground_z_offset_m", 0.0)
        self.declare_parameter("min_range_m", 1.0)
        self.declare_parameter("max_range_m", 30.0)
        self.declare_parameter("max_bbox_point_spread_m", 3.0)
        self.declare_parameter("detections_stale_sec", 0.5)
        self.declare_parameter("camera_info_stale_sec", 5.0)
        self.declare_parameter("pointcloud_stale_sec", 0.5)
        self.declare_parameter("require_dynamic_valid", True)
        self.declare_parameter("dynamic_association_radius_m", 1.5)
        self.declare_parameter("max_dynamic_target_age_sec", 1.0)
        self.declare_parameter("max_sync_dt_sec", 0.25)
        self.declare_parameter("require_camera_optical_frame", True)
        self.declare_parameter("camera_lidar_extrinsic_path", os.path.expanduser("~/ros2_ws5/FSD_Vehicle/config/sensors/camera_lidar_extrinsic.yaml"))
        self.declare_parameter("require_calibrated_extrinsic", True)

        self.camera_info: CameraInfo | None = None
        self.camera_info_time = 0.0
        self.detections: Detection2DArray | None = None
        self.detections_time = 0.0
        self.bird_confirmed = False
        self.dynamic_valid = False
        self.dynamic_targets: list[PoseStamped] = []
        self.dynamic_targets_time = 0.0
        self.last_cloud_time = 0.0
        self.synced_packet_count = 0
        self.rejected_packet_count = 0

        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.pose_base_pub = self.create_publisher(PoseStamped, str(self.get_parameter("pose_base_topic").value), 10)
        self.pose_map_pub = self.create_publisher(PoseStamped, str(self.get_parameter("pose_map_topic").value), 10)
        self.valid_pub = self.create_publisher(Bool, str(self.get_parameter("valid_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.sync_state_pub = self.create_publisher(String, "/waver/bird_fusion_sync_state", 10)
        self.marker_pub = self.create_publisher(Marker, str(self.get_parameter("marker_topic").value), 10)

        self.create_subscription(Bool, str(self.get_parameter("bird_confirmed_topic").value), lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("moving_target_valid_topic").value), lambda m: setattr(self, "dynamic_valid", bool(m.data)), 10)
        self.create_subscription(PoseArray, str(self.get_parameter("dynamic_targets_topic").value), self.dynamic_targets_callback, 10)
        if message_filters is not None:
            self.det_filter = message_filters.Subscriber(self, Detection2DArray, str(self.get_parameter("detections_topic").value))
            self.info_filter = message_filters.Subscriber(self, CameraInfo, str(self.get_parameter("camera_info_topic").value))
            self.cloud_filter = message_filters.Subscriber(self, PointCloud2, str(self.get_parameter("pointcloud_topic").value))
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.det_filter, self.info_filter, self.cloud_filter],
                queue_size=10,
                slop=float(self.get_parameter("max_sync_dt_sec").value),
            )
            self.sync.registerCallback(self.synced_callback)
            self.sync_state_pub.publish(String(data="SYNC_ACTIVE backend=message_filters"))
        else:
            self.create_subscription(Detection2DArray, str(self.get_parameter("detections_topic").value), self.detections_callback, 10)
            self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.camera_info_callback, 10)
            self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), self.cloud_callback, 5)
            self.sync_state_pub.publish(String(data="SYNC_ACTIVE backend=fallback_latest_values"))
        self.create_timer(0.2, self.health_tick)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.camera_info_time = self._now()

    def detections_callback(self, msg: Detection2DArray) -> None:
        self.detections = msg
        self.detections_time = self._now()

    def synced_callback(self, detections: Detection2DArray, info: CameraInfo, cloud: PointCloud2) -> None:
        now = self._now()
        self.synced_packet_count += 1
        self.camera_info = info
        self.camera_info_time = now
        self.detections = detections
        self.detections_time = now
        sync_dt = self._stamp_delta_sec(cloud.header.stamp, detections.header.stamp)
        info_dt = self._stamp_delta_sec(cloud.header.stamp, info.header.stamp)
        self.sync_state_pub.publish(
            String(
                data=(
                    "SYNC_PACKET backend=message_filters "
                    f"packets={self.synced_packet_count} rejected={self.rejected_packet_count} "
                    f"detection_cloud_dt_sec={sync_dt:.3f} info_cloud_dt_sec={info_dt:.3f}"
                )
            )
        )
        self.cloud_callback(cloud)

    def dynamic_targets_callback(self, msg: PoseArray) -> None:
        targets: list[PoseStamped] = []
        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = pose
            targets.append(stamped)
        self.dynamic_targets = targets
        self.dynamic_targets_time = self._now()

    def cloud_callback(self, msg: PointCloud2) -> None:
        self.last_cloud_time = self._now()
        if not self.bird_confirmed:
            self._reject("FUSION_INVALID_NO_BIRD reason=bird_not_confirmed")
            return
        calibration_reason = self._calibration_invalid_reason()
        if calibration_reason:
            self._reject(calibration_reason)
            return
        if self.camera_info is None or self._now() - self.camera_info_time > float(self.get_parameter("camera_info_stale_sec").value):
            self._reject("FUSION_INVALID_SYNC_STALE reason=camera_info_stale")
            return
        if not self.camera_info.header.frame_id:
            self._reject("FUSION_INVALID_NO_EXTRINSIC reason=camera_info_frame_empty")
            return
        if bool(self.get_parameter("require_camera_optical_frame").value) and "optical" not in self.camera_info.header.frame_id:
            self._reject(f"FUSION_INVALID_NO_EXTRINSIC reason=camera_frame_not_optical frame={self.camera_info.header.frame_id}")
            return
        if self.detections is None or self._now() - self.detections_time > float(self.get_parameter("detections_stale_sec").value):
            self._reject("FUSION_INVALID_SYNC_STALE reason=detection_stale")
            return
        if self._stamp_delta_sec(msg.header.stamp, self.detections.header.stamp) > float(self.get_parameter("max_sync_dt_sec").value):
            self._reject("FUSION_INVALID_SYNC_STALE reason=sync_dt_too_large")
            return
        detection = self._best_detection(self.detections)
        if detection is None:
            self._reject("FUSION_INVALID_NO_BIRD reason=no_bird_bbox")
            return
        try:
            centroid_camera = self._centroid_from_cloud(msg, detection, self.camera_info)
            base_pose = self._pose_in_frame(centroid_camera, self.camera_info.header.frame_id, str(self.get_parameter("base_frame").value), msg.header.stamp)
            if base_pose is None:
                self._reject("FUSION_INVALID_TF_FAIL target=base")
                return
            height = base_pose.pose.position.z - float(self.get_parameter("ground_z_offset_m").value)
            target_range = math.hypot(base_pose.pose.position.x, base_pose.pose.position.y)
            if height < float(self.get_parameter("min_height_m").value) or height > float(self.get_parameter("max_height_m").value):
                self._reject(f"FUSION_INVALID_HEIGHT height={height:.3f}")
                return
            if target_range < float(self.get_parameter("min_range_m").value) or target_range > float(self.get_parameter("max_range_m").value):
                self._reject(f"FUSION_INVALID_HEIGHT reason=range_invalid range={target_range:.3f}")
                return
            map_pose = self._pose_in_frame(centroid_camera, self.camera_info.header.frame_id, str(self.get_parameter("global_frame").value), msg.header.stamp)
            if map_pose is None:
                self._reject("FUSION_INVALID_TF_FAIL target=map")
                return
            dynamic_ok, dynamic_reason = self._associated_dynamic_target(map_pose)
            if bool(self.get_parameter("require_dynamic_valid").value) and not dynamic_ok:
                self._reject(f"FUSION_INVALID_DYNAMIC_ASSOCIATION {dynamic_reason}")
                return
            self.valid_pub.publish(Bool(data=True))
            self.pose_base_pub.publish(base_pose)
            self.pose_map_pub.publish(map_pose)
            self._publish_marker(map_pose)
            self.state_pub.publish(
                String(
                    data=(
                        "FUSION_VALID bird_confirmed=true z_valid=true dynamic_valid=true "
                        f"height={height:.3f} range={target_range:.3f} source=PointCloud2 "
                        f"dynamic_reason={dynamic_reason}"
                    )
                )
            )
        except Exception as exc:
            text = str(exc)
            if "insufficient" in text:
                self._reject(f"FUSION_INVALID_INSUFFICIENT_POINTS {text}")
            elif "spread too large" in text:
                self._reject(f"FUSION_INVALID_SPREAD_TOO_LARGE {text}")
            elif "tf2" in text.lower() or "transform" in text.lower():
                self._reject(f"FUSION_INVALID_TF_FAIL error={text}")
            else:
                self._reject(f"FUSION_INVALID_SYNC_STALE error={text}")

    def _calibration_invalid_reason(self) -> str:
        path = os.path.expanduser(str(self.get_parameter("camera_lidar_extrinsic_path").value))
        if not os.path.exists(path):
            return f"FUSION_INVALID_NO_EXTRINSIC path={path}"
        if not bool(self.get_parameter("require_calibrated_extrinsic").value):
            return ""
        try:
            import yaml

            data = yaml.safe_load(open(path, "r", encoding="utf-8")) or {}
        except Exception as exc:
            return f"FUSION_INVALID_NO_EXTRINSIC error={exc}"
        if data.get("calibrated") is not True:
            return f"FUSION_INVALID_CALIBRATION_NOT_VERIFIED path={path}"
        return ""

    def _best_detection(self, detections: Detection2DArray) -> Detection2D | None:
        best = None
        best_score = -1.0
        for det in detections.detections:
            if not det.results:
                continue
            score = max(float(result.hypothesis.score) for result in det.results)
            if score > best_score:
                best = det
                best_score = score
        return best

    def _centroid_from_cloud(self, msg: PointCloud2, detection: Detection2D, info: CameraInfo) -> tuple[float, float, float]:
        camera_frame = info.header.frame_id
        transform = None
        if msg.header.frame_id != camera_frame:
            if self.tf_buffer is None:
                raise RuntimeError("tf2 unavailable")
            transform = self.tf_buffer.lookup_transform(camera_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("invalid camera intrinsics")
        half_w = float(detection.bbox.size_x) * 0.5
        half_h = float(detection.bbox.size_y) * 0.5
        min_u = float(detection.bbox.center.x) - half_w
        max_u = float(detection.bbox.center.x) + half_w
        min_v = float(detection.bbox.center.y) - half_h
        max_v = float(detection.bbox.center.y) + half_h
        selected: list[tuple[float, float, float]] = []
        max_points = int(self.get_parameter("max_points").value)
        for count, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if count >= max_points:
                break
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            if transform is not None:
                x, y, z = self._transform_xyz(x, y, z, transform)
            if z <= 0.05:
                continue
            u = fx * (x / z) + cx
            v = fy * (y / z) + cy
            if min_u <= u <= max_u and min_v <= v <= max_v:
                selected.append((x, y, z))
        if len(selected) < int(self.get_parameter("min_points_in_bbox").value):
            raise RuntimeError(f"insufficient bbox point count={len(selected)}")
        xs = sorted(p[0] for p in selected)
        ys = sorted(p[1] for p in selected)
        zs = sorted(p[2] for p in selected)
        spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        if spread > float(self.get_parameter("max_bbox_point_spread_m").value):
            raise RuntimeError(f"bbox point spread too large spread={spread:.3f}")
        mid = len(selected) // 2
        return xs[mid], ys[mid], zs[mid]

    def _pose_in_frame(self, xyz: tuple[float, float, float], source_frame: str, target_frame: str, stamp) -> PoseStamped | None:
        x, y, z = xyz
        if source_frame != target_frame:
            if self.tf_buffer is None:
                return None
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time.from_msg(stamp))
                x, y, z = self._transform_xyz(x, y, z, transform)
            except TransformException:
                return None
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = target_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _transform_xyz(x: float, y: float, z: float, transform) -> tuple[float, float, float]:
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
        return rx + float(tx), ry + float(ty), rz + float(tz)

    def _publish_marker(self, pose: PoseStamped) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "bird_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

    def _associated_dynamic_target(self, map_pose: PoseStamped) -> tuple[bool, str]:
        if self._now() - self.dynamic_targets_time > float(self.get_parameter("max_dynamic_target_age_sec").value):
            return False, "DYNAMIC_TARGET_STALE"
        radius = float(self.get_parameter("dynamic_association_radius_m").value)
        best = math.inf
        for target in self.dynamic_targets:
            target_map = target if target.header.frame_id == map_pose.header.frame_id else self._transform_pose(target, map_pose.header.frame_id)
            if target_map is None:
                continue
            dx = target_map.pose.position.x - map_pose.pose.position.x
            dy = target_map.pose.position.y - map_pose.pose.position.y
            dz = target_map.pose.position.z - map_pose.pose.position.z
            best = min(best, math.sqrt(dx * dx + dy * dy + dz * dz))
        if best <= radius:
            return True, f"DYNAMIC_ASSOCIATED distance={best:.3f}"
        return False, f"DYNAMIC_ASSOCIATION_FAILED nearest={best if math.isfinite(best) else -1.0:.3f} radius={radius:.3f}"

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped | None:
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, Time.from_msg(pose.header.stamp))
            x, y, z = self._transform_xyz(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, transform)
        except Exception:
            return None
        out = PoseStamped()
        out.header.stamp = pose.header.stamp
        out.header.frame_id = target_frame
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.position.z = z
        out.pose.orientation = pose.pose.orientation
        return out

    @staticmethod
    def _stamp_delta_sec(a, b) -> float:
        if (a.sec == 0 and a.nanosec == 0) or (b.sec == 0 and b.nanosec == 0):
            return 0.0
        return abs((float(a.sec) + float(a.nanosec) * 1e-9) - (float(b.sec) + float(b.nanosec) * 1e-9))

    def _reject(self, reason: str) -> None:
        self.rejected_packet_count += 1
        self.valid_pub.publish(Bool(data=False))
        text = reason if reason.startswith("FUSION_INVALID_") else f"FUSION_INVALID_SYNC_STALE reason={reason}"
        self.state_pub.publish(String(data=text))
        self.sync_state_pub.publish(
            String(data=f"SYNC_STATE packets={self.synced_packet_count} rejected={self.rejected_packet_count} last_reason='{text}'")
        )

    def health_tick(self) -> None:
        if self.last_cloud_time and self._now() - self.last_cloud_time > float(self.get_parameter("pointcloud_stale_sec").value):
            self._reject("FUSION_INVALID_SYNC_STALE reason=pointcloud_stale")

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Bird3DFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

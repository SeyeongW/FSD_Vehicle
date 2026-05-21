from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


@dataclass
class Track:
    point: PointStamped
    last_seen: float
    first_seen: float
    last_bearing: float
    confirm_count: int = 1
    moving: bool = False
    velocity_mps: float = 0.0
    bearing_rate_dps: float = 0.0


class LidarAerialMotionDetectorNode(Node):
    """Select moving elevated object candidates from 3D object interfaces.

    2D LaserScan has no z value, so it cannot classify aerial objects by height.
    Use PoseArray, PointStamped, or PointCloud2-derived object centers for aerial tracking.
    PoseArray alone has no bbox/class/size metadata, so scoring remains heuristic.
    """

    def __init__(self) -> None:
        super().__init__("lidar_aerial_motion_detector_node")
        self.declare_parameter("lidar_objects_topic", "/waver/lidar_objects")
        self.declare_parameter("detections_topic", "/lidar/detections")
        self.declare_parameter("object_point_topic", "/waver/object_point")
        self.declare_parameter("pointcloud_topic", "/points")
        self.declare_parameter("enable_pointcloud_input", False)
        self.declare_parameter("target_topic", "/waver/aerial_target")
        self.declare_parameter("active_topic", "/waver/aerial_target_active")
        self.declare_parameter("state_topic", "/waver/aerial_motion_state")
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("lateral_axis", "y", axis_descriptor)
        self.declare_parameter("depth_axis", "x", axis_descriptor)
        self.declare_parameter("height_axis", "z", axis_descriptor)
        self.declare_parameter("positive_lateral_is_left", True)
        self.declare_parameter("reject_negative_depth", True)
        self.declare_parameter("min_height_m", 0.35)
        self.declare_parameter("max_height_m", 8.0)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 20.0)
        self.declare_parameter("min_motion_m", 0.08)
        self.declare_parameter("motion_min_mps", 0.10)
        self.declare_parameter("bearing_motion_min_deg_per_sec", 3.0)
        self.declare_parameter("track_match_gate_m", 1.0)
        self.declare_parameter("motion_confirm_count", 2)
        self.declare_parameter("target_hold_sec", 1.0)
        self.declare_parameter("lost_timeout_sec", 1.5)
        self.declare_parameter("lock_switch_cooldown_sec", 1.0)
        self.declare_parameter("prefer_existing_lock", True)
        self.declare_parameter("timer_hz", 10.0)

        self.target_pub = self.create_publisher(PointStamped, str(self.get_parameter("target_topic").value), 10)
        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("active_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.tracks: list[Track] = []
        self.locked: Track | None = None
        self.last_lock_switch = -1e9
        self.pointcloud_warned = False

        self.create_subscription(PoseArray, str(self.get_parameter("lidar_objects_topic").value), self.pose_array_callback, 10)
        self.create_subscription(PoseArray, str(self.get_parameter("detections_topic").value), self.pose_array_callback, 10)
        self.create_subscription(PointStamped, str(self.get_parameter("object_point_topic").value), self.point_callback, 10)
        if bool(self.get_parameter("enable_pointcloud_input").value):
            self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), self.pointcloud_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def pose_array_callback(self, msg: PoseArray) -> None:
        now = self._now()
        candidates: list[PointStamped] = []
        for pose in msg.poses:
            ps = PointStamped()
            ps.header = msg.header
            ps.point = pose.position
            if self._passes_geometry(ps.point):
                candidates.append(ps)
        self._update_tracks(candidates, now, msg.header.frame_id)

    def point_callback(self, msg: PointStamped) -> None:
        now = self._now()
        candidates = [msg] if self._passes_geometry(msg.point) else []
        self._update_tracks(candidates, now, msg.header.frame_id)

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        if not self.pointcloud_warned:
            self.get_logger().warn("Raw PointCloud2 is not handled here; enable pointcloud_lidar_objects_node instead")
            self.pointcloud_warned = True
        self.locked = None
        self.active_pub.publish(Bool(data=False))
        self.state_pub.publish(String(data=f"POINTCLOUD_HANDLER_NOT_ENABLED frame={msg.header.frame_id} active=false"))

    def tick(self) -> None:
        now = self._now()
        self.tracks = [t for t in self.tracks if now - t.last_seen <= float(self.get_parameter("lost_timeout_sec").value)]
        if self.locked and now - self.locked.last_seen <= float(self.get_parameter("target_hold_sec").value):
            self.target_pub.publish(self.locked.point)
            self.active_pub.publish(Bool(data=True))
            p = self.locked.point.point
            self.state_pub.publish(
                String(data=f"LOCKED_MOVING_TARGET frame={self.locked.point.header.frame_id} x={p.x:.2f} y={p.y:.2f} z={p.z:.2f} v={self.locked.velocity_mps:.2f}")
            )
            return
        best = self._best_confirmed_track()
        if best:
            if self.locked is not best and now - self.last_lock_switch < float(self.get_parameter("lock_switch_cooldown_sec").value):
                best = self.locked if self.locked in self.tracks else best
            if best is not self.locked:
                self.last_lock_switch = now
            self.locked = best
            self.target_pub.publish(best.point)
            self.active_pub.publish(Bool(data=True))
            self.state_pub.publish(String(data=f"ACTIVE_MOVING_TARGET frame={best.point.header.frame_id} score={self._score(best):.3f}"))
        else:
            self.active_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data=f"NO_CONFIRMED_AERIAL_TARGET tracks={len(self.tracks)}"))

    def _update_tracks(self, candidates: list[PointStamped], now: float, frame_id: str) -> None:
        for candidate in candidates:
            track = self._nearest_track(candidate)
            bearing = self._bearing(candidate.point)
            if track is None:
                self.tracks.append(Track(candidate, now, now, bearing))
                continue
            dt = max(now - track.last_seen, 1e-3)
            movement = self._distance(track.point.point, candidate.point)
            velocity = movement / dt
            bearing_rate = abs(math.degrees((bearing - track.last_bearing + math.pi) % (2.0 * math.pi) - math.pi)) / dt
            track.velocity_mps = velocity
            track.bearing_rate_dps = bearing_rate
            moved = (
                movement >= float(self.get_parameter("min_motion_m").value)
                or velocity >= float(self.get_parameter("motion_min_mps").value)
                or bearing_rate >= float(self.get_parameter("bearing_motion_min_deg_per_sec").value)
            )
            track.moving = track.moving or moved
            track.confirm_count += 1 if moved else 0
            track.point = candidate
            track.last_seen = now
            track.last_bearing = bearing
        self.state_pub.publish(String(data=f"CANDIDATES frame={frame_id} count={len(candidates)} tracks={len(self.tracks)}"))

    def _nearest_track(self, candidate: PointStamped) -> Track | None:
        gate = float(self.get_parameter("track_match_gate_m").value)
        best: Track | None = None
        best_dist = math.inf
        for track in self.tracks:
            dist = self._distance(track.point.point, candidate.point)
            if dist < best_dist:
                best = track
                best_dist = dist
        return best if best is not None and best_dist <= gate else None

    def _best_confirmed_track(self) -> Track | None:
        if bool(self.get_parameter("prefer_existing_lock").value) and self.locked in self.tracks:
            if self.locked and self.locked.moving and self.locked.confirm_count >= int(self.get_parameter("motion_confirm_count").value):
                return self.locked
        confirmed = [t for t in self.tracks if t.moving and t.confirm_count >= int(self.get_parameter("motion_confirm_count").value)]
        if not confirmed:
            return None
        return min(confirmed, key=self._score)

    def _score(self, track: Track) -> float:
        depth = abs(self._axis_value(track.point.point, self._axis_param("depth_axis", "x")))
        bearing = abs(self._bearing(track.point.point))
        return 0.7 * bearing + 0.05 * depth - 0.2 * min(track.velocity_mps, 2.0)

    def _passes_geometry(self, point: Point) -> bool:
        height = self._axis_value(point, self._axis_param("height_axis", "z"))
        depth = self._axis_value(point, self._axis_param("depth_axis", "x"))
        if bool(self.get_parameter("reject_negative_depth").value) and depth < 0.0:
            return False
        return (
            float(self.get_parameter("min_height_m").value) <= height <= float(self.get_parameter("max_height_m").value)
            and float(self.get_parameter("min_depth_m").value) <= abs(depth) <= float(self.get_parameter("max_depth_m").value)
        )

    def _bearing(self, point: Point) -> float:
        lateral = self._axis_value(point, self._axis_param("lateral_axis", "y"))
        depth = self._axis_value(point, self._axis_param("depth_axis", "x"))
        if not bool(self.get_parameter("positive_lateral_is_left").value):
            lateral *= -1.0
        return math.atan2(lateral, max(depth, 1e-3))

    @staticmethod
    def _axis_value(point: Point, axis: str) -> float:
        return float(getattr(point, axis, 0.0))

    def _axis_param(self, name: str, default: str) -> str:
        """Return a valid axis name even if ROS YAML parsed bare `y` as true."""
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return default
        text = str(value).strip()
        if text in {"x", "y", "z"}:
            return text
        self.get_logger().warn(f"Invalid {name}={text!r}; using {default}")
        return default

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarAerialMotionDetectorNode()
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

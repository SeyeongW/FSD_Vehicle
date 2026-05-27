from __future__ import annotations

import math
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def _axis_value(point: object, axis: str) -> float:
    sign = -1.0 if axis.startswith("-") else 1.0
    axis = axis[1:] if axis.startswith("-") else axis
    mapping = {"x": 0, "y": 1, "z": 2}
    return sign * float(point[mapping.get(axis, 0)])


class LivoxPointCloudToScanNode(Node):
    """Project a 3D point cloud into a conservative 2D safety scan.

    Role:
      - Let Gazebo Livox `/mid360` or a real 3D LiDAR feed Waver's final safety mux.
      - Keep aerial/bird detection separate: this node only creates a ground-obstacle
        LaserScan for stop/slow-down logic, not a height classifier.
      - Distinguish a healthy clear outdoor scene from a broken scan. Healthy clear
        rays are published as range_max; stale/empty sensor data is reported as
        DEGRADED_* so the safety mux can stop.
    """

    def __init__(self) -> None:
        super().__init__("livox_pointcloud_to_scan_node")
        self.declare_parameter("pointcloud_topic", "/mid360")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("state_topic", "/waver/livox_scan_adapter_state")
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("forward_axis", "x", axis_descriptor)
        self.declare_parameter("lateral_axis", "y", axis_descriptor)
        self.declare_parameter("height_axis", "z", axis_descriptor)
        self.declare_parameter("positive_lateral_is_left", True)
        self.declare_parameter("obstacle_min_height_m", -0.35)
        self.declare_parameter("obstacle_max_height_m", 0.80)
        self.declare_parameter("range_min_m", 0.08)
        self.declare_parameter("range_max_m", 12.0)
        self.declare_parameter("angle_min_rad", -math.pi)
        self.declare_parameter("angle_max_rad", math.pi)
        self.declare_parameter("angle_increment_rad", math.radians(1.0))
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("min_raw_points", 50)
        self.declare_parameter("min_used_points", 10)
        self.declare_parameter("clear_empty_obstacle_scan", True)

        self.scan_pub = self.create_publisher(LaserScan, str(self.get_parameter("scan_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), self.cloud_callback, 5)

    def cloud_callback(self, msg: PointCloud2) -> None:
        try:
            if not msg.header.frame_id:
                raise RuntimeError("PointCloud2 header.frame_id is empty")
            scan, used, total = self._cloud_to_scan(msg)
            if total < int(self.get_parameter("min_raw_points").value):
                state = "DEGRADED_LOW_COVERAGE"
            elif used > 0:
                state = "OK_OBSTACLE"
            else:
                state = "OK_CLEAR"
        except Exception as exc:
            self.get_logger().warn(f"PointCloud2 projection failed; publishing degraded scan: {exc}")
            scan = self._empty_scan(msg, clear=False)
            used = 0
            total = 0
            state = "DEGRADED_TF_FAIL" if "TF" in str(exc).upper() else "DEGRADED_FRAME_EMPTY"
        self.scan_pub.publish(scan)
        self.state_pub.publish(String(data=f"{state} frame={scan.header.frame_id} obstacle_points={used} raw_points={total}"))

    def _cloud_to_scan(self, msg: PointCloud2) -> tuple[LaserScan, int, int]:
        scan = self._empty_scan(msg, clear=bool(self.get_parameter("clear_empty_obstacle_scan").value))
        bins = len(scan.ranges)
        used = 0
        total = 0
        max_points = int(self.get_parameter("max_points").value)
        points: Iterable[object] = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            total += 1
            if total > max_points:
                break
            forward = _axis_value(point, self._axis_param("forward_axis", "x"))
            lateral = _axis_value(point, self._axis_param("lateral_axis", "y"))
            height = _axis_value(point, self._axis_param("height_axis", "z"))
            if not bool(self.get_parameter("positive_lateral_is_left").value):
                lateral *= -1.0
            if not all(math.isfinite(v) for v in (forward, lateral, height)):
                continue
            if height < float(self.get_parameter("obstacle_min_height_m").value):
                continue
            if height > float(self.get_parameter("obstacle_max_height_m").value):
                continue
            distance = math.hypot(forward, lateral)
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = math.atan2(lateral, forward)
            if angle < scan.angle_min or angle > scan.angle_max:
                continue
            index = int((angle - scan.angle_min) / scan.angle_increment)
            if 0 <= index < bins and distance < scan.ranges[index]:
                scan.ranges[index] = distance
                used += 1
        return scan, used, total

    def _empty_scan(self, msg: PointCloud2, clear: bool = True) -> LaserScan:
        angle_min = float(self.get_parameter("angle_min_rad").value)
        angle_max = float(self.get_parameter("angle_max_rad").value)
        angle_increment = max(float(self.get_parameter("angle_increment_rad").value), math.radians(0.25))
        bins = max(1, int(math.ceil((angle_max - angle_min) / angle_increment)))
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = angle_min
        scan.angle_max = angle_min + angle_increment * (bins - 1)
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = float(self.get_parameter("range_min_m").value)
        scan.range_max = float(self.get_parameter("range_max_m").value)
        fill = scan.range_max if clear else math.inf
        scan.ranges = [fill] * bins
        return scan

    def _axis_param(self, name: str, default: str) -> str:
        """Return a valid axis name even if YAML/launch parsed `y` as boolean."""
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return default
        text = str(value).strip()
        if text in {"x", "y", "z", "-x", "-y", "-z"}:
            return text
        self.get_logger().warn(f"Invalid {name}={text!r}; using {default}")
        return default


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LivoxPointCloudToScanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

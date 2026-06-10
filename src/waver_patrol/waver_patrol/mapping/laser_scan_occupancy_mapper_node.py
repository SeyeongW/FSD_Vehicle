from __future__ import annotations

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LaserScanOccupancyMapperNode(Node):
    """Small Gazebo mapping backend that accumulates LaserScan endpoints.

    This node is intentionally simple: it uses odometry as the map frame and
    ray-traces `/scan_slam` into an OccupancyGrid.  It is for Gazebo UI mapping
    validation where slam_toolbox may publish free-space but too few occupied
    endpoints for the static-obstacle regression test.
    """

    def __init__(self) -> None:
        super().__init__("laser_scan_occupancy_mapper_node")
        self.declare_parameter("scan_topic", "/scan_slam")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("state_topic", "/waver/mapping_backend_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("extent_m", 30.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("ray_step", 2)
        self.declare_parameter("occupied_inflate_cells", 1)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("publish_map_to_odom_tf", True)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.width = max(1, int(math.ceil(float(self.get_parameter("extent_m").value) / self.resolution)))
        self.height = self.width
        cx = float(self.get_parameter("center_x").value)
        cy = float(self.get_parameter("center_y").value)
        self.origin_x = cx - 0.5 * self.width * self.resolution
        self.origin_y = cy - 0.5 * self.height * self.resolution
        self.grid = [-1] * (self.width * self.height)
        self.odom: Odometry | None = None
        self.scan_count = 0
        self.last_scan_stamp = self.get_clock().now()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, str(self.get_parameter("map_topic").value), qos)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.publish_map)
        self.get_logger().info(
            f"LaserScan mapper active: {self.width}x{self.height} res={self.resolution:.3f} "
            f"topic={self.get_parameter('scan_topic').value}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        if self.odom is None:
            self.state_pub.publish(String(data="WAITING_FOR_ODOM"))
            return
        pose = self.odom.pose.pose
        rx = float(pose.position.x)
        ry = float(pose.position.y)
        ryaw = yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        max_range = min(float(self.get_parameter("max_range_m").value), float(msg.range_max))
        ray_step = max(1, int(self.get_parameter("ray_step").value))
        occupied_written = 0
        free_written = 0
        finite_count = 0
        for index in range(0, len(msg.ranges), ray_step):
            raw_range = float(msg.ranges[index])
            if not math.isfinite(raw_range) or raw_range < float(msg.range_min):
                continue
            finite_count += 1
            used_range = min(raw_range, max_range)
            angle = ryaw + float(msg.angle_min) + float(index) * float(msg.angle_increment)
            ex = rx + math.cos(angle) * used_range
            ey = ry + math.sin(angle) * used_range
            start = self.world_to_cell(rx, ry)
            end = self.world_to_cell(ex, ey)
            if start is None or end is None:
                continue
            cells = list(self.bresenham(start[0], start[1], end[0], end[1]))
            if len(cells) > 1:
                for cx, cy in cells[:-1]:
                    offset = cy * self.width + cx
                    if self.grid[offset] < 0:
                        self.grid[offset] = 0
                        free_written += 1
            if raw_range <= max_range - 0.05:
                occupied_written += self.mark_occupied(end[0], end[1])
        self.scan_count += 1
        self.last_scan_stamp = self.get_clock().now()
        self.state_pub.publish(
            String(
                data=(
                    f"SCAN_MAPPER_OK scans={self.scan_count} finite={finite_count} "
                    f"free_written={free_written} occupied_written={occupied_written}"
                )
            )
        )

    def publish_map(self) -> None:
        now = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = now
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.grid)
        self.map_pub.publish(msg)
        if bool(self.get_parameter("publish_map_to_odom_tf").value):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.odom_frame
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def mark_occupied(self, cx: int, cy: int) -> int:
        radius = max(0, int(self.get_parameter("occupied_inflate_cells").value))
        written = 0
        for yy in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
            for xx in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                offset = yy * self.width + xx
                if self.grid[offset] != 100:
                    self.grid[offset] = 100
                    written += 1
        return written

    @staticmethod
    def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[tuple[int, int]]:
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaserScanOccupancyMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()

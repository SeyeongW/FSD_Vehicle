#!/usr/bin/env python3
"""
PointCloud2 -> LaserScan(/scan) + 2D OccupancyGrid(/projected_map)

ROS 2 Humble용 노드

목적:
- Gazebo 또는 실제 MID360/Livox 3D PointCloud2 데이터를 입력받음
- base_link 기준 높이 필터로 지면 제거
- 남은 장애물 점을 LaserScan(/scan)으로 변환
- 동시에 odom 기준 XY 평면에 누적 투영하여 OccupancyGrid(/projected_map) 발행

사용 예:
ros2 run 패키지명 pointcloud_to_2d_map_node --ros-args \
  -p input_topic:=/mid360_PointCloud2 \
  -p scan_topic:=/scan \
  -p map_topic:=/projected_map \
  -p scan_frame:=base_link \
  -p target_frame:=odom \
  -p min_obstacle_height:=0.10 \
  -p max_obstacle_height:=1.50
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


def transform_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation

    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    mat[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float32)

    return mat


class PointCloudToScanAndMapNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan_and_map_node')

        # ===== Topics / frames =====
        self.declare_parameter('input_topic', '/mid360_PointCloud2')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/projected_map')

        # scan_frame: LaserScan이 발행될 좌표계
        # Cartographer 설정에서 tracking_frame/base_frame과 맞추는 게 중요함
        self.declare_parameter('scan_frame', 'base_link')

        # target_frame: 2D 투영 맵을 만들 기준 좌표계
        # Gazebo에서는 odom 추천
        self.declare_parameter('target_frame', 'odom')

        self.declare_parameter('enable_scan', True)
        self.declare_parameter('enable_map', True)

        # ===== Height / range filter =====
        # scan_frame 기준 z 높이로 지면 제거
        self.declare_parameter('min_obstacle_height', 0.10)
        self.declare_parameter('max_obstacle_height', 1.50)

        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 20.0)

        # ===== LaserScan settings =====
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0058)
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('use_inf', True)
        self.declare_parameter('inf_epsilon', 1.0)

        # ===== OccupancyGrid settings =====
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 30.0)
        self.declare_parameter('height_m', 30.0)
        self.declare_parameter('origin_x', -15.0)
        self.declare_parameter('origin_y', -15.0)

        self.declare_parameter('hit_threshold', 2)
        self.declare_parameter('hit_increment', 1)
        self.declare_parameter('hit_max', 100)

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('publish_unknown_as_free', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.enable_scan = bool(self.get_parameter('enable_scan').value)
        self.enable_map = bool(self.get_parameter('enable_map').value)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)

        self.width_cells = int(math.ceil(self.width_m / self.resolution))
        self.height_cells = int(math.ceil(self.height_m / self.resolution))

        if self.width_cells <= 0 or self.height_cells <= 0:
            raise ValueError('Invalid map size.')

        self.hit_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.uint16)
        self.seen_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.bool_)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            10
        )

        self.scan_pub = None
        if self.enable_scan:
            self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.map_pub = None
        if self.enable_map:
            self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)

        publish_rate = float(self.get_parameter('publish_rate').value)
        timer_period = 1.0 / max(publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.publish_map)

        self.last_stamp = self.get_clock().now().to_msg()
        self.received_cloud_count = 0

        self.get_logger().info(f'Subscribed PointCloud2: {self.input_topic}')

        if self.enable_scan:
            self.get_logger().info(f'Publishing LaserScan: {self.scan_topic}')
            self.get_logger().info(f'LaserScan frame: {self.scan_frame}')

        if self.enable_map:
            self.get_logger().info(f'Publishing OccupancyGrid: {self.map_topic}')
            self.get_logger().info(f'Map target frame: {self.target_frame}')
            self.get_logger().info(
                f'Map size: {self.width_m:.1f} x {self.height_m:.1f} m, '
                f'resolution={self.resolution:.3f}, '
                f'cells={self.width_cells} x {self.height_cells}'
            )

    def lookup_transform_matrix(self, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float32)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            return transform_to_matrix(tf_msg)

        except TransformException as ex:
            self.get_logger().warn(
                f'No TF from {source_frame} to {target_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return None

    def cloud_callback(self, msg: PointCloud2):
        self.received_cloud_count += 1
        self.last_stamp = msg.header.stamp

        source_frame = msg.header.frame_id

        if not source_frame:
            self.get_logger().warn(
                'PointCloud2 header.frame_id is empty.',
                throttle_duration_sec=2.0
            )
            return

        # source_frame -> scan_frame 변환
        tf_scan_from_source = self.lookup_transform_matrix(self.scan_frame, source_frame)
        if tf_scan_from_source is None:
            return

        try:
            pts_iter = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            points = np.array([[p[0], p[1], p[2]] for p in pts_iter], dtype=np.float32)

            if points.size == 0:
                return

            finite_mask = np.isfinite(points).all(axis=1)
            points = points[finite_mask]

            if points.size == 0:
                return

            # sensor frame 기준 거리 필터
            range_min = float(self.get_parameter('range_min').value)
            range_max = float(self.get_parameter('range_max').value)

            sensor_ranges = np.linalg.norm(points[:, :2], axis=1)
            range_mask = (sensor_ranges >= range_min) & (sensor_ranges <= range_max)
            points = points[range_mask]

            if points.size == 0:
                return

            # scan_frame으로 변환
            ones = np.ones((points.shape[0], 1), dtype=np.float32)
            points_h = np.hstack((points, ones))
            points_scan = (tf_scan_from_source @ points_h.T).T[:, :3]

            # scan_frame 기준 높이 필터
            min_h = float(self.get_parameter('min_obstacle_height').value)
            max_h = float(self.get_parameter('max_obstacle_height').value)

            z = points_scan[:, 2]
            height_mask = (z >= min_h) & (z <= max_h)
            points_scan = points_scan[height_mask]

            if points_scan.size == 0:
                return

            if self.enable_scan:
                self.publish_scan(points_scan, msg.header.stamp)

            if self.enable_map:
                self.update_projected_map(points_scan)

        except Exception as ex:
            self.get_logger().error(f'Failed to process point cloud: {ex}')

    def publish_scan(self, points_scan: np.ndarray, stamp):
        angle_min = float(self.get_parameter('angle_min').value)
        angle_max = float(self.get_parameter('angle_max').value)
        angle_increment = float(self.get_parameter('angle_increment').value)

        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        scan_time = float(self.get_parameter('scan_time').value)

        use_inf = bool(self.get_parameter('use_inf').value)
        inf_epsilon = float(self.get_parameter('inf_epsilon').value)

        if angle_max <= angle_min:
            self.get_logger().error('angle_max must be greater than angle_min.')
            return

        num_readings = int(math.ceil((angle_max - angle_min) / angle_increment))

        if num_readings <= 0:
            self.get_logger().error('Invalid number of LaserScan bins.')
            return

        if use_inf:
            scan_ranges = np.full(num_readings, np.inf, dtype=np.float32)
        else:
            scan_ranges = np.full(num_readings, range_max + inf_epsilon, dtype=np.float32)

        x = points_scan[:, 0]
        y = points_scan[:, 1]

        ranges = np.sqrt(x * x + y * y)
        angles = np.arctan2(y, x)

        valid = (
            (ranges >= range_min) &
            (ranges <= range_max) &
            (angles >= angle_min) &
            (angles < angle_max)
        )

        if not np.any(valid):
            return

        ranges = ranges[valid]
        angles = angles[valid]

        indices = ((angles - angle_min) / angle_increment).astype(np.int32)
        inside = (indices >= 0) & (indices < num_readings)

        indices = indices[inside]
        ranges = ranges[inside]

        if indices.size == 0:
            return

        # 같은 각도 bin에 여러 점이 있으면 가장 가까운 점 사용
        np.minimum.at(scan_ranges, indices, ranges)

        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.header.frame_id = self.scan_frame

        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = scan_time

        scan_msg.range_min = range_min
        scan_msg.range_max = range_max

        scan_msg.ranges = scan_ranges.tolist()
        scan_msg.intensities = []

        self.scan_pub.publish(scan_msg)

    def update_projected_map(self, points_scan: np.ndarray):
        # scan_frame -> target_frame 변환
        tf_target_from_scan = self.lookup_transform_matrix(self.target_frame, self.scan_frame)

        if tf_target_from_scan is None:
            return

        ones = np.ones((points_scan.shape[0], 1), dtype=np.float32)
        points_h = np.hstack((points_scan, ones))
        points_target = (tf_target_from_scan @ points_h.T).T[:, :3]

        gx = np.floor((points_target[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        gy = np.floor((points_target[:, 1] - self.origin_y) / self.resolution).astype(np.int32)

        inside = (
            (gx >= 0) & (gx < self.width_cells) &
            (gy >= 0) & (gy < self.height_cells)
        )

        gx = gx[inside]
        gy = gy[inside]

        if gx.size == 0:
            return

        cells = np.unique(np.stack((gy, gx), axis=1), axis=0)

        cy = cells[:, 0]
        cx = cells[:, 1]

        hit_increment = int(self.get_parameter('hit_increment').value)
        hit_max = int(self.get_parameter('hit_max').value)

        self.seen_grid[cy, cx] = True

        new_values = self.hit_grid[cy, cx].astype(np.uint32) + hit_increment
        self.hit_grid[cy, cx] = np.minimum(new_values, hit_max).astype(np.uint16)

    def publish_map(self):
        if not self.enable_map or self.map_pub is None:
            return

        hit_threshold = int(self.get_parameter('hit_threshold').value)

        unknown_value = int(self.get_parameter('unknown_value').value)
        occupied_value = int(self.get_parameter('occupied_value').value)
        free_value = int(self.get_parameter('free_value').value)

        publish_unknown_as_free = bool(self.get_parameter('publish_unknown_as_free').value)

        if publish_unknown_as_free:
            occ = np.full((self.height_cells, self.width_cells), free_value, dtype=np.int8)
        else:
            occ = np.full((self.height_cells, self.width_cells), unknown_value, dtype=np.int8)
            occ[self.seen_grid] = free_value

        occ[self.hit_grid >= hit_threshold] = occupied_value

        msg = OccupancyGrid()
        msg.header.stamp = self.last_stamp
        msg.header.frame_id = self.target_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = occ.reshape(-1).tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudToScanAndMapNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
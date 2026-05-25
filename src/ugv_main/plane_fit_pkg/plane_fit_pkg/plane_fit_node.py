#!/usr/bin/env python3
import math
import time
import random
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class SimpleKalman:
    def __init__(self):
        self.x = None  # 상태 [a,b,c,d]
        self.P = np.eye(4) * 0.1
        self.Q = np.eye(4) * 0.001  # 프로세스 노이즈
        self.R = np.eye(4) * 0.01   # 측정 노이즈

    def update(self, z):
        z = np.asarray(z)

        if self.x is None:
            self.x = z
            return self.x

        # Predict
        x_pred = self.x
        P_pred = self.P + self.Q

        # Kalman Gain
        K = P_pred @ np.linalg.inv(P_pred + self.R)

        # Update
        self.x = x_pred + K @ (z - x_pred)
        self.P = (np.eye(4) - K) @ P_pred

        return self.x


def normalize_plane(plane):
    plane = np.asarray(plane, dtype=float)
    n = np.linalg.norm(plane[:3])
    if n < 1e-12:
        return plane
    return plane / n


def point_plane_distances(points, plane):
    plane = normalize_plane(plane)
    a, b, c, d = plane
    return np.abs(points @ np.array([a, b, c]) + d)


def fit_plane_from_3points(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        return None
    normal = normal / norm
    d = -np.dot(normal, p1)
    return normalize_plane([normal[0], normal[1], normal[2], d])


def least_squares_plane(points):
    """
    Orthogonal least squares plane fitting using SVD.
    Returns ax + by + cz + d = 0.
    """
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1, :]
    d = -np.dot(normal, centroid)
    return normalize_plane([normal[0], normal[1], normal[2], d])


def custom_ransac(points, threshold=0.05, iterations=500):
    best_plane = None
    best_inliers = np.array([], dtype=int)

    n_points = len(points)
    if n_points < 3:
        return None, best_inliers

    for _ in range(iterations):
        ids = random.sample(range(n_points), 3)
        plane = fit_plane_from_3points(points[ids[0]], points[ids[1]], points[ids[2]])
        if plane is None:
            continue

        distances = point_plane_distances(points, plane)
        inliers = np.where(distances < threshold)[0]

        if len(inliers) > len(best_inliers):
            best_plane = plane
            best_inliers = inliers

    return best_plane, best_inliers


def open3d_ransac(points, threshold=0.05, iterations=500):
    try:
        import open3d as o3d
    except ImportError:
        return None, np.array([], dtype=int)

    if len(points) < 3:
        return None, np.array([], dtype=int)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    plane, inliers = pcd.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=iterations
    )
    return normalize_plane(plane), np.asarray(inliers, dtype=int)


def angle_error_deg(plane, gt_normal):
    n = normalize_plane(plane)[:3]
    gt = np.asarray(gt_normal, dtype=float)
    gt = gt / (np.linalg.norm(gt) + 1e-12)

    # plane normal direction can be flipped, so use absolute dot product
    dot = abs(float(np.dot(n, gt)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def evaluate(points, plane, inliers, gt_normal):
    if plane is None or len(inliers) == 0:
        return {
            "inlier_ratio": 0.0,
            "rmse": float("nan"),
            "angle_error": float("nan"),
        }

    inlier_points = points[inliers]
    distances = point_plane_distances(inlier_points, plane)
    rmse = float(np.sqrt(np.mean(distances ** 2)))
    return {
        "inlier_ratio": float(len(inliers) / len(points)),
        "rmse": rmse,
        "angle_error": angle_error_deg(plane, gt_normal),
    }


class PlaneFitNode(Node):
    def __init__(self):
        super().__init__('plane_fit_node')

        self.kalman = SimpleKalman()

        self.declare_parameter('input_topic', '/mid360_PointCloud2')
        self.declare_parameter('target_mode', 'ground')  # ground, ramp, wall
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('ransac_iterations', 500)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 15000)
        self.declare_parameter('process_every_n', 5)
        self.declare_parameter('frame_id', '')  # empty: use cloud header frame
        self.declare_parameter('z_min', -2.0)
        self.declare_parameter('z_max', 5.0)
        self.declare_parameter('range_max', 25.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.target_mode = self.get_parameter('target_mode').value
        self.threshold = float(self.get_parameter('distance_threshold').value)
        self.iterations = int(self.get_parameter('ransac_iterations').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.process_every_n = int(self.get_parameter('process_every_n').value)
        self.fixed_frame_id = self.get_parameter('frame_id').value
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/plane_fit/markers', 10)

        self.count = 0
        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info('RViz MarkerArray topic: /plane_fit/markers')

    def gt_normal(self):
        # 기준 normal은 world 구성에 맞춰 사용.
        # ground: z=0, ramp: pitch 15 deg around Y, wall: vertical wall
        if self.target_mode == 'ground':
            return np.array([0.0, 0.0, 1.0])
        if self.target_mode == 'wall_x':
            return np.array([1.0, 0.0, 0.0])
        if self.target_mode == 'wall_y':
            return np.array([0.0, 1.0, 0.0])
        if self.target_mode == 'ramp':
            theta = math.radians(15.0)
            # local z normal rotated about Y-axis by +15 deg
            return np.array([math.sin(theta), 0.0, math.cos(theta)])
        return np.array([0.0, 0.0, 1.0])

    def cloud_to_numpy(self, msg):
        pts = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            r = math.sqrt(x*x + y*y + z*z)
            if self.z_min <= z <= self.z_max and r <= self.range_max:
                pts.append([x, y, z])

        points = np.asarray(pts, dtype=np.float64)
        if len(points) == 0:
            return points

        # simple voxel downsample without Open3D dependency
        if self.voxel_size > 0:
            keys = np.floor(points / self.voxel_size).astype(np.int64)
            _, unique_idx = np.unique(keys, axis=0, return_index=True)
            points = points[unique_idx]

        if len(points) > self.max_points:
            idx = np.random.choice(len(points), self.max_points, replace=False)
            points = points[idx]

        return points

    def cloud_callback(self, msg):
        self.count += 1
        if self.count % self.process_every_n != 0:
            return

        points = self.cloud_to_numpy(msg)
        if len(points) < 30:
            self.get_logger().warn('Not enough valid points.')
            return

        gt = self.gt_normal()
        results = []

        t0 = time.perf_counter()
        plane_o3d, inliers_o3d = open3d_ransac(points, self.threshold, self.iterations)
        time_o3d = time.perf_counter() - t0
        m_o3d = evaluate(points, plane_o3d, inliers_o3d, gt)
        results.append(('Open3D RANSAC', plane_o3d, inliers_o3d, time_o3d, m_o3d))

        t0 = time.perf_counter()
        plane_custom, inliers_custom = custom_ransac(points, self.threshold, self.iterations)
        time_custom = time.perf_counter() - t0
        m_custom = evaluate(points, plane_custom, inliers_custom, gt)
        results.append(('Custom RANSAC', plane_custom, inliers_custom, time_custom, m_custom))

        t0 = time.perf_counter()
       
        if plane_o3d is not None and len(inliers_o3d) >= 3:
            # 1) Open3D RANSAC의 inlier로 LS 평면 보정
            plane_ls_raw = least_squares_plane(points[inliers_o3d])

            # 2) Kalman Filter 적용
            plane_ls = self.kalman.update(plane_ls_raw)

            # 3) 평면 계수 정규화
            plane_ls = normalize_plane(plane_ls)

            # 4) Kalman 적용된 평면 기준으로 inlier 다시 계산
            inliers_ls = np.where(
                point_plane_distances(points, plane_ls) < self.threshold
            )[0]
        else:
            plane_ls = None
            inliers_ls = np.array([], dtype=int)

        time_ls = time_o3d + (time.perf_counter() - t0)
        m_ls = evaluate(points, plane_ls, inliers_ls, gt)
        results.append(('RANSAC + LS + Kalman', plane_ls, inliers_ls, time_ls, m_ls))

        log_lines = []
        for name, plane, inliers, elapsed, metrics in results:
            log_lines.append(
                f'{name}: ratio={metrics["inlier_ratio"]:.3f}, '
                f'RMSE={metrics["rmse"]:.4f} m, '
                f'angle={metrics["angle_error"]:.3f} deg, '
                f'time={elapsed*1000:.1f} ms'
            )
        self.get_logger().info(' | '.join(log_lines))

        frame_id = self.fixed_frame_id if self.fixed_frame_id else msg.header.frame_id
        self.publish_markers(frame_id, msg.header.stamp, points, results)

    def publish_markers(self, frame_id, stamp, points, results):
        ma = MarkerArray()

        # delete previous markers
        delete = Marker()
        delete.header.frame_id = frame_id
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        marker_id = 0

        # publish only best visual result: RANSAC + LS, and text for all algorithms
        name, plane, inliers, elapsed, metrics = results[-1]

        # Inlier points marker
        if plane is not None and len(inliers) > 0:
            inlier_marker = Marker()
            inlier_marker.header.frame_id = frame_id
            inlier_marker.header.stamp = stamp
            inlier_marker.ns = 'plane_fit_inliers'
            inlier_marker.id = marker_id
            marker_id += 1
            inlier_marker.type = Marker.POINTS
            inlier_marker.action = Marker.ADD
            inlier_marker.scale.x = 0.04
            inlier_marker.scale.y = 0.04
            inlier_marker.color.r = 0.0
            inlier_marker.color.g = 1.0
            inlier_marker.color.b = 0.0
            inlier_marker.color.a = 1.0

            sample = inliers
            if len(sample) > 3000:
                sample = np.random.choice(sample, 3000, replace=False)
            for p in points[sample]:
                inlier_marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ma.markers.append(inlier_marker)

            outlier_idx = np.setdiff1d(np.arange(len(points)), inliers)
            out_marker = Marker()
            out_marker.header.frame_id = frame_id
            out_marker.header.stamp = stamp
            out_marker.ns = 'plane_fit_outliers'
            out_marker.id = marker_id
            marker_id += 1
            out_marker.type = Marker.POINTS
            out_marker.action = Marker.ADD
            out_marker.scale.x = 0.035
            out_marker.scale.y = 0.035
            out_marker.color.r = 1.0
            out_marker.color.g = 0.0
            out_marker.color.b = 0.0
            out_marker.color.a = 1.0

            if len(outlier_idx) > 3000:
                outlier_idx = np.random.choice(outlier_idx, 3000, replace=False)
            for p in points[outlier_idx]:
                out_marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ma.markers.append(out_marker)

        # Text marker
        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = 'plane_fit_text'
        text.id = marker_id
        marker_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 2.0
        text.scale.z = 0.35
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        lines = []
        for alg_name, plane_i, inliers_i, elapsed_i, metrics_i in results:
            lines.append(
                f'{alg_name}: IR={metrics_i["inlier_ratio"]:.2f}, '
                f'RMSE={metrics_i["rmse"]:.3f}m, '
                f'Angle={metrics_i["angle_error"]:.1f}deg, '
                f'T={elapsed_i*1000:.1f}ms'
            )
        text.text = '\n'.join(lines)
        ma.markers.append(text)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PlaneFitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

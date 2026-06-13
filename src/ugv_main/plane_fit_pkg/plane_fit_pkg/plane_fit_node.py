#!/usr/bin/env python3
import math
import time
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def normalize_plane(plane):
    plane = np.asarray(plane, dtype=float)
    n = np.linalg.norm(plane[:3])
    if n < 1e-12:
        return plane
    return plane / n


def point_plane_distances(points, plane):
    """점 집합과 평면 ax+by+cz+d=0 사이의 수직거리 (정규화된 평면 기준)."""
    plane = normalize_plane(plane)
    a, b, c, d = plane
    return np.abs(points @ np.array([a, b, c]) + d)


def fit_plane_from_3points(p1, p2, p3):
    """3점으로 평면 방정식 결정 (외적으로 법선벡터 계산)."""
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
    다중 선형 회귀 (수업 내용의 3D 확장).

    모델:  z = a0 + a1*x + a2*y
    목적:  Sr = Σ(zi - a0 - a1*xi - a2*yi)² 최소화
    풀이:  정규방정식  [A^T A]{c} = {A^T z}  →  np.linalg.solve

    수업 슬라이드 다중 선형 회귀의 행렬식과 동일한 구조:
      [ n    Σxi   Σyi  ] [a0]   [Σzi   ]
      [ Σxi  Σxi²  Σxiyi] [a1] = [Σxizi ]
      [ Σyi  Σxiyi Σyi² ] [a2]   [Σyizi ]

    반환: normalize된 ax+by+cz+d=0 계수 (a1*x + a2*y - z + a0 = 0)
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    A = np.column_stack([np.ones(len(points)), x, y])  # 설계행렬 (n×3)
    ATA = A.T @ A   # 3×3 정규방정식 좌변
    ATb = A.T @ z   # 3×1 정규방정식 우변
    try:
        coeffs = np.linalg.solve(ATA, ATb)  # [a0, a1, a2]
    except np.linalg.LinAlgError:
        return None
    # z = a0 + a1*x + a2*y  →  a1*x + a2*y + (-1)*z + a0 = 0
    return normalize_plane([coeffs[1], coeffs[2], -1.0, coeffs[0]])


def custom_ransac(points, threshold=0.05, iterations=500):
    """
    RANSAC (Random Sample Consensus) 평면 피팅.
    매 반복: 3점 랜덤 샘플 → 평면 가설 → 인라이어 카운팅 → 최적 보존.
    """
    best_plane = None
    best_inliers = np.array([], dtype=int)

    n_points = len(points)
    if n_points < 3:
        return None, best_inliers

    for _ in range(iterations):
        ids = np.random.choice(n_points, 3, replace=False)
        plane = fit_plane_from_3points(points[ids[0]], points[ids[1]], points[ids[2]])
        if plane is None:
            continue
        distances = point_plane_distances(points, plane)
        inliers = np.where(distances < threshold)[0]
        if len(inliers) > len(best_inliers):
            best_plane = plane
            best_inliers = inliers

    return best_plane, best_inliers


def angle_error_deg(plane, gt_normal):
    """탐지된 평면의 법선벡터와 정답 법선벡터 사이의 각도 오차 (도 단위)."""
    n = normalize_plane(plane)[:3]
    gt = np.asarray(gt_normal, dtype=float)
    gt = gt / (np.linalg.norm(gt) + 1e-12)
    # n과 -n은 동일한 평면 → abs(dot)으로 180° 뒤집힘 처리
    dot = abs(float(np.dot(n, gt)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def evaluate(points, plane, inliers, gt_normal):
    """평면 피팅 결과 평가: 인라이어 비율, RMSE, 각도 오차."""
    if plane is None or len(inliers) == 0:
        return {"inlier_ratio": 0.0, "rmse": float("nan"), "angle_error": float("nan")}
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

        self.declare_parameter('input_topic',        '/mid360_PointCloud2')
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('ransac_iterations',  500)
        self.declare_parameter('voxel_size',         0.05)
        self.declare_parameter('max_points',         15000)
        self.declare_parameter('process_every_n',    5)
        self.declare_parameter('frame_id',           '')
        self.declare_parameter('z_min',              -2.0)
        self.declare_parameter('z_max',              5.0)
        self.declare_parameter('range_max',          25.0)
        self.declare_parameter('stats_window',       30)

        self.input_topic     = self.get_parameter('input_topic').value
        self.threshold       = float(self.get_parameter('distance_threshold').value)
        self.iterations      = int(self.get_parameter('ransac_iterations').value)
        self.voxel_size      = float(self.get_parameter('voxel_size').value)
        self.max_points      = int(self.get_parameter('max_points').value)
        self.process_every_n = int(self.get_parameter('process_every_n').value)
        self.fixed_frame_id  = self.get_parameter('frame_id').value
        self.z_min           = float(self.get_parameter('z_min').value)
        self.z_max           = float(self.get_parameter('z_max').value)
        self.range_max       = float(self.get_parameter('range_max').value)
        self.stats_window    = int(self.get_parameter('stats_window').value)
        self.declare_parameter('save_data_path', '/tmp/plane_fit_data.npz')
        self.save_data_path  = self.get_parameter('save_data_path').value

        # 알고리즘별 지표 버퍼: 최근 stats_window 프레임 평균/표준편차 계산용
        self.stats_buf: dict[str, dict[str, deque]] = {}
        self.processed  = 0
        self.data_saved = False

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/plane_fit/markers', 10)

        self.count = 0
        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info('Ground truth: horizontal floor (normal=[0,0,1])')
        self.get_logger().info('RViz MarkerArray topic: /plane_fit/markers')

    def gt_normal(self):
        """정답 법선벡터: 수평 바닥 기준 [0, 0, 1]."""
        return np.array([0.0, 0.0, 1.0])

    def cloud_to_numpy(self, msg):
        """PointCloud2 → numpy 배열 변환, 범위 필터링 및 다운샘플링."""
        raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(raw) == 0:
            return np.empty((0, 3), dtype=np.float64)

        # read_points는 structured array 반환: dtype=[('x','f4'),('y','f4'),('z','f4')]
        points = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)

        # z 범위 및 거리 필터 (벡터화)
        r = np.linalg.norm(points, axis=1)
        mask = (points[:, 2] >= self.z_min) & \
               (points[:, 2] <= self.z_max) & \
               (r <= self.range_max)
        points = points[mask]
        if len(points) == 0:
            return points

        # 복셀 다운샘플
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

        # --- Algorithm 1: Custom RANSAC (이상치에 강건한 랜덤 샘플링) ---
        t0 = time.perf_counter()
        plane_ransac, inliers_ransac = custom_ransac(points, self.threshold, self.iterations)
        time_ransac = time.perf_counter() - t0
        results.append(('Custom RANSAC', plane_ransac, inliers_ransac, time_ransac,
                         evaluate(points, plane_ransac, inliers_ransac, gt)))

        # --- Algorithm 2: RANSAC + Least Squares (수업 내용의 3D 확장) ---
        # RANSAC 인라이어 전체에 다중 선형 회귀 적용 → 정규방정식으로 계수 결정
        t0 = time.perf_counter()
        if plane_ransac is not None and len(inliers_ransac) >= 3:
            plane_ls = least_squares_plane(points[inliers_ransac])
            if plane_ls is not None:
                # LS 평면 기준으로 인라이어 재계산
                inliers_ls = np.where(
                    point_plane_distances(points, plane_ls) < self.threshold)[0]
            else:
                plane_ls, inliers_ls = plane_ransac, inliers_ransac
        else:
            plane_ls, inliers_ls = None, np.array([], dtype=int)
        time_ls = time_ransac + (time.perf_counter() - t0)
        results.append(('RANSAC + LS', plane_ls, inliers_ls, time_ls,
                         evaluate(points, plane_ls, inliers_ls, gt)))

        # 지표 버퍼 업데이트
        self.processed += 1
        for name, _, _, elapsed, metrics in results:
            if name not in self.stats_buf:
                self.stats_buf[name] = {
                    k: deque(maxlen=self.stats_window)
                    for k in ('inlier_ratio', 'rmse', 'angle_error')
                }
            if not math.isnan(metrics['rmse']):
                self.stats_buf[name]['inlier_ratio'].append(metrics['inlier_ratio'])
                self.stats_buf[name]['rmse'].append(metrics['rmse'])
                self.stats_buf[name]['angle_error'].append(metrics['angle_error'])

        # 프레임별 로그
        log_lines = []
        for name, _, _, elapsed, metrics in results:
            log_lines.append(
                f'{name}: ratio={metrics["inlier_ratio"]:.3f}, '
                f'RMSE={metrics["rmse"]:.4f} m, '
                f'angle={metrics["angle_error"]:.3f} deg, '
                f'time={elapsed*1000:.1f} ms'
            )
        self.get_logger().info(' | '.join(log_lines))

        # stats_window 프레임마다 평균/표준편차 출력
        if (self.processed % self.stats_window == 0 and
                all(len(b['rmse']) == self.stats_window
                    for b in self.stats_buf.values())):
            self.get_logger().info(
                f'=== 최근 {self.stats_window}프레임 통계 (총 처리: {self.processed}프레임) ===')
            for name, buf in self.stats_buf.items():
                ir  = np.array(buf['inlier_ratio'])
                rm  = np.array(buf['rmse'])
                ang = np.array(buf['angle_error'])
                self.get_logger().info(
                    f'  [{name}]  '
                    f'IR={ir.mean():.3f}±{ir.std():.3f}  '
                    f'RMSE={rm.mean():.4f}±{rm.std():.4f} m  '
                    f'Angle={ang.mean():.3f}±{ang.std():.3f} deg'
                )

        # 5번째 프레임에서 실제 데이터 저장 (그림 생성용)
        if not self.data_saved and self.processed == 5:
            self._save_frame(points, results)

        frame_id = self.fixed_frame_id if self.fixed_frame_id else msg.header.frame_id
        self.publish_markers(frame_id, msg.header.stamp, points, results)

    def _save_frame(self, points, results):
        _, plane_r, inliers_r, _, _ = results[0]
        _, plane_l, inliers_l, _, _ = results[1]
        np.savez(self.save_data_path,
                 points=points,
                 inliers_ransac=inliers_r,
                 inliers_ls=inliers_l,
                 plane_ransac=plane_r if plane_r is not None else np.zeros(4),
                 plane_ls=plane_l if plane_l is not None else np.zeros(4))
        self.data_saved = True
        self.get_logger().info(f'Frame data saved → {self.save_data_path}')

    def publish_markers(self, frame_id, stamp, points, results):
        ma = MarkerArray()

        delete = Marker()
        delete.header.frame_id = frame_id
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        marker_id = 0

        # 시각화: 최종 알고리즘(RANSAC + LS) 인라이어/아웃라이어
        _, plane, inliers, _, _ = results[-1]

        if plane is not None and len(inliers) > 0:
            # 인라이어 포인트 (초록 = 바닥으로 탐지됨)
            inlier_marker = Marker()
            inlier_marker.header.frame_id = frame_id
            inlier_marker.header.stamp = stamp
            inlier_marker.ns = 'plane_fit_inliers'
            inlier_marker.id = marker_id; marker_id += 1
            inlier_marker.type = Marker.POINTS
            inlier_marker.action = Marker.ADD
            inlier_marker.scale.x = 0.04
            inlier_marker.scale.y = 0.04
            inlier_marker.color.r = 0.0
            inlier_marker.color.g = 1.0
            inlier_marker.color.b = 0.0
            inlier_marker.color.a = 1.0
            sample = inliers if len(inliers) <= 3000 else np.random.choice(inliers, 3000, replace=False)
            for p in points[sample]:
                inlier_marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ma.markers.append(inlier_marker)

            # 아웃라이어 포인트 (빨강 = 벽/장애물 등)
            outlier_idx = np.setdiff1d(np.arange(len(points)), inliers)
            out_marker = Marker()
            out_marker.header.frame_id = frame_id
            out_marker.header.stamp = stamp
            out_marker.ns = 'plane_fit_outliers'
            out_marker.id = marker_id; marker_id += 1
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

        # 텍스트 오버레이: 알고리즘별 지표 비교
        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = 'plane_fit_text'
        text.id = marker_id; marker_id += 1
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
        for alg_name, _, _, elapsed_i, metrics_i in results:
            lines.append(
                f'{alg_name}: IR={metrics_i["inlier_ratio"]:.3f}, '
                f'RMSE={metrics_i["rmse"]:.4f}m, '
                f'Angle={metrics_i["angle_error"]:.3f}deg, '
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

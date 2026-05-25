#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointField
import std_msgs.msg



class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')

        self.sub = self.create_subscription(
            PointCloud2,
            '/mid360_PointCloud2',
            self.callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/cluster_markers',
            10
        )

        # 🔥 후처리된 포인트들을 발행할 새로운 퍼블리셔 추가
        self.pcd_pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10
        )

        # ROI 설정
        self.min_range = 0.3
        self.max_range = 15.0

        # 🔥 Self-Body 제거 범위 (충분히 넓게 설정)
        self.self_x = (-0.3, 0.3)
        self.self_y = (-0.3, 0.3)
        self.self_z = (-0.3, 0.5)

        # RANSAC (ransac.py 성공 수치 적용)
        self.ransac_iter = 100
        self.distance_threshold = 0.1  # 지면 제거 임계값 상향

        # 바닥 방향 조건 완화
        self.vertical_threshold = 0.7

        # DBSCAN (cluster.py 성공 수치 적용)
        self.dbscan_eps = 0.5          # 군집 거리 상향
        self.dbscan_min_samples = 10

        self.min_cluster_points = 20

    def callback(self, msg):
        raw_points = []

        # 1. 포인트 데이터 읽기
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            r = math.sqrt(x*x + y*y)
            if self.min_range < r < self.max_range:
                raw_points.append([x, y, z])

        if len(raw_points) < 50:
            return

        # 배열 변환 및 초기 개수 기록
        points = np.array(raw_points, dtype=np.float32)
        initial_count = len(points)  # 🔥 여기서 변수를 먼저 정의합니다.

        # 2. Self-Body 제거 및 디버깅
        filtered_points = self.remove_self_points(points)
        after_self_count = len(filtered_points)
        self.get_logger().info(f"Self-Filter: {initial_count} -> {after_self_count} (삭제됨: {initial_count - after_self_count})")

        if after_self_count == 0:
            return

        # 3. RANSAC 지면 제거 및 디버깅
        non_ground = self.ransac_ground_removal(filtered_points)
        after_ground_count = len(non_ground)
        self.get_logger().info(f"Ground-Filter: {after_self_count} -> {after_ground_count} (삭제됨: {after_self_count - after_ground_count})")

        if after_ground_count == 0:
            return

        # 4. DBSCAN 군집화
        clustering = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples
        ).fit(non_ground)

        labels = clustering.labels_
        unique_labels = set(labels)

        # 🔥 [추가] 후처리된 포인트(non_ground)를 RViz2에 뿌려주기
        if len(non_ground) > 0:
            filtered_msg = self.create_pointcloud2(non_ground, msg.header)
            self.pcd_pub.publish(filtered_msg)

        # 마커 초기화
        self.clear_markers(msg.header.stamp, msg.header.frame_id)
        marker_id = 0

        for label in unique_labels:
            if label == -1: # 노이즈 무시
                continue

            cluster = non_ground[labels == label]
            if len(cluster) < self.min_cluster_points:
                continue

            # 바운딩 박스(Bounding Box) 크기 및 중심 계산
            x_min, y_min, z_min = np.min(cluster, axis=0)
            x_max, y_max, z_max = np.max(cluster, axis=0)

            cx = float((x_min + x_max) / 2.0)
            cy = float((y_min + y_max) / 2.0)
            cz = float((z_min + z_max) / 2.0)

            sx = float(max(x_max - x_min, 0.05))
            sy = float(max(y_max - y_min, 0.05))
            sz = float(max(z_max - z_min, 0.05))

            self.publish_box_marker(cx, cy, cz, sx, sy, sz, marker_id, msg.header)
            marker_id += 1

    def remove_self_points(self, points):
        mask = ~(
            (self.self_x[0] < points[:, 0]) & (points[:, 0] < self.self_x[1]) &
            (self.self_y[0] < points[:, 1]) & (points[:, 1] < self.self_y[1]) &
            (self.self_z[0] < points[:, 2]) & (points[:, 2] < self.self_z[1])
        )
        return points[mask]

    def ransac_ground_removal(self, points):
        best_plane = None
        max_inliers = 0

        for _ in range(self.ransac_iter):
            if len(points) < 3: break
            sample = points[np.random.choice(len(points), 3, replace=False)]
            p1, p2, p3 = sample
            v1, v2 = p2 - p1, p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm == 0: continue
            normal = normal / norm

            if abs(normal[2]) < self.vertical_threshold:
                continue

            d = -np.dot(normal, p1)
            distances = np.abs(points @ normal + d)
            inliers = distances < self.distance_threshold
            count = np.sum(inliers)

            if count > max_inliers:
                max_inliers = count
                best_plane = (normal, d)

        if best_plane is None:
            return points

        normal, d = best_plane
        distances = np.abs(points @ normal + d)
        return points[distances > self.distance_threshold]

    def publish_box_marker(self, cx, cy, cz, sx, sy, sz, m_id, header):
        marker = Marker()
        marker.header = header
        marker.ns = "clusters"
        marker.id = m_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = cx, cy, cz
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.6
        self.marker_pub.publish(marker)

    def clear_markers(self, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def create_pointcloud2(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points)    
    

def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
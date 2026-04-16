#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

from sklearn.cluster import DBSCAN

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.sub = self.create_subscription(
            PointCloud2, '/mid360_PointCloud2', self.callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.marker_pub = self.create_publisher(Marker, '/cluster_markers', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -------------------------
        # TF
        # -------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------------
        # Parameters
        # -------------------------
        self.target_frame = 'odom'

        # ROI / filtering
        self.ground_z_limit = 0.25
        self.roi_min_range = 0.3
        self.roi_max_range = 15.0

        # Remove robot body points
        self.self_x = (-0.6, 0.6)
        self.self_y = (-0.6, 0.6)
        self.self_z = (-0.3, 0.8)

        # DBSCAN
        self.dbscan_eps = 0.7
        self.dbscan_min_samples = 8
        self.min_cluster_points = 15

        # Trackable candidate filter
        self.trackable_max_size_x = 2.0
        self.trackable_max_size_y = 2.0
        self.trackable_max_size_z = 1.5
        self.trackable_min_centroid_z = 0.2

        # Tracking
        self.track_match_dist = 3.0
        self.move_threshold = 0.05
        self.max_motion_dist = 3.0
        self.min_motion_frames = 1
        self.max_missed_frames = 12

        # Locking
        self.lock_lost_frames = 20
        self.lock_keep_missed = 2
        self.lock_switch_cooldown = 20   # 프레임 수
        self.lock_age = 0

        # Rotation freeze
        self.rotation_freeze_threshold = 0.20

        # Control
        self.angle_deadband = 0.08
        self.angular_gain = 1.5
        self.max_angular_speed = 1.2

        # -------------------------
        # Runtime state
        # -------------------------
        self.tracks = []
        self.next_track_id = 0

        self.locked_target_id = None
        self.locked_target_missed = 0

        self.current_angular_z = 0.0
        self.last_cmd_angular_z = 0.0

        self.display_clusters = []

        self.get_logger().info('Lock-stable cluster tracking node started.')

    # =========================================================
    # Callbacks
    # =========================================================
    def odom_callback(self, msg: Odometry):
        self.current_angular_z = msg.twist.twist.angular.z

    def callback(self, msg: PointCloud2):
        raw_points = []

        # 1) ROI filtering
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            r = math.sqrt(x * x + y * y)

            if self.roi_min_range < r < self.roi_max_range and z > self.ground_z_limit:
                raw_points.append([x, y, z])

        if len(raw_points) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, msg.header.frame_id)
            self.update_tracks_no_detection()
            self.stop_robot()
            return

        points = np.array(raw_points, dtype=np.float32)

        # 2) Remove self points
        non_self = self.remove_self_points(points)
        if len(non_self) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, msg.header.frame_id)
            self.update_tracks_no_detection()
            self.stop_robot()
            return

        # Optional light downsampling for stability/speed
        if len(non_self) > 3000:
            non_self = non_self[::2]

        # Publish filtered cloud
        filtered_msg = self.create_pointcloud2(non_self, msg.header)
        self.pcd_pub.publish(filtered_msg)

        # 3) Clustering
        clustering = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples
        ).fit(non_self)

        labels = clustering.labels_
        unique_labels = set(labels)

        detections = []
        self.display_clusters = []

        # 4) Build display clusters and tracking detections
        for label in unique_labels:
            if label == -1:
                continue

            cluster = non_self[labels == label]
            if len(cluster) < self.min_cluster_points:
                continue

            local_centroid = np.mean(cluster, axis=0)
            cx, cy, cz = local_centroid

            x_min, y_min, z_min = np.min(cluster, axis=0)
            x_max, y_max, z_max = np.max(cluster, axis=0)
            sx = max(x_max - x_min, 0.1)
            sy = max(y_max - y_min, 0.1)
            sz = max(z_max - z_min, 0.1)

            # Display all valid clusters
            self.display_clusters.append({
                'id': int(label),
                'local_centroid': local_centroid,
                'bbox': (sx, sy, sz)
            })

            # Trackable candidate
            is_trackable = (
                sx < self.trackable_max_size_x and
                sy < self.trackable_max_size_y and
                sz < self.trackable_max_size_z and
                cz > self.trackable_min_centroid_z
            )

            if not is_trackable:
                continue

            odom_centroid = self.transform_point(
                local_centroid,
                source_frame=msg.header.frame_id,
                target_frame=self.target_frame,
                stamp=msg.header.stamp
            )

            if odom_centroid is None:
                continue

            detections.append({
                'local_centroid': local_centroid,
                'odom_centroid': odom_centroid,
                'bbox': (sx, sy, sz),
            })

        # 5) Update tracks
        if detections:
            self.update_tracks(detections)
        else:
            self.update_tracks_no_detection()

        # 6) Publish markers
        self.publish_all_markers(msg.header)

        # 7) Select/track target
        self.select_and_track_target()

    # =========================================================
    # Tracking
    # =========================================================
    def update_tracks(self, detections):
        unmatched_det_indices = set(range(len(detections)))

        for tr in self.tracks:
            tr['missed'] += 1

        candidate_pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, det in enumerate(detections):
                dist = np.linalg.norm(det['odom_centroid'] - tr['odom_centroid'])
                if dist < self.track_match_dist:
                    candidate_pairs.append((dist, ti, di))

        candidate_pairs.sort(key=lambda x: x[0])

        used_tracks = set()
        used_dets = set()

        for dist, ti, di in candidate_pairs:
            if ti in used_tracks or di in used_dets:
                continue

            track = self.tracks[ti]
            det = detections[di]

            prev_odom = track['odom_centroid'].copy()
            new_odom = det['odom_centroid']
            motion_dist = np.linalg.norm(new_odom - prev_odom)

            track['odom_centroid'] = new_odom
            track['local_centroid'] = det['local_centroid']
            track['bbox'] = det['bbox']
            track['age'] += 1
            track['missed'] = 0
            track['seen_count'] += 1
            track['last_motion_dist'] = motion_dist

            # Motion accumulation
            if self.move_threshold < motion_dist < self.max_motion_dist:
                track['moving_count'] += 1
            else:
                # 이번엔 너무 쉽게 깎지 않음
                track['moving_count'] = track['moving_count']

            track['is_moving_confirmed'] = track['moving_count'] >= self.min_motion_frames

            unmatched_det_indices.discard(di)
            used_tracks.add(ti)
            used_dets.add(di)

        # Create new tracks
        for di in unmatched_det_indices:
            det = detections[di]
            self.tracks.append({
                'id': self.next_track_id,
                'odom_centroid': det['odom_centroid'],
                'local_centroid': det['local_centroid'],
                'bbox': det['bbox'],
                'age': 1,
                'missed': 0,
                'seen_count': 1,
                'moving_count': 1,
                'last_motion_dist': self.move_threshold + 0.01,
                'is_moving_confirmed': self.min_motion_frames <= 1
            })
            self.next_track_id += 1

        self.tracks = [tr for tr in self.tracks if tr['missed'] <= self.max_missed_frames]

        # Validate locked target
        if self.locked_target_id is not None:
            locked = self.get_track_by_id(self.locked_target_id)

            if locked is None:
                self.locked_target_missed += 1
            elif locked['missed'] > self.lock_keep_missed:
                self.locked_target_missed += 1
            else:
                self.locked_target_missed = 0

            if self.locked_target_missed > self.lock_lost_frames:
                self.locked_target_id = None
                self.locked_target_missed = 0
                self.lock_age = 0

    def update_tracks_no_detection(self):
        for tr in self.tracks:
            tr['missed'] += 1

        self.tracks = [tr for tr in self.tracks if tr['missed'] <= self.max_missed_frames]

        if self.locked_target_id is not None:
            self.locked_target_missed += 1
            if self.locked_target_missed > self.lock_lost_frames:
                self.locked_target_id = None
                self.locked_target_missed = 0
                self.lock_age = 0

    def select_and_track_target(self):
        rotating = abs(self.current_angular_z) > self.rotation_freeze_threshold

        locked = None
        if self.locked_target_id is not None:
            locked = self.get_track_by_id(self.locked_target_id)

        # 1) 기존 lock 우선 유지
        if locked is not None and locked['missed'] <= self.lock_keep_missed:
            self.lock_age += 1
            cx, cy, cz = locked['local_centroid']
            self.track_target(cx, cy, cz)
            return

        # 2) 회전 중이면 새 타깃 선택 안 함
        if rotating:
            self.stop_robot()
            return

        candidates = [
            tr for tr in self.tracks
            if tr['missed'] == 0 and (
                tr['is_moving_confirmed'] or tr['last_motion_dist'] > self.move_threshold
            )
        ]

        if not candidates:
            self.stop_robot()
            return

        def candidate_score(tr):
            cx, cy, _ = tr['local_centroid']
            angle = abs(math.atan2(cy, cx))
            dist = math.sqrt(cx * cx + cy * cy)

            # 정면, 가까운 대상 우선
            score = angle + 0.15 * dist

            # 잠깐이라도 이전 lock과 가까우면 약간 우대
            if self.locked_target_id is not None:
                old_locked = self.get_track_by_id(self.locked_target_id)
                if old_locked is not None:
                    lock_dist = np.linalg.norm(tr['odom_centroid'] - old_locked['odom_centroid'])
                    score += 0.05 * lock_dist

            return score

        # 3) lock cooldown 중이면 기존 lock 근처 후보 우선
        if self.locked_target_id is not None and self.lock_age < self.lock_switch_cooldown:
            old_locked = self.get_track_by_id(self.locked_target_id)
            if old_locked is not None:
                nearby = []
                for tr in candidates:
                    d = np.linalg.norm(tr['odom_centroid'] - old_locked['odom_centroid'])
                    if d < self.track_match_dist:
                        nearby.append((d, tr))
                if nearby:
                    nearby.sort(key=lambda x: x[0])
                    best = nearby[0][1]
                else:
                    best = min(candidates, key=candidate_score)
            else:
                best = min(candidates, key=candidate_score)
        else:
            best = min(candidates, key=candidate_score)

        self.locked_target_id = best['id']
        self.locked_target_missed = 0
        self.lock_age = 0

        cx, cy, cz = best['local_centroid']
        self.track_target(cx, cy, cz)

    def get_track_by_id(self, track_id):
        for tr in self.tracks:
            if tr['id'] == track_id:
                return tr
        return None

    # =========================================================
    # Control
    # =========================================================
    def track_target(self, cx, cy, cz):
        target_angle_rad = math.atan2(cy, cx)

        twist = Twist()
        if abs(target_angle_rad) > self.angle_deadband:
            cmd = self.angular_gain * target_angle_rad
            cmd = max(min(cmd, self.max_angular_speed), -self.max_angular_speed)
            twist.angular.z = cmd
        else:
            twist.angular.z = 0.0

        self.last_cmd_angular_z = twist.angular.z
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.last_cmd_angular_z = 0.0
        self.cmd_pub.publish(Twist())

    # =========================================================
    # TF / Point utilities
    # =========================================================
    def remove_self_points(self, points: np.ndarray) -> np.ndarray:
        mask = ~(
            (self.self_x[0] < points[:, 0]) & (points[:, 0] < self.self_x[1]) &
            (self.self_y[0] < points[:, 1]) & (points[:, 1] < self.self_y[1]) &
            (self.self_z[0] < points[:, 2]) & (points[:, 2] < self.self_z[1])
        )
        return points[mask]

    def transform_point(self, point_xyz, source_frame, target_frame, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        rot = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        p = np.array(point_xyz, dtype=np.float32)
        transformed = rot @ p + np.array([tx, ty, tz], dtype=np.float32)
        return transformed

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)]
        ], dtype=np.float32)

    # =========================================================
    # Marker publishing
    # =========================================================
    def publish_all_markers(self, header):
        self.clear_markers(header.stamp, header.frame_id)

        locked_track = None
        if self.locked_target_id is not None:
            locked_track = self.get_track_by_id(self.locked_target_id)

        # lock된 target과 가장 가까운 display cluster를 따로 찾음
        locked_display_id = None
        if locked_track is not None and locked_track['missed'] <= self.lock_keep_missed:
            best_dist = float('inf')
            for dc in self.display_clusters:
                d = np.linalg.norm(dc['local_centroid'] - locked_track['local_centroid'])
                if d < best_dist:
                    best_dist = d
                    locked_display_id = dc['id']

        for dc in self.display_clusters:
            cx, cy, cz = dc['local_centroid']
            sx, sy, sz = dc['bbox']

            color = (0.0, 1.0, 0.0)  # green default

            matched_track = self.match_display_cluster_to_track(dc)

            if locked_display_id is not None and dc['id'] == locked_display_id:
                color = (0.0, 0.3, 1.0)   # blue = locked target
            elif matched_track is not None:
                if matched_track['is_moving_confirmed']:
                    color = (1.0, 0.0, 0.0)   # red = moving confirmed
                elif matched_track['last_motion_dist'] > self.move_threshold:
                    color = (1.0, 0.5, 0.0)   # orange = moving candidate

            self.publish_box_marker(cx, cy, cz, sx, sy, sz, dc['id'], header, color)

    def match_display_cluster_to_track(self, display_cluster):
        dc = display_cluster['local_centroid']
        best_track = None
        best_dist = float('inf')

        for tr in self.tracks:
            if tr['missed'] > 0:
                continue
            dist = np.linalg.norm(dc - tr['local_centroid'])
            if dist < best_dist:
                best_dist = dist
                best_track = tr

        if best_dist < 2.5:
            return best_track
        return None

    def publish_box_marker(self, cx, cy, cz, sx, sy, sz, m_id, header, color):
        marker = Marker()
        marker.header = header
        marker.ns = "bird_bbox"
        marker.id = int(m_id)
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        dx, dy, dz = sx / 2.0, sy / 2.0, sz / 2.0
        v = [
            [cx - dx, cy - dy, cz - dz], [cx + dx, cy - dy, cz - dz],
            [cx + dx, cy + dy, cz - dz], [cx - dx, cy + dy, cz - dz],
            [cx - dx, cy - dy, cz + dz], [cx + dx, cy - dy, cz + dz],
            [cx + dx, cy + dy, cz + dz], [cx - dx, cy + dy, cz + dz]
        ]
        lines = [
            v[0], v[1], v[1], v[2], v[2], v[3], v[3], v[0],
            v[4], v[5], v[5], v[6], v[6], v[7], v[7], v[4],
            v[0], v[4], v[1], v[5], v[2], v[6], v[3], v[7]
        ]

        for p in lines:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            marker.points.append(pt)

        self.marker_pub.publish(marker)

    def clear_markers(self, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    # =========================================================
    # PointCloud2 utility
    # =========================================================
    def create_pointcloud2(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points.tolist())


def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
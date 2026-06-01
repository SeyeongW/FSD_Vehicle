#!/usr/bin/env python3
"""
cluster_node.py — LiDAR DBSCAN 클러스터 기반 조류 추적
                  + 칼만 필터 (위치 예측으로 추적 지연 보상)
                  + 전방 추적 기동 (bbox x,y 기반 추적 + linear.x 제어)
                  + 카메라 틸트 제어 (상하: 틸트, 좌우: 로봇 회전)
                  + LiDAR 유실 시 카메라 YOLO 방위각 융합

[추적 모드]
  LIDAR_MODE: LiDAR KF 예측 위치 기반 (3D). 기본 모드.
  VISUAL_MODE: LiDAR 트랙을 잃었을 때 YOLO 방위각으로 방향만 유지.
               LiDAR가 재검출하면 즉시 LIDAR_MODE로 복귀.

[칼만 필터]
  상태: [x, y, z, vx, vy, vz] (odom 프레임)
  예측: 등속 운동 모델로 다음 위치 추정 (센서 지연 보상)
  lookahead 0.15s: 처리 지연 시간만큼 앞의 위치를 겨냥

[카메라 틸트]
  pt_link1_to_pt_link2 관절(revolute, axis [0,-1,0])을 /set_joint_trajectory 로 제어.
  양의 각도 = 카메라 아래, 음의 각도 = 카메라 위.
  새의 앙각(elevation)을 계산해 카메라가 새를 수직 중앙에 보도록 틸트.
  tilt_sign = -1.0: elevation > 0 (새가 위) → 음수 명령 → 카메라 위.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

from sklearn.cluster import DBSCAN

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


# ============================================================
# 칼만 필터 유틸리티 (numpy 전용)
# ============================================================

def kf_init(odom_pos: np.ndarray) -> dict:
    """
    새 트랙용 칼만 필터 상태 초기화.
    x = [px, py, pz, vx, vy, vz]^T  (odom 프레임)
    위치는 첫 측정값으로, 속도는 0으로 시작.
    """
    x = np.zeros(6, dtype=np.float64)
    x[:3] = odom_pos
    P = np.diag([1.0, 1.0, 1.0, 5.0, 5.0, 5.0])
    return {'x': x, 'P': P}


def kf_predict(kf: dict, dt: float) -> dict:
    """
    dt초 후 상태 예측 (등속 운동 모델).
    Q: 새의 갑작스러운 방향 전환을 허용하도록 속도 노이즈를 크게 설정.
    """
    dt = min(dt, 0.5)
    F = np.eye(6, dtype=np.float64)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt

    q_p = 0.5 * dt * dt   # 속도 불확실성이 위치에 미치는 영향
    q_v = 2.0 * dt         # 새의 기동에 의한 속도 변화 허용
    Q = np.diag([q_p, q_p, q_p, q_v, q_v, q_v])

    kf['x'] = F @ kf['x']
    kf['P'] = F @ kf['P'] @ F.T + Q
    return kf


def kf_update(kf: dict, measurement: np.ndarray) -> dict:
    """
    위치 측정값으로 KF 갱신. H는 위치만 추출(첫 3요소).
    R: LiDAR 클러스터 centroid 의 위치 불확실성.
    """
    H = np.zeros((3, 6), dtype=np.float64)
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    H[2, 2] = 1.0

    R = np.diag([0.25, 0.25, 0.20])

    z = np.array(measurement, dtype=np.float64)
    y = z - H @ kf['x']
    S = H @ kf['P'] @ H.T + R
    K = kf['P'] @ H.T @ np.linalg.inv(S)

    kf['x'] = kf['x'] + K @ y
    kf['P'] = (np.eye(6, dtype=np.float64) - K @ H) @ kf['P']
    return kf


# ============================================================
# 추적 모드
# ============================================================
LIDAR_MODE = 'lidar'
VISUAL_MODE = 'visual'


class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')

        # -------------------------
        # ROS 인터페이스
        # -------------------------
        self.sub = self.create_subscription(
            PointCloud2, '/mid360_PointCloud2', self.callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        # YOLO 노드가 퍼블리시하는 카메라 방위각 (LiDAR 유실 시 폴백)
        self.visual_bearing_sub = self.create_subscription(
            Vector3Stamped, '/bird_visual_bearing', self.visual_bearing_callback, 10
        )

        self.marker_pub = self.create_publisher(Marker, '/cluster_markers', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tilt_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        # YOLO 노드에 LiDAR 3D 위치 전달
        self.target_pub = self.create_publisher(PointStamped, '/bird_target', 10)

        # -------------------------
        # TF
        # -------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------------
        # 파라미터
        # -------------------------
        self.target_frame = 'odom'

        self.ground_z_limit = 1.5
        self.roi_min_range = 0.3
        self.roi_max_range = 15.0

        self.self_x = (-0.6, 0.6)
        self.self_y = (-0.6, 0.6)
        self.self_z = (-0.3, 0.8)

        self.dbscan_eps = 0.5
        self.dbscan_min_samples = 4
        self.min_cluster_points = 5

        self.trackable_max_size_x = 2.0
        self.trackable_max_size_y = 2.0
        self.trackable_max_size_z = 1.5
        self.trackable_min_centroid_z = 2.0

        self.track_match_dist = 3.0
        self.move_threshold = 0.05
        self.max_motion_dist = 3.0
        self.min_motion_frames = 1
        self.max_missed_frames = 40

        self.lock_lost_frames = 50  # LiDAR 블라인드 구간에서 잠금 유지
        self.lock_keep_missed = 10  # KF 예측으로 추적 유지할 프레임 수
        self.lock_switch_cooldown = 20
        self.lock_age = 0

        self.rotation_freeze_threshold = 0.20

        self.angle_deadband = 0.08
        self.angular_gain = 1.5
        self.max_angular_speed = 1.2

        # ── 전방 추적 기동 ──
        # bbox x,y 기반으로 새를 향해 전진. 정렬 각도 임계값 이내일 때만 전진.
        self.chase_target_dist = 2.0      # 목표 유지 거리 (m)
        self.linear_gain = 0.5            # 거리 오차 → 전진 속도 비례 게인
        self.max_linear_speed = 0.5       # 최대 전진 속도 (m/s)
        self.chase_align_threshold = 1.0  # 전진 허용 각도 범위 (rad, 약 57°) — 넓게 허용

        # ── 칼만 필터 ──
        # lookahead: 이 시간(초)만큼 앞 위치를 겨냥해 추적 지연 보상
        self.kf_lookahead_sec = 0.15

        # ── 카메라 틸트 ──
        self.camera_mount_height = 0.168  # 카메라 마운트 높이 (m, base_footprint 기준)
        # 실제 동작 검증: 양의 각도=위, 음의 각도=아래 → elevation>0이면 양수 명령
        self.tilt_sign = +1.0
        self.tilt_min_rad = -0.52         # ≈ -30° (아래 한계)
        self.tilt_max_rad = 1.20          # ≈ +69° (위 한계)
        self.tilt_deadband = 0.04
        self.tilt_last_angle = 0.0
        self.tilt_last_send_time = 0.0
        self.tilt_cmd_interval = 0.10
        self.current_tilt_cmd = 0.0

        # ── 시각 융합 (Visual Fallback) ──
        self.visual_bearing_yaw = 0.0
        self.visual_bearing_tilt = 0.0
        self.visual_bearing_conf = 0.0
        self.visual_bearing_time = 0.0
        self.visual_bearing_timeout = 0.8
        self.tilt_visual_gain = 0.6

        # ── 런타임 상태 ──
        self.tracks = []
        self.next_track_id = 0

        self.locked_target_id = None
        self.locked_target_missed = 0

        self.tracking_mode = LIDAR_MODE

        self.current_angular_z = 0.0
        self.last_cmd_angular_z = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0

        self.display_clusters = []
        self.last_cb_time = None

        self.tilt_hold_enabled = True
        self.create_timer(0.1, self._tilt_hold_callback)

        # 0.5초 간격으로 새의 좌표(LiDAR 포인트 클라우드 기반 KF 추정치) 출력
        self.create_timer(0.5, self._bird_coord_log_callback)

        self.get_logger().info('Cluster tracking node (KF + chase + tilt + coord logger) started.')

    # =========================================================
    # 콜백
    # =========================================================
    def odom_callback(self, msg: Odometry):
        self.current_angular_z = msg.twist.twist.angular.z
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def visual_bearing_callback(self, msg: Vector3Stamped):
        """
        YOLO 노드로부터 카메라 방위각 오차를 수신.
        x: 수평 오차 (rad), 양수=조류가 좌측 → 좌회전 필요
        y: 수직 오차 (rad) — 카메라 틸트 비활성화로 미사용
        z: YOLO 신뢰도
        """
        self.visual_bearing_yaw = msg.vector.x
        self.visual_bearing_tilt = msg.vector.y
        self.visual_bearing_conf = msg.vector.z
        self.visual_bearing_time = self.get_clock().now().nanoseconds * 1e-9

    def callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        if self.last_cb_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_cb_time).nanoseconds * 1e-9
        self.last_cb_time = now

        raw_points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            r = math.sqrt(x * x + y * y)
            if self.roi_min_range < r < self.roi_max_range and z > self.ground_z_limit:
                raw_points.append([x, y, z])

        if len(raw_points) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, msg.header.frame_id)
            self.update_tracks_no_detection(dt)
            self.stop_robot()
            return

        points = np.array(raw_points, dtype=np.float32)
        non_self = self.remove_self_points(points)

        if len(non_self) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, msg.header.frame_id)
            self.update_tracks_no_detection(dt)
            self.stop_robot()
            return

        if len(non_self) > 3000:
            non_self = non_self[::2]

        filtered_msg = self.create_pointcloud2(non_self, msg.header)
        self.pcd_pub.publish(filtered_msg)

        clustering = DBSCAN(
            eps=self.dbscan_eps, min_samples=self.dbscan_min_samples
        ).fit(non_self)

        labels = clustering.labels_
        unique_labels = set(labels)

        detections = []
        self.display_clusters = []

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

            is_trackable = (
                sx < self.trackable_max_size_x and
                sy < self.trackable_max_size_y and
                sz < self.trackable_max_size_z and
                cz > self.trackable_min_centroid_z
            )
            if not is_trackable:
                continue

            self.display_clusters.append({
                'id': int(label), 'local_centroid': local_centroid, 'bbox': (sx, sy, sz)
            })

            odom_centroid = self.transform_point(
                local_centroid, source_frame=msg.header.frame_id,
                target_frame=self.target_frame, stamp=msg.header.stamp
            )
            if odom_centroid is None:
                continue

            detections.append({
                'local_centroid': local_centroid,
                'odom_centroid': odom_centroid,
                'bbox': (sx, sy, sz),
            })

        if detections:
            self.update_tracks(detections, dt)
        else:
            self.update_tracks_no_detection(dt)

        self.publish_all_markers(msg.header)
        self.select_and_track_target()

    # =========================================================
    # 트랙 갱신 (칼만 필터)
    # =========================================================
    def update_tracks(self, detections, dt: float):
        for tr in self.tracks:
            tr['missed'] += 1
            if dt > 0.0 and 'kf' in tr:
                tr['kf'] = kf_predict(tr['kf'], dt)

        candidate_pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, det in enumerate(detections):
                kf_pos = tr['kf']['x'][:3] if 'kf' in tr else tr['odom_centroid']
                dist = np.linalg.norm(det['odom_centroid'] - kf_pos)
                if dist < self.track_match_dist:
                    candidate_pairs.append((dist, ti, di))

        candidate_pairs.sort(key=lambda x: x[0])
        used_tracks = set()
        used_dets = set()
        unmatched_det_indices = set(range(len(detections)))

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

            if self.move_threshold < motion_dist < self.max_motion_dist:
                track['moving_count'] += 1
            track['is_moving_confirmed'] = track['moving_count'] >= self.min_motion_frames

            if 'kf' in track:
                track['kf'] = kf_update(track['kf'], new_odom)

            unmatched_det_indices.discard(di)
            used_tracks.add(ti)
            used_dets.add(di)

        for di in unmatched_det_indices:
            det = detections[di]
            self.tracks.append({
                'id': self.next_track_id,
                'odom_centroid': det['odom_centroid'],
                'local_centroid': det['local_centroid'],
                'bbox': det['bbox'],
                'age': 1, 'missed': 0, 'seen_count': 1, 'moving_count': 1,
                'last_motion_dist': self.move_threshold + 0.01,
                'is_moving_confirmed': self.min_motion_frames <= 1,
                'kf': kf_init(det['odom_centroid']),
            })
            self.next_track_id += 1

        self.tracks = [tr for tr in self.tracks if tr['missed'] <= self.max_missed_frames]

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

    def update_tracks_no_detection(self, dt: float):
        for tr in self.tracks:
            tr['missed'] += 1
            if dt > 0.0 and 'kf' in tr:
                tr['kf'] = kf_predict(tr['kf'], dt)
        self.tracks = [tr for tr in self.tracks if tr['missed'] <= self.max_missed_frames]
        if self.locked_target_id is not None:
            self.locked_target_missed += 1
            if self.locked_target_missed > self.lock_lost_frames:
                self.locked_target_id = None
                self.locked_target_missed = 0
                self.lock_age = 0

    # =========================================================
    # 타깃 선택 및 추적
    # =========================================================
    def select_and_track_target(self):
        rotating = abs(self.current_angular_z) > self.rotation_freeze_threshold
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        locked = None
        if self.locked_target_id is not None:
            locked = self.get_track_by_id(self.locked_target_id)

        # ── LIDAR_MODE: 정상 잠금 유지 ──────────────────────────────
        if locked is not None and locked['missed'] <= self.lock_keep_missed:
            self.lock_age += 1
            self.tracking_mode = LIDAR_MODE

            local_pred = self.get_kf_predicted_local(locked)
            cx, cy, cz = local_pred if local_pred is not None else locked['local_centroid']

            self.track_target(cx, cy, cz)
            self.publish_bird_target(locked)
            return

        # ── VISUAL_MODE 진입 조건: LiDAR 유실 + YOLO 방위각 유효 ──
        visual_fresh = (now_sec - self.visual_bearing_time) < self.visual_bearing_timeout
        if visual_fresh and self.locked_target_missed > 0:
            self.tracking_mode = VISUAL_MODE
            self.track_target_visual(self.visual_bearing_yaw, self.visual_bearing_tilt)
            return

        # ── LiDAR 새 후보 탐색 ──────────────────────────────────────
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
            self.tracking_mode = LIDAR_MODE
            self.stop_robot()
            self._tilt_reset()
            return

        def candidate_score(tr):
            cx, cy, _ = tr['local_centroid']
            angle = abs(math.atan2(cy, cx))
            dist = math.sqrt(cx * cx + cy * cy)
            score = angle + 0.15 * dist
            if self.locked_target_id is not None:
                old_locked = self.get_track_by_id(self.locked_target_id)
                if old_locked is not None:
                    lock_dist = np.linalg.norm(tr['odom_centroid'] - old_locked['odom_centroid'])
                    score += 0.05 * lock_dist
            return score

        if self.locked_target_id is not None and self.lock_age < self.lock_switch_cooldown:
            old_locked = self.get_track_by_id(self.locked_target_id)
            if old_locked is not None:
                nearby = [(np.linalg.norm(tr['odom_centroid'] - old_locked['odom_centroid']), tr)
                          for tr in candidates
                          if np.linalg.norm(tr['odom_centroid'] - old_locked['odom_centroid']) < self.track_match_dist]
                best = nearby[0][1] if nearby else min(candidates, key=candidate_score)
            else:
                best = min(candidates, key=candidate_score)
        else:
            best = min(candidates, key=candidate_score)

        self.locked_target_id = best['id']
        self.locked_target_missed = 0
        self.lock_age = 0
        self.tracking_mode = LIDAR_MODE

        local_pred = self.get_kf_predicted_local(best)
        cx, cy, cz = local_pred if local_pred is not None else best['local_centroid']
        self.track_target(cx, cy, cz)
        self.publish_bird_target(best)

    def get_kf_predicted_local(self, track: dict):
        """
        KF 상태에서 lookahead 앞의 위치를 예측하고 로봇 로컬 프레임으로 변환.
        odom 예측 위치를 로봇 yaw 만큼 역회전 → 로컬 프레임 (x=전방, y=좌측, z=위).
        """
        if 'kf' not in track:
            return None

        kf_state = track['kf']['x']
        predicted_odom = kf_state[:3] + kf_state[3:] * self.kf_lookahead_sec

        dx = predicted_odom[0] - self.robot_x
        dy = predicted_odom[1] - self.robot_y
        dz = predicted_odom[2] - self.robot_z

        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        local_z = dz

        return np.array([local_x, local_y, local_z])

    def get_track_by_id(self, track_id):
        for tr in self.tracks:
            if tr['id'] == track_id:
                return tr
        return None

    # =========================================================
    # 제어 명령
    # =========================================================
    def track_target(self, cx: float, cy: float, cz: float):
        """
        LiDAR bbox x,y 기반 추적 제어.
        cx, cy: 로봇 로컬 프레임 (KF 예측 또는 sensor 프레임 위치)
        로봇을 새 방향으로 회전 후, 정렬 시 전방 추적(chase) 기동.
        """
        target_angle_rad = math.atan2(cy, cx)
        dist_2d = math.sqrt(cx * cx + cy * cy)

        twist = Twist()

        # yaw 제어: 새를 향해 회전
        if abs(target_angle_rad) > self.angle_deadband:
            cmd = self.angular_gain * target_angle_rad
            cmd = max(min(cmd, self.max_angular_speed), -self.max_angular_speed)
            twist.angular.z = cmd
        else:
            twist.angular.z = 0.0

        # 전방 추적 기동: 정렬 각도 이내이고 목표 거리보다 멀면 전진
        if abs(target_angle_rad) < self.chase_align_threshold and dist_2d > self.chase_target_dist:
            fwd = self.linear_gain * (dist_2d - self.chase_target_dist)
            twist.linear.x = min(fwd, self.max_linear_speed)
        else:
            twist.linear.x = 0.0

        self.last_cmd_angular_z = twist.angular.z
        self.cmd_pub.publish(twist)

        # 카메라 틸트: 새의 고도각에 맞춰 상하 추적
        if dist_2d > 0.1:
            self.control_camera_tilt_lidar(cz, dist_2d)

    def track_target_visual(self, yaw_err: float, tilt_err: float):
        """
        VISUAL_MODE: YOLO 방위각 오차로 방향 유지 + 카메라 틸트 보정.
        LiDAR가 재검출하면 즉시 LIDAR_MODE로 복귀.
        """
        twist = Twist()
        if abs(yaw_err) > self.angle_deadband:
            cmd = self.angular_gain * yaw_err
            cmd = max(min(cmd, self.max_angular_speed), -self.max_angular_speed)
            twist.angular.z = cmd
        else:
            twist.angular.z = 0.0
        twist.linear.x = 0.0  # 거리 미확인 상태이므로 전진 금지
        self.cmd_pub.publish(twist)

        # 카메라 틸트: YOLO 수직 오차로 점진 보정
        delta = self.tilt_sign * tilt_err * self.tilt_visual_gain
        new_tilt = self.current_tilt_cmd + delta
        new_tilt = max(self.tilt_min_rad, min(self.tilt_max_rad, new_tilt))
        self._send_tilt_command(new_tilt)

    def control_camera_tilt_lidar(self, local_z: float, dist_2d: float):
        """
        LiDAR 위치 기반 절대 틸트 계산.
        local_z: 로봇 베이스 기준 새의 높이, dist_2d: 수평 거리.
        elevation = atan2(local_z - camera_height, dist_2d)
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.tilt_last_send_time < self.tilt_cmd_interval:
            return

        elevation = math.atan2(local_z - self.camera_mount_height, dist_2d)
        if elevation < 0.0:
            return  # 새가 카메라보다 아래 → 오감지 가능성, 틸트 유지

        tilt_cmd = self.tilt_sign * elevation
        tilt_cmd = max(self.tilt_min_rad, min(self.tilt_max_rad, tilt_cmd))

        if abs(tilt_cmd - self.tilt_last_angle) < self.tilt_deadband:
            return

        self._send_tilt_command(tilt_cmd)

    def _send_tilt_command(self, tilt_cmd: float):
        """JointTrajectory로 /set_joint_trajectory 퍼블리시.
        header.stamp 미설정: Gazebo 플러그인이 즉시 처리 (tilt_test.py 방식).
        """
        traj = JointTrajectory()
        # header.stamp 설정 안 함 — now()를 넣으면 sim clock과 미세 불일치로
        # 플러그인이 trajectory를 "과거 명령"으로 간주해 무시하는 경우 발생.
        traj.header.frame_id = 'base_link'
        traj.joint_names = ['pt_link1_to_pt_link2']

        pt = JointTrajectoryPoint()
        pt.positions = [float(tilt_cmd)]
        pt.velocities = []
        pt.accelerations = []
        pt.time_from_start = DurationMsg(sec=0, nanosec=500_000_000)  # 0.5s

        traj.points = [pt]
        self.tilt_pub.publish(traj)

        self.tilt_last_angle = tilt_cmd
        self.tilt_last_send_time = self.get_clock().now().nanoseconds * 1e-9
        self.current_tilt_cmd = tilt_cmd

    def _tilt_hold_callback(self):
        """0.1초마다 현재 틸트 명령을 재전송.
        time_from_start=0.5s: hold callback 주기(0.1s)의 5배 → 타이머 지터가
        생겨도 항상 유효한 trajectory가 플러그인 안에 존재함.
        """
        if not self.tilt_hold_enabled:
            return

        traj = JointTrajectory()
        # header.stamp 설정 안 함 (tilt_test.py와 동일)
        traj.header.frame_id = 'base_link'
        traj.joint_names = ['pt_link1_to_pt_link2']

        pt = JointTrajectoryPoint()
        pt.positions = [float(self.current_tilt_cmd)]
        pt.velocities = []
        pt.accelerations = []
        pt.time_from_start = DurationMsg(sec=0, nanosec=500_000_000)  # 0.5s

        traj.points = [pt]
        self.tilt_pub.publish(traj)

    def stop_robot(self):
        self.last_cmd_angular_z = 0.0
        self.cmd_pub.publish(Twist())

    def _tilt_reset(self):
        """추적 대상 없을 때 카메라를 정면(0°)으로 복귀."""
        if abs(self.current_tilt_cmd) > self.tilt_deadband:
            self._send_tilt_command(0.0)

    def publish_bird_target(self, track: dict):
        """KF 예측 위치(또는 마지막 관측 위치)를 /bird_target 으로 퍼블리시."""
        pos = track['kf']['x'][:3] if 'kf' in track else track['odom_centroid']
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.point.x = float(pos[0])
        msg.point.y = float(pos[1])
        msg.point.z = float(pos[2])
        self.target_pub.publish(msg)

    def _bird_coord_log_callback(self):
        """
        0.1초 간격으로 LiDAR 포인트 클라우드 기반 KF 추정 새 좌표를 로그 출력.
        잠금된 타깃이 없거나 트랙이 유실된 경우 출력 생략.
        출력 항목: odom 절대 좌표, 로봇-새 수평 거리, 로봇 정면 기준 수평 각도(deg)
        """
        if self.locked_target_id is None:
            return
        track = self.get_track_by_id(self.locked_target_id)
        if track is None:
            return
        if track['missed'] > self.lock_keep_missed:
            return

        if 'kf' in track:
            pos = track['kf']['x'][:3]
        else:
            pos = track['odom_centroid']

        # odom 프레임에서 로봇-새 벡터 → 로봇 로컬 프레임으로 변환
        dx = pos[0] - self.robot_x
        dy = pos[1] - self.robot_y
        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy

        dist_2d = math.sqrt(dx * dx + dy * dy)          # 수평 거리 (m)
        bearing_rad = math.atan2(local_y, local_x)       # 로봇 정면 기준 수평 각도
        bearing_deg = math.degrees(bearing_rad)           # deg 변환 (+좌/-우)

        self.get_logger().info(
            f'[BIRD] odom x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} '
            f'| dist={dist_2d:.2f}m bearing={bearing_deg:+.1f}deg '
            f'| id={self.locked_target_id}'
        )

    # =========================================================
    # TF / 포인트 유틸리티
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
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.2)
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
        return rot @ p + np.array([tx, ty, tz], dtype=np.float32)

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        xx = qx*qx; yy = qy*qy; zz = qz*qz
        xy = qx*qy; xz = qx*qz; yz = qy*qz
        wx = qw*qx; wy = qw*qy; wz = qw*qz
        return np.array([
            [1.0-2.0*(yy+zz), 2.0*(xy-wz),     2.0*(xz+wy)    ],
            [2.0*(xy+wz),     1.0-2.0*(xx+zz), 2.0*(yz-wx)    ],
            [2.0*(xz-wy),     2.0*(yz+wx),     1.0-2.0*(xx+yy)]
        ], dtype=np.float32)

    # =========================================================
    # 마커 퍼블리시
    # =========================================================
    def publish_all_markers(self, header):
        self.clear_markers(header.stamp, header.frame_id)

        locked_track = None
        if self.locked_target_id is not None:
            locked_track = self.get_track_by_id(self.locked_target_id)

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
            color = (0.0, 1.0, 0.0)
            matched_track = self.match_display_cluster_to_track(dc)

            if locked_display_id is not None and dc['id'] == locked_display_id:
                color = (0.0, 0.3, 1.0)
            elif matched_track is not None:
                if matched_track['is_moving_confirmed']:
                    color = (1.0, 0.0, 0.0)
                elif matched_track['last_motion_dist'] > self.move_threshold:
                    color = (1.0, 0.5, 0.0)

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
        return best_track if best_dist < 2.5 else None

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

        dx, dy, dz = sx/2.0, sy/2.0, sz/2.0
        v = [
            [cx-dx, cy-dy, cz-dz], [cx+dx, cy-dy, cz-dz],
            [cx+dx, cy+dy, cz-dz], [cx-dx, cy+dy, cz-dz],
            [cx-dx, cy-dy, cz+dz], [cx+dx, cy-dy, cz+dz],
            [cx+dx, cy+dy, cz+dz], [cx-dx, cy+dy, cz+dz]
        ]
        lines = [
            v[0],v[1], v[1],v[2], v[2],v[3], v[3],v[0],
            v[4],v[5], v[5],v[6], v[6],v[7], v[7],v[4],
            v[0],v[4], v[1],v[5], v[2],v[6], v[3],v[7]
        ]
        for p in lines:
            pt = Point()
            pt.x = float(p[0]); pt.y = float(p[1]); pt.z = float(p[2])
            marker.points.append(pt)
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

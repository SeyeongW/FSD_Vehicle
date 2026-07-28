#!/usr/bin/env python3
"""
bird_detector_node.py — 3-LiDAR 융합 포인트클라우드 기반 DBSCAN 클러스터 조류 검출
                         + 칼만 필터 추적

cluster_node.py에서 검출/추적 로직만 분리한 버전. 로봇 추격 제어(Pure Pursuit/PN)와
카메라 틸트 제어는 이 노드가 하지 않는다 — 각각 bird_pursuit_node.py와 벽 고정 카메라
팀의 노드가 담당한다. 이 노드는 /bird_target(world 프레임 위치+속도)만 발행하고,
누가 그걸로 뭘 하는지는 모른다.

[입력]
  /fused_pointcloud2 (world 프레임) — lidar_fusion_node가 3개 LiDAR를 합쳐서 발행
  /odom — self-point 제거(로봇 자신의 실시간 world 위치 기준)용

[칼만 필터]
  상태: [x, y, z, vx, vy, vz] (world 프레임)
  예측: 등속 운동 모델. lookahead 없이 원시 위치/속도를 그대로 발행 —
        "표적을 앞질러 겨냥"하는 건 PN 기반 bird_pursuit_node의 몫.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header

from sklearn.cluster import DBSCAN


# ============================================================
# 칼만 필터 유틸리티 (numpy 전용) — cluster_node.py와 동일
# ============================================================

def kf_init(pos: np.ndarray) -> dict:
    x = np.zeros(6, dtype=np.float64)
    x[:3] = pos
    P = np.diag([1.0, 1.0, 1.0, 5.0, 5.0, 5.0])
    return {'x': x, 'P': P}


def kf_predict(kf: dict, dt: float) -> dict:
    dt = min(dt, 0.5)
    F = np.eye(6, dtype=np.float64)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt

    q_p = 0.5 * dt * dt
    q_v = 2.0 * dt
    Q = np.diag([q_p, q_p, q_p, q_v, q_v, q_v])

    kf['x'] = F @ kf['x']
    kf['P'] = F @ kf['P'] @ F.T + Q
    return kf


def kf_update(kf: dict, measurement: np.ndarray) -> dict:
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


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _fast_read_xyz(msg: PointCloud2) -> np.ndarray:
    """point_step=12(x,y,z float32)일 때 numpy.frombuffer로 벡터화 파싱.
    point_cloud2.read_points()의 파이썬 루프는 최대 15만 포인트를 5Hz로 처리할 때
    CPU를 과도하게 먹어 시스템 전체 TF 발행을 지연시킴(lidar_fusion_node와 동일 문제).
    비표준 레이아웃이면 안전하게 느린 경로로 폴백."""
    if msg.point_step != 12 or len(msg.data) % 12 != 0:
        pts = np.array(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.stack([pts['x'], pts['y'], pts['z']], axis=-1)

    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
    valid = ~np.isnan(arr).any(axis=1)
    return arr[valid]


def _make_cloud_xyz32(header, points: np.ndarray) -> PointCloud2:
    """point_cloud2.create_cloud_xyz32()의 포인트별 파이썬 루프를 피하려고 직접 패킹
    (lidar_fusion_node._make_cloud_xyz32와 동일 패턴)."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class BirdDetectorNode(Node):
    def __init__(self):
        super().__init__('bird_detector_node')

        # -------------------------
        # ROS 인터페이스
        # -------------------------
        self.sub = self.create_subscription(
            PointCloud2, '/fused_pointcloud2', self.callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.marker_pub = self.create_publisher(Marker, '/cluster_markers', 10)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.bird_detected_pub = self.create_publisher(Bool, '/bird_detected', 10)
        self.target_pub = self.create_publisher(PointStamped, '/bird_target', 10)
        self.target_vel_pub = self.create_publisher(Vector3Stamped, '/bird_target_velocity', 10)

        # -------------------------
        # 파라미터
        # -------------------------
        # 'odom'을 공통 좌표계로 사용. 'map'은 AMCL이 /scan을 받아야만 map->odom을
        # 발행하는데 그 /scan은 이 융합 파이프라인이 만들어야 해서 순환 의존이 생김.
        # 새 검출/추격은 로봇 기준 로컬 동작이라 전역 map 프레임이 필요 없음.
        self.world_frame = 'odom'

        self.ground_z_limit = 2.1   # 벽 높이(2.0m) 위쪽만 남김 (바닥/벽 클러터 제거)

        # self-point 제거: 로봇의 실시간 world 위치 기준 동적 박스 (로봇 자신이 world
        # 프레임 안을 돌아다니므로, 로컬 프레임 전제였던 기존 고정 박스는 못 씀)
        self.self_half_x = 0.6
        self.self_half_y = 0.6
        self.self_z = (-0.3, 0.8)

        self.dbscan_eps = 0.6
        self.dbscan_min_samples = 3
        self.min_cluster_points = 3

        self.trackable_max_size_x = 2.0
        self.trackable_max_size_y = 2.0
        self.trackable_max_size_z = 1.5
        self.trackable_min_centroid_z = 2.0

        # 라이더 실제 갱신 주기가 초 단위(샘플 수 줄여도 3~7초대)라, 그 사이 새가
        # 3m 넘게 이동하는 경우가 흔해서 3.0m로는 매 갱신마다 트랙 ID가 리셋됐음
        # (칼만필터 속도 추정치도 같이 리셋되어 PN 추격이 매끄럽지 못한 원인).
        self.track_match_dist = 8.0
        self.move_threshold = 0.05
        self.max_motion_dist = 8.0
        self.min_motion_frames = 3
        self.max_missed_frames = 60

        self.lock_lost_frames = 50
        self.lock_keep_missed = 20
        self.lock_switch_cooldown = 20
        self.lock_age = 0

        self.rotation_freeze_threshold = 0.20

        # 코스팅(칼만필터 예측만, 실측 보정 없음) 중 방 경계 밖으로 발산한 트랙을
        # 걸러내기 위한 상한 — lidar_fusion_node의 bounds_xy(20x20 방 기준)와 동일 값.
        # 실측 없이 등속 모델로만 최대 lock_keep_missed(~4~5초) 프레임을 외삽하다보면
        # 속도 추정치가 조금만 커도 방 밖까지 선형으로 발산해, 그 위치를 그대로
        # /bird_target으로 내보내면 추격 로봇이 벽으로 돌진하는 사고로 이어진다.
        self.world_bounds_xy = 11.0

        self.declare_parameter('use_kf', True)
        self.use_kf = self.get_parameter('use_kf').value

        # ── 런타임 상태 ──
        self.tracks = []
        self.next_track_id = 0

        self.locked_target_id = None
        self.locked_target_missed = 0

        self.current_angular_z = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0

        self.display_clusters = []
        self.last_cb_time = None
        self.last_data_stamp_sec = None

        self.create_timer(0.5, self._bird_coord_log_callback)

        self.get_logger().info('Bird detector node (DBSCAN + KF, world frame) started.')

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

    def callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        if self.last_cb_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_cb_time).nanoseconds * 1e-9
        self.last_cb_time = now

        # lidar_fusion_node는 원본 라이더가 아직 새 프레임을 안 냈어도 5Hz 타이머로
        # 직전과 동일한(stale) 데이터를 계속 재발행한다. 이걸 그대로 DBSCAN+kf_update에
        # 태우면, 실제로는 안 움직인 것도 아닌데 "위치가 그대로"라는 반복 관측이 계속
        # 들어가서 칼만필터 속도 추정치가 자꾸 0쪽으로 끌려간다. header.stamp가 직전과
        # 같으면 새 데이터가 아니므로 예측(coast)만 하고 보정은 건너뛴다.
        data_stamp_sec = _stamp_to_sec(msg.header.stamp)
        if self.last_data_stamp_sec is not None and data_stamp_sec == self.last_data_stamp_sec:
            self.update_tracks_no_detection(dt)
            self.select_and_track_target()
            return
        self.last_data_stamp_sec = data_stamp_sec

        all_points = _fast_read_xyz(msg)
        points = all_points[all_points[:, 2] > self.ground_z_limit]

        if len(points) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, self.world_frame)
            self.publish_filtered_points(points, msg.header.stamp)
            self.update_tracks_no_detection(dt)
            self.select_and_track_target()
            return

        non_self = self.remove_self_points(points)

        if len(non_self) < self.min_cluster_points:
            self.display_clusters = []
            self.clear_markers(msg.header.stamp, self.world_frame)
            self.publish_filtered_points(non_self, msg.header.stamp)
            self.update_tracks_no_detection(dt)
            self.select_and_track_target()
            return

        if len(non_self) > 3000:
            non_self = non_self[::2]

        self.publish_filtered_points(non_self, msg.header.stamp)

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

            world_centroid = np.mean(cluster, axis=0)
            cx, cy, cz = world_centroid
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
                'id': int(label), 'world_centroid': world_centroid, 'bbox': (sx, sy, sz)
            })

            detections.append({
                'world_centroid': world_centroid,
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
            if self.use_kf and dt > 0.0 and 'kf' in tr:
                tr['kf'] = kf_predict(tr['kf'], dt)

        candidate_pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, det in enumerate(detections):
                kf_pos = tr['kf']['x'][:3] if 'kf' in tr else tr['world_centroid']
                dist = np.linalg.norm(det['world_centroid'] - kf_pos)
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

            prev_world = track['world_centroid'].copy()
            new_world = det['world_centroid']
            motion_dist = np.linalg.norm(new_world - prev_world)

            track['world_centroid'] = new_world
            track['bbox'] = det['bbox']
            track['age'] += 1
            track['missed'] = 0
            track['seen_count'] += 1
            track['last_motion_dist'] = motion_dist

            if abs(self.current_angular_z) <= self.rotation_freeze_threshold:
                if self.move_threshold < motion_dist < self.max_motion_dist:
                    track['moving_count'] += 1
                track['is_moving_confirmed'] = track['moving_count'] >= self.min_motion_frames

            if self.use_kf and 'kf' in track:
                track['kf'] = kf_update(track['kf'], new_world)

            unmatched_det_indices.discard(di)
            used_tracks.add(ti)
            used_dets.add(di)

        # 잠금 타깃이 이번 라운드에 track_match_dist를 넘겨서 매칭 실패했더라도, 씬에
        # 새는 한 마리뿐이므로 남은 미매칭 후보 중 가장 가까운 걸 "같은 새"로 보고
        # 새 ID를 발급하는 대신 잠금 트랙 자체를 그 위치로 이어붙인다. 그냥 새 트랙을
        # 만들면 나이/속도 이력이 전부 버려져서(칼만필터 속도 0부터 재시작) ID가 계속
        # 올라가고 PN 추격이 매번 끊긴다.
        locked_track = (
            self.get_track_by_id(self.locked_target_id)
            if self.locked_target_id is not None else None
        )
        if locked_track is not None and locked_track['missed'] > 0 and unmatched_det_indices:
            di = min(
                unmatched_det_indices,
                key=lambda i: np.linalg.norm(detections[i]['world_centroid'] - locked_track['world_centroid']),
            )
            det = detections[di]
            prev_world = locked_track['world_centroid'].copy()
            new_world = det['world_centroid']
            motion_dist = np.linalg.norm(new_world - prev_world)

            locked_track['world_centroid'] = new_world
            locked_track['bbox'] = det['bbox']
            locked_track['age'] += 1
            locked_track['missed'] = 0
            locked_track['seen_count'] += 1
            locked_track['last_motion_dist'] = motion_dist

            if abs(self.current_angular_z) <= self.rotation_freeze_threshold:
                if self.move_threshold < motion_dist < self.max_motion_dist:
                    locked_track['moving_count'] += 1
                locked_track['is_moving_confirmed'] = locked_track['moving_count'] >= self.min_motion_frames

            if self.use_kf and 'kf' in locked_track:
                locked_track['kf'] = kf_update(locked_track['kf'], new_world)

            unmatched_det_indices.discard(di)

        for di in unmatched_det_indices:
            det = detections[di]
            new_track = {
                'id': self.next_track_id,
                'world_centroid': det['world_centroid'],
                'bbox': det['bbox'],
                'age': 1, 'missed': 0, 'seen_count': 1, 'moving_count': 1,
                'last_motion_dist': self.move_threshold + 0.01,
                'is_moving_confirmed': self.min_motion_frames <= 1,
            }
            if self.use_kf:
                new_track['kf'] = kf_init(det['world_centroid'])
            self.tracks.append(new_track)
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
            if self.use_kf and dt > 0.0 and 'kf' in tr:
                tr['kf'] = kf_predict(tr['kf'], dt)
        self.tracks = [tr for tr in self.tracks if tr['missed'] <= self.max_missed_frames]
        if self.locked_target_id is not None:
            self.locked_target_missed += 1
            if self.locked_target_missed > self.lock_lost_frames:
                self.locked_target_id = None
                self.locked_target_missed = 0
                self.lock_age = 0

    # =========================================================
    # 타깃 선택
    # =========================================================
    def select_and_track_target(self):
        rotating = abs(self.current_angular_z) > self.rotation_freeze_threshold

        locked = None
        if self.locked_target_id is not None:
            locked = self.get_track_by_id(self.locked_target_id)

        if locked is not None and locked['missed'] > 0 and self._track_out_of_bounds(locked):
            # 실측 보정 없이 코스팅만 하다 방 경계 밖으로 발산 — 더 이상 신뢰 불가.
            # 트랙 자체를 버려서 잠금을 풀고, 다음 실측 탐지가 새 트랙으로 잡히게 한다.
            self.tracks = [tr for tr in self.tracks if tr['id'] != locked['id']]
            self.locked_target_id = None
            self.locked_target_missed = 0
            self.lock_age = 0
            locked = None

        if locked is not None and locked['missed'] <= self.lock_keep_missed:
            self.lock_age += 1
            self._publish_bird_detected(True)
            self.publish_bird_target(locked)
            return

        if rotating:
            # 회전 중엔 odom 노이즈로 오탐 위험 — 새 타깃 잠금은 보류(기존 잠금 유지 로직은 위에서 처리됨)
            return

        candidates = [
            tr for tr in self.tracks
            if tr['missed'] == 0 and (
                tr['is_moving_confirmed'] or tr['last_motion_dist'] > self.move_threshold
            )
        ]

        if not candidates:
            if self.locked_target_id is not None:
                self._publish_bird_detected(True)
            else:
                self._publish_bird_detected(False)
            return

        def candidate_score(tr):
            # 로봇 기준 상대 방위각/거리로 점수 매김 (정면에 가깝고 가까운 후보 우선)
            dx = tr['world_centroid'][0] - self.robot_x
            dy = tr['world_centroid'][1] - self.robot_y
            cos_yaw = math.cos(-self.robot_yaw)
            sin_yaw = math.sin(-self.robot_yaw)
            local_x = cos_yaw * dx - sin_yaw * dy
            local_y = sin_yaw * dx + cos_yaw * dy
            angle = abs(math.atan2(local_y, local_x))
            dist = math.sqrt(local_x * local_x + local_y * local_y)
            score = angle + 0.15 * dist
            if self.locked_target_id is not None:
                old_locked = self.get_track_by_id(self.locked_target_id)
                if old_locked is not None:
                    lock_dist = np.linalg.norm(tr['world_centroid'] - old_locked['world_centroid'])
                    score += 0.05 * lock_dist
            return score

        if self.locked_target_id is not None and self.lock_age < self.lock_switch_cooldown:
            old_locked = self.get_track_by_id(self.locked_target_id)
            if old_locked is not None:
                nearby = [(np.linalg.norm(tr['world_centroid'] - old_locked['world_centroid']), tr)
                          for tr in candidates
                          if np.linalg.norm(tr['world_centroid'] - old_locked['world_centroid']) < self.track_match_dist]
                best = nearby[0][1] if nearby else min(candidates, key=candidate_score)
            else:
                best = min(candidates, key=candidate_score)
        else:
            best = min(candidates, key=candidate_score)

        self.locked_target_id = best['id']
        self.locked_target_missed = 0
        self.lock_age = 0

        self._publish_bird_detected(True)
        self.publish_bird_target(best)

    def get_track_by_id(self, track_id):
        for tr in self.tracks:
            if tr['id'] == track_id:
                return tr
        return None

    def _track_out_of_bounds(self, track: dict) -> bool:
        pos = track['kf']['x'][:3] if 'kf' in track else track['world_centroid']
        return abs(pos[0]) > self.world_bounds_xy or abs(pos[1]) > self.world_bounds_xy

    def _publish_bird_detected(self, detected: bool):
        msg = Bool()
        msg.data = detected
        self.bird_detected_pub.publish(msg)

    def publish_bird_target(self, track: dict):
        """KF 위치/속도(lookahead 없는 원시값)를 각각 world 프레임으로 발행.
        표적을 앞질러 겨냥하는 건 이 값을 받는 bird_pursuit_node(PN)의 몫."""
        if 'kf' in track:
            pos = track['kf']['x'][:3]
            vel = track['kf']['x'][3:6]
        else:
            pos = track['world_centroid']
            vel = np.zeros(3)

        now = self.get_clock().now().to_msg()

        pos_msg = PointStamped()
        pos_msg.header.stamp = now
        pos_msg.header.frame_id = self.world_frame
        pos_msg.point.x = float(pos[0])
        pos_msg.point.y = float(pos[1])
        pos_msg.point.z = float(pos[2])
        self.target_pub.publish(pos_msg)

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = now
        vel_msg.header.frame_id = self.world_frame
        vel_msg.vector.x = float(vel[0])
        vel_msg.vector.y = float(vel[1])
        vel_msg.vector.z = float(vel[2])
        self.target_vel_pub.publish(vel_msg)

    def _bird_coord_log_callback(self):
        """0.5초 간격으로 잠금 타깃의 world 좌표 + 로봇 기준 거리/방위각 로그 출력."""
        if self.locked_target_id is None:
            return
        track = self.get_track_by_id(self.locked_target_id)
        if track is None:
            return
        if track['missed'] > self.lock_keep_missed:
            return

        pos = track['kf']['x'][:3] if 'kf' in track else track['world_centroid']

        dx = pos[0] - self.robot_x
        dy = pos[1] - self.robot_y
        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy

        dist_2d = math.sqrt(dx * dx + dy * dy)
        bearing_deg = math.degrees(math.atan2(local_y, local_x))

        self.get_logger().info(
            f'[BIRD] world x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} '
            f'| dist={dist_2d:.2f}m bearing={bearing_deg:+.1f}deg '
            f'| id={self.locked_target_id}'
        )

    # =========================================================
    # 포인트 유틸리티
    # =========================================================
    def remove_self_points(self, points: np.ndarray) -> np.ndarray:
        """로봇의 실시간 world 위치 주변 박스 안 포인트를 제거 (로봇 자신의 반사)."""
        mask = ~(
            (self.robot_x - self.self_half_x < points[:, 0]) & (points[:, 0] < self.robot_x + self.self_half_x) &
            (self.robot_y - self.self_half_y < points[:, 1]) & (points[:, 1] < self.robot_y + self.self_half_y) &
            (self.self_z[0] < points[:, 2]) & (points[:, 2] < self.self_z[1])
        )
        return points[mask]

    def publish_filtered_points(self, points: np.ndarray, stamp):
        """DBSCAN에 실제로 들어가는 필터링 결과(높이+자기포인트 제거+다운샘플 후)를
        그대로 내보내서 rviz2에서 검증할 수 있게 함."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self.world_frame
        self.filtered_pub.publish(_make_cloud_xyz32(header, points))

    # =========================================================
    # 마커 퍼블리시
    # =========================================================
    def publish_all_markers(self, header):
        self.clear_markers(header.stamp, self.world_frame)

        locked_track = None
        if self.locked_target_id is not None:
            locked_track = self.get_track_by_id(self.locked_target_id)

        locked_display_id = None
        if locked_track is not None and locked_track['missed'] <= self.lock_keep_missed:
            best_dist = float('inf')
            for dc in self.display_clusters:
                d = np.linalg.norm(dc['world_centroid'] - locked_track['world_centroid'])
                if d < best_dist:
                    best_dist = d
                    locked_display_id = dc['id']

        for dc in self.display_clusters:
            cx, cy, cz = dc['world_centroid']
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
        dc = display_cluster['world_centroid']
        best_track = None
        best_dist = float('inf')
        for tr in self.tracks:
            if tr['missed'] > 0:
                continue
            dist = np.linalg.norm(dc - tr['world_centroid'])
            if dist < best_dist:
                best_dist = dist
                best_track = tr
        return best_track if best_dist < 2.5 else None

    def publish_box_marker(self, cx, cy, cz, sx, sy, sz, m_id, header, color):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = header.stamp
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
            pt.x = float(p[0]); pt.y = float(p[1]); pt.z = float(p[2])
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def clear_markers(self, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BirdDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

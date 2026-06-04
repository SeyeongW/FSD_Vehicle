#!/usr/bin/env python3
"""
data_logger_node.py — 논문용 CSV 데이터 로거 (10 Hz)

[CSV 컬럼]
  timestamp       — 시작부터 경과 시간 (s)
  bird_gt_x/y/z   — bird_manager 실제 위치 / ground truth (m)
  bird_kf_x/y/z   — cluster_node 칼만 필터 추정 위치 (m)
  pos_error       — GT vs KF 3D 위치 오차 (m)
  robot_x/y       — 로봇 위치 odom 기준 (m)
  robot_yaw_deg   — 로봇 방향 (deg, +CCW)
  distance        — 로봇-조류 수평 거리 (m)
  bearing_deg     — 로봇 정면 기준 조류 방향 (deg, +좌/-우)
  detected        — LiDAR 조류 감지 여부
  tracking_active — 추적 모드 여부

[사용]
  ros2 run pcd_cluster_pkg data_logger_node
  또는 patrol_nav.launch.py log_data:=true 옵션으로 자동 시작

[출력 경로]
  ~/ros2_ws/ugv_ws/logs/bird_tracking_YYYYMMDD_HHMMSS.csv
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class DataLoggerNode(Node):

    LOG_HZ = 10.0

    def __init__(self):
        super().__init__('data_logger')

        # ── 최신 수신 값 ──────────────────────────────────────────
        self._bird_gt  = None   # (x, y, z) — ground truth
        self._bird_kf  = None   # (x, y, z) — KF 추정
        self._robot    = None   # (x, y, yaw_rad)
        self._detected = False
        self._tracking = False
        self._t0       = None   # 첫 로그 시각

        # ── 구독 ──────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/bird/nearest_pose', self._cb_gt, 10)
        self.create_subscription(
            PointStamped, '/bird_target', self._cb_kf, 10)
        self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(
            Bool, '/bird_detected', self._cb_detected, 10)
        self.create_subscription(
            Bool, '/tracking_active', self._cb_tracking, 10)

        # ── CSV 초기화 ────────────────────────────────────────────
        self.declare_parameter('use_kf', True)
        self.declare_parameter('kf_lookahead_sec', 0.15)
        self.declare_parameter('bird_speed', 0.18)
        use_kf     = self.get_parameter('use_kf').value
        kf_la      = self.get_parameter('kf_lookahead_sec').value
        bird_speed = self.get_parameter('bird_speed').value

        stamp   = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/ros2_ws/ugv_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        kf_cond = 'nokf' if not use_kf else f'kf{kf_la:.2f}'
        self._path = os.path.join(log_dir, f'exp_spd{bird_speed:.2f}_{kf_cond}_{stamp}.csv')

        self._file   = open(self._path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            'timestamp',
            'bird_gt_x', 'bird_gt_y', 'bird_gt_z',
            'bird_kf_x', 'bird_kf_y', 'bird_kf_z',
            'pos_error',
            'robot_x', 'robot_y', 'robot_yaw_deg',
            'distance', 'bearing_deg',
            'detected', 'tracking_active',
        ])
        self._file.flush()

        self.get_logger().info(f'[DataLogger] 로깅 시작 → {self._path}')
        self.create_timer(1.0 / self.LOG_HZ, self._write_row)

    # ── 콜백 ──────────────────────────────────────────────────────

    def _cb_gt(self, msg: PoseStamped):
        p = msg.pose.position
        self._bird_gt = (p.x, p.y, p.z)

    def _cb_kf(self, msg: PointStamped):
        self._bird_kf = (msg.point.x, msg.point.y, msg.point.z)

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot = (p.x, p.y, math.atan2(siny, cosy))

    def _cb_detected(self, msg: Bool):
        self._detected = msg.data

    def _cb_tracking(self, msg: Bool):
        self._tracking = msg.data

    # ── 행 기록 ───────────────────────────────────────────────────

    def _write_row(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        gt, kf, rob = self._bird_gt, self._bird_kf, self._robot

        # ground truth
        gx, gy, gz = gt  if gt  else (float('nan'),) * 3
        # KF 추정
        kx, ky, kz = kf  if kf  else (float('nan'),) * 3
        # 로봇
        rx, ry, ryaw = rob if rob else (float('nan'),) * 3

        # GT vs KF 오차
        if gt and kf:
            pos_err = math.sqrt((gx-kx)**2 + (gy-ky)**2 + (gz-kz)**2)
        else:
            pos_err = float('nan')

        # 로봇-조류 거리 / 베어링
        if kf and rob:
            dx, dy = kx - rx, ky - ry
            dist    = math.sqrt(dx*dx + dy*dy)
            bear    = math.degrees(math.atan2(dy, dx) - ryaw)
            # -180 ~ 180 정규화
            while bear >  180: bear -= 360
            while bear < -180: bear += 360
        else:
            dist = bear = float('nan')

        yaw_deg = math.degrees(ryaw) if rob else float('nan')

        self._writer.writerow([
            f'{t:.3f}',
            f'{gx:.4f}', f'{gy:.4f}', f'{gz:.4f}',
            f'{kx:.4f}', f'{ky:.4f}', f'{kz:.4f}',
            f'{pos_err:.4f}',
            f'{rx:.4f}', f'{ry:.4f}', f'{yaw_deg:.2f}',
            f'{dist:.4f}', f'{bear:.2f}',
            int(self._detected), int(self._tracking),
        ])
        self._file.flush()

    # ── 종료 처리 ──────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self._file.close()
            self.get_logger().info(f'[DataLogger] 저장 완료 → {self._path}')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""
eval_logger_node.py — 졸업논문 단일/다중 LiDAR 비교 실험용 CSV 로거 (자체 10Hz 타이머)

융합 노드가 아예 발행을 멈춘 경우(단일 LiDAR 고장)에도 "탐지 실패" 행이 기록되도록
자체 타이머로 기록하고, 최신 /thesis/fusion_stats가 stats_timeout_sec보다 오래되면
모든 소스를 dead/hits=0으로 기록한다. 새가 등장한 이후 구간만 기록.

[CSV 컬럼]
  t_sim, bird_t(새 등장 후 경과 s), fault_active,
  gt_x/y/z, est_x/y/z, est_age(s), pos_err(m), tracked(최신 추정이 GT 1m 이내),
  bird_detected, tracking_active, robot_x/y,
  n_alive, n_points, total_hits, proc_ms, latency_ms(추정 발행 시각 - 센서 stamp, sim time),
  <src>_alive, <src>_dist, <src>_hits  (소스별)

[출력] <out_dir>/thesis_<mode>_<fault>_s<seed>_YYYYMMDD_HHMMSS.csv
"""

import csv
import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

NAN = float('nan')


class EvalLoggerNode(Node):

    def __init__(self):
        super().__init__('eval_logger_node')
        self.declare_parameter('mode', 'multi')
        self.declare_parameter('fault', 'none')
        self.declare_parameter('seed', -1)
        self.declare_parameter('source_names', ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'mid360_PointCloud2'])
        self.declare_parameter('out_dir', os.path.expanduser('~/ros2_ws/ugv_ws/logs/thesis'))
        self.declare_parameter('log_rate_hz', 10.0)
        self.declare_parameter('stats_timeout_sec', 0.3)
        self.declare_parameter('est_timeout_sec', 0.5)
        self.declare_parameter('track_err_thresh', 1.0)
        self.declare_parameter('fault_delay_sec', 60.0)

        self.sources = list(self.get_parameter('source_names').value)
        self.stats_timeout = float(self.get_parameter('stats_timeout_sec').value)
        self.est_timeout = float(self.get_parameter('est_timeout_sec').value)
        self.track_thresh = float(self.get_parameter('track_err_thresh').value)
        self.fault_delay = float(self.get_parameter('fault_delay_sec').value)

        self.stats, self.stats_rx = None, None
        self.est, self.est_rx, self.latency_ms = None, None, NAN
        self.gt, self.visible, self.first_seen = None, False, None
        self.detected, self.tracking = False, False
        self.robot = (NAN, NAN)
        self.fault_active = False

        self.create_subscription(String, '/thesis/fusion_stats', self._on_stats, 10)
        self.create_subscription(PointStamped, '/bird_target', self._on_est, 10)
        self.create_subscription(PoseStamped, '/bird/nearest_pose', self._on_gt, 10)
        self.create_subscription(Bool, '/bird/visible', self._on_visible, 10)
        self.create_subscription(Bool, '/bird_detected', lambda m: setattr(self, 'detected', m.data), 10)
        self.create_subscription(Bool, '/tracking_active', lambda m: setattr(self, 'tracking', m.data), 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        mode = self.get_parameter('mode').value
        fault = self.get_parameter('fault').value
        seed = self.get_parameter('seed').value
        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(out_dir, f'thesis_{mode}_{fault}_s{seed}_{stamp}.csv')
        self.file = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        header = ['t_sim', 'bird_t', 'fault_active',
                  'gt_x', 'gt_y', 'gt_z', 'est_x', 'est_y', 'est_z', 'est_age', 'pos_err', 'tracked',
                  'bird_detected', 'tracking_active', 'robot_x', 'robot_y',
                  'n_alive', 'n_points', 'total_hits', 'proc_ms', 'latency_ms']
        for s in self.sources:
            short = s.replace('_PointCloud2', '')
            header += [f'{short}_alive', f'{short}_dist', f'{short}_hits']
        self.writer.writerow(header)

        self.create_timer(1.0 / float(self.get_parameter('log_rate_hz').value), self._write)
        self.get_logger().info(f'[평가로거] 기록 시작 → {self.path}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_stats(self, msg: String):
        self.stats = json.loads(msg.data)
        self.stats_rx = self._now()
        self.fault_active = self.fault_active or self.stats['fault_active']

    def _on_est(self, msg: PointStamped):
        now = self._now()
        self.est = (msg.point.x, msg.point.y, msg.point.z)
        self.est_rx = now
        self.latency_ms = (now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)) * 1e3

    def _on_gt(self, msg: PoseStamped):
        p = msg.pose.position
        self.gt = (p.x, p.y, p.z)

    def _on_visible(self, msg: Bool):
        self.visible = msg.data
        if msg.data and self.first_seen is None:
            self.first_seen = self._now()

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y)

    def _write(self):
        if not self.visible or self.gt is None:
            return
        now = self._now()
        gx, gy, gz = self.gt
        # 단일 LiDAR 고장 시 융합 노드가 통계 발행을 멈춰 fault_active=True를 못 받으므로 시간으로도 판정
        fault = self.get_parameter('fault').value
        if fault != 'none' and now - self.first_seen >= self.fault_delay:
            self.fault_active = True

        est_fresh = self.est is not None and now - self.est_rx <= self.est_timeout
        if est_fresh:
            ex, ey, ez = self.est
            err = math.sqrt((ex - gx) ** 2 + (ey - gy) ** 2 + (ez - gz) ** 2)
            age = now - self.est_rx
        else:
            ex = ey = ez = err = age = NAN
        tracked = est_fresh and err <= self.track_thresh

        stats_fresh = self.stats is not None and now - self.stats_rx <= self.stats_timeout
        src_cols, total_hits = [], 0
        for s in self.sources:
            e = self.stats['sources'].get(s) if self.stats else None
            # 원점-GT 거리는 기하 정보라 stats가 끊겨도 마지막 값 유지 (거리별 분석용)
            dist = e['dist'] if e and e['dist'] is not None else NAN
            alive = bool(stats_fresh and e and e['alive'])
            hits = e['hits'] if (alive and e) else 0
            total_hits += hits
            src_cols += [int(alive), dist, hits]

        row = [now, now - self.first_seen, int(self.fault_active),
               gx, gy, gz, ex, ey, ez, age, err, int(tracked),
               int(self.detected), int(self.tracking), self.robot[0], self.robot[1],
               self.stats['n_alive'] if stats_fresh else 0,
               self.stats['n_points'] if stats_fresh else 0,
               total_hits,
               self.stats['proc_ms'] if stats_fresh else NAN,
               self.latency_ms if est_fresh else NAN] + src_cols
        self.writer.writerow(row)
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = EvalLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.get_logger().info(f'[평가로거] 종료 → {node.path}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

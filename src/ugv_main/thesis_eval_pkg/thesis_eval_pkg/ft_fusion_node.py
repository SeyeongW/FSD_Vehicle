"""
ft_fusion_node.py — 고장 허용(fault-tolerant) 다중 LiDAR 융합 + 고장 주입 + 소스별 평가 통계

[기존 lidar_fusion_node와의 차이]
  lidar_fusion_node는 ApproximateTimeSynchronizer로 모든 소스가 다 들어와야만 발행한다
  → 소스 하나만 멈춰도 /fused_pointcloud2 전체가 멈춤(다중 센서인데 단일 고장점이 3개).
  이 노드는 소스별 최신 메시지를 캐시하고, 고정 주기(publish_rate_hz)마다
  "가장 최신 소스 대비 stale_timeout_sec 이내"인 소스만 골라 병합한다.
  → 소스가 고장 나면 timeout 후 자동 제외되고 나머지 소스로 계속 발행(graceful degradation).
  input_topics에 소스 하나만 주면 단일 LiDAR 시스템과 동일한 경로로 동작한다(비교 실험 공정성).

[고장 주입]
  fault_source(토픽명)를 지정하면, 새가 처음 등장(/bird/visible=True)한 시점부터
  fault_delay_sec 뒤에 그 소스의 메시지를 전부 버린다(=센서 고장/통신 단절과 동일 효과).

[평가 통계 /thesis/fusion_stats (std_msgs/String, JSON)]
  발행할 때마다 소스별 alive 여부, 새(GT)와 센서 원점 사이 거리, GT 반경 hit_radius 이내
  포인트 수(hits), 처리 시간(ms)을 내보낸다. eval_logger_node가 CSV로 기록.
  GT는 시뮬레이션 전용(bird_manager의 /bird/nearest_pose, world 프레임 ≈ odom).
"""

import json
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String
import tf2_ros

from lidar_fusion_pkg.lidar_fusion_node import (
    _fast_read_xyz, _make_cloud_xyz32, _quat_to_rotation_matrix, _stride_downsample,
)


def _sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FtFusionNode(Node):

    def __init__(self):
        super().__init__('ft_fusion_node')

        self.declare_parameter('input_topics', ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'mid360_PointCloud2'])
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('output_topic', 'fused_pointcloud2')
        self.declare_parameter('max_points_per_source', [8000, 8000, 15000])
        self.declare_parameter('bounds_xy', 11.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        # 소스 주기 ~8Hz(125ms) 기준, 2~3 주기 동안 안 들어오면 고장으로 판단
        self.declare_parameter('stale_timeout_sec', 0.3)
        self.declare_parameter('fault_source', 'none')
        self.declare_parameter('fault_delay_sec', 60.0)
        self.declare_parameter('gt_topic', '/bird/nearest_pose')
        self.declare_parameter('gt_visible_topic', '/bird/visible')
        # 0.5m 큐브 반대각(0.433m) + 노이즈 여유
        self.declare_parameter('hit_radius', 0.5)

        self.topics = list(self.get_parameter('input_topics').value)
        self.target_frame = self.get_parameter('target_frame').value
        caps = [int(c) for c in self.get_parameter('max_points_per_source').value]
        caps += [caps[-1]] * (len(self.topics) - len(caps))
        self.caps = dict(zip(self.topics, caps))
        self.bounds_xy = float(self.get_parameter('bounds_xy').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_sec').value)
        self.fault_source = self.get_parameter('fault_source').value
        self.fault_delay = float(self.get_parameter('fault_delay_sec').value)
        self.hit_radius = float(self.get_parameter('hit_radius').value)

        if self.fault_source not in ('none', '') and self.fault_source not in self.topics:
            self.get_logger().warn(f'fault_source={self.fault_source}가 input_topics에 없음 — 고장 주입 안 함')

        # 소스별 상태: 원본 메시지, 변환된 odom 포인트, 센서 원점, 새 데이터 여부
        self.raw = {t: None for t in self.topics}
        self.pts = {t: None for t in self.topics}
        self.origin = {t: None for t in self.topics}
        self.stamp = {t: None for t in self.topics}
        self.dropped = {t: 0 for t in self.topics}
        self.new_data = False

        self.gt = None
        self.bird_first_seen = None   # sim time (s)
        self.fault_active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for t in self.topics:
            self.create_subscription(
                PointCloud2, t, lambda msg, topic=t: self._on_cloud(topic, msg), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.get_parameter('gt_topic').value, self._on_gt, 10)
        self.create_subscription(Bool, self.get_parameter('gt_visible_topic').value, self._on_visible, 10)

        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value,
                                         qos_profile_sensor_data)
        self.stats_pub = self.create_publisher(String, '/thesis/fusion_stats', 10)

        self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value), self._tick)

        self.get_logger().info(
            f'[고장허용 융합] 소스={self.topics}, stale_timeout={self.stale_timeout}s, '
            f'고장주입={self.fault_source} (새 등장 +{self.fault_delay}s), max pts={caps}')

    # ── 콜백 ─────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_visible(self, msg: Bool):
        if msg.data and self.bird_first_seen is None:
            self.bird_first_seen = self._now()
            self.get_logger().info(f'새 등장 감지 (t={self.bird_first_seen:.1f}s)')
        if not msg.data:
            self.gt = None

    def _on_gt(self, msg: PoseStamped):
        if self.bird_first_seen is None:
            return
        p = msg.pose.position
        self.gt = np.array([p.x, p.y, p.z])

    def _on_cloud(self, topic: str, msg: PointCloud2):
        if (not self.fault_active and topic == self.fault_source and self.bird_first_seen is not None
                and self._now() >= self.bird_first_seen + self.fault_delay):
            self.fault_active = True
            self.get_logger().warn(f'[고장 주입] {topic} 메시지 차단 시작 (t={self._now():.1f}s)')
        if self.fault_active and topic == self.fault_source:
            self.dropped[topic] += 1
            return
        self.raw[topic] = msg
        self.new_data = True

    # ── 변환/병합 ─────────────────────────────────────────
    def _transform(self, topic: str):
        """새로 들어온 원본 메시지만 odom으로 변환해서 캐시 (변환 실패 시 다음 틱 재시도)."""
        msg = self.raw[topic]
        if msg is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, msg.header.stamp)
        except tf2_ros.TransformException:
            return
        local = _stride_downsample(_fast_read_xyz(msg), self.caps[topic])
        t = tf.transform.translation
        origin = np.array([t.x, t.y, t.z])
        world = local @ _quat_to_rotation_matrix(tf.transform.rotation).T + origin
        keep = (np.abs(world[:, 0]) <= self.bounds_xy) & (np.abs(world[:, 1]) <= self.bounds_xy)
        self.pts[topic] = world[keep]
        self.origin[topic] = origin
        self.stamp[topic] = _sec(msg.header.stamp)
        self.raw[topic] = None

    def _tick(self):
        if not self.new_data:
            return
        self.new_data = False
        t0 = time.perf_counter()

        for topic in self.topics:
            self._transform(topic)

        stamps = [s for s in self.stamp.values() if s is not None]
        if not stamps:
            return
        newest = max(stamps)
        alive = [t for t in self.topics
                 if self.stamp[t] is not None and newest - self.stamp[t] <= self.stale_timeout]

        clouds = [self.pts[t] for t in alive if len(self.pts[t]) > 0]
        merged = np.concatenate(clouds, axis=0) if clouds else np.empty((0, 3), dtype=np.float32)

        header = Header()
        sec = int(newest)
        header.stamp.sec, header.stamp.nanosec = sec, int(round((newest - sec) * 1e9))
        header.frame_id = self.target_frame
        self.pub.publish(_make_cloud_xyz32(header, merged))

        proc_ms = (time.perf_counter() - t0) * 1e3
        self._publish_stats(newest, alive, len(merged), proc_ms)

    def _publish_stats(self, stamp: float, alive, n_points: int, proc_ms: float):
        sources = {}
        for t in self.topics:
            entry = {'alive': t in alive, 'hits': 0, 'dist': None}
            if self.gt is not None and self.origin[t] is not None:
                entry['dist'] = float(np.linalg.norm(self.origin[t] - self.gt))
                if t in alive and len(self.pts[t]) > 0:
                    d = np.linalg.norm(self.pts[t] - self.gt, axis=1)
                    entry['hits'] = int(np.count_nonzero(d <= self.hit_radius))
            sources[t] = entry
        stats = {
            'stamp': stamp,
            'bird_t': None if self.bird_first_seen is None else self._now() - self.bird_first_seen,
            'gt': None if self.gt is None else self.gt.tolist(),
            'fault_active': self.fault_active,
            'n_alive': len(alive),
            'n_points': n_points,
            'proc_ms': proc_ms,
            'sources': sources,
        }
        self.stats_pub.publish(String(data=json.dumps(stats)))


def main():
    rclpy.init()
    node = FtFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

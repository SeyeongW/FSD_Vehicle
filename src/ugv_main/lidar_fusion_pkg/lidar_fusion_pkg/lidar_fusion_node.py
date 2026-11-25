import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import message_filters


def _quat_to_rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _fast_read_xyz(msg: PointCloud2):
    """point_step이 12(x,y,z float32)일 때 numpy.frombuffer로 벡터화 파싱.
    point_cloud2.read_points()의 파이썬 루프 방식은 소스당 초당 수만~수십만 포인트를
    처리할 때 CPU를 과도하게 먹어서(측정: 이 노드 하나가 CPU 577% 점유, 시스템 전체
    TF 발행이 밀려 AMCL/RViz RobotModel까지 깨짐), 벡터화 파싱으로 대체함.
    비표준 레이아웃이면 안전하게 느린 경로로 폴백."""
    if msg.point_step != 12 or len(msg.data) % 12 != 0:
        pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.stack([pts['x'], pts['y'], pts['z']], axis=-1)

    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
    valid = ~np.isnan(arr).any(axis=1)
    return arr[valid]


def _stride_downsample(points: np.ndarray, cap: int) -> np.ndarray:
    """결정적(매번 같은 규칙) 다운샘플. np.random.choice는 호출마다 다른 무작위
    부분집합을 뽑아서, 로봇도 벽도 안 움직이는 정적인 장면에서조차 발행되는
    포인트가 프레임마다 계속 바뀌는 인위적 잡음을 만든다. 이게 코스트맵/AMCL
    입장에서는 "벽이 미세하게 흔들린다"로 보여서 Nav2 재계획 잦음(멈칫거림)과
    맵 프레임 흔들림의 원인이 됐다. 균등 간격 스트라이드로 대체해 같은 입력이면
    항상 같은 부분집합이 나오게 한다."""
    n = len(points)
    if n <= cap:
        return points
    idx = np.linspace(0, n - 1, cap).astype(np.int64)
    return points[idx]


def _make_cloud_xyz32(header, points: np.ndarray) -> PointCloud2:
    """create_cloud_xyz32()도 포인트별 파이썬 루프라 대량 데이터에서 느려서 직접 패킹."""
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


class LidarFusionNode(Node):
    """message_filters.ApproximateTimeSynchronizer로 세 소스를 실제 시간 동기화해서
    합친다.

    과거(2026-07-24)엔 부하가 큰 상황에서 소스별 갱신 주기가 3~7초로 서로
    무관하게 흔들려 slop 0.35s로도 35초에 1번만 매칭되는 것을 실측하고, opportunistic
    merge(소스별 최신값 캐시, 아무거나 갱신되면 즉시 병합·발행) 방식으로 대체했었다.
    이후 CPU/스레드 과점유(OpenBLAS 미제한) 문제를 해결하고 LiDAR 샘플 수를
    낮추면서 소스별 발행 주기가 ~8Hz(약 125ms)로 크게 개선됐고, 이 조건에서
    재실측한 결과 slop 0.03~0.07s만으로도 개별 소스 발행률의 ~88%가 매칭되는
    것으로 확인되어(60초간 slop=0.07s에서 424회 매칭, 7.06/s) 하드 동기화로
    되돌렸다.

    단, 세 소스 중 하나라도 완전히 멈추면(opportunistic merge와 달리) 매칭 자체가
    발생하지 않아 /fused_pointcloud2가 함께 멈춘다 — 소스 하나가 죽어도 나머지로
    계속 발행되던 이전 방식과의 트레이드오프.
    """

    def __init__(self):
        super().__init__('lidar_fusion_node')

        self.declare_parameter(
            'input_topics',
            ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'lidar3_PointCloud2'],
        )
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('output_topic', 'fused_pointcloud2')
        # input_topics 순서와 1:1 대응하는 소스별 최대 포인트 수. 길이가 안 맞으면
        # 마지막 값을 나머지 소스에도 그대로 씀.
        self.declare_parameter('max_points_per_source', [50000])
        # world 프레임 기준 |x|, |y| 경계. 방 크기 밖 포인트는 손상된 값으로 간주하고 버림.
        self.declare_parameter('bounds_xy', 8.0)
        # ApproximateTimeSynchronizer 허용 오차. 60초 실측(전체 파이프라인 부하 상태,
        # 소스별 발행률 ~8Hz)에서 0.07s와 0.03s가 동일하게 424회(7.06/s, 개별 소스
        # 발행률의 ~88%)를 매칭해 이 값으로 이미 충분함을 확인. 약간의 여유를 둠.
        self.declare_parameter('sync_slop', 0.07)
        self.declare_parameter('sync_queue_size', 30)

        input_topics = list(self.get_parameter('input_topics').value)
        self.target_frame = self.get_parameter('target_frame').value
        output_topic = self.get_parameter('output_topic').value
        max_points_list = list(self.get_parameter('max_points_per_source').value)
        if len(max_points_list) < len(input_topics):
            max_points_list += [max_points_list[-1]] * (len(input_topics) - len(max_points_list))
        self.max_points = [int(n) for n in max_points_list[:len(input_topics)]]
        self.bounds_xy = self.get_parameter('bounds_xy').value
        sync_slop = self.get_parameter('sync_slop').value
        sync_queue_size = self.get_parameter('sync_queue_size').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.input_topics = input_topics
        self.filters = [
            message_filters.Subscriber(self, PointCloud2, topic, qos_profile=qos_profile_sensor_data)
            for topic in input_topics
        ]
        self.ts = message_filters.ApproximateTimeSynchronizer(
            self.filters, queue_size=sync_queue_size, slop=sync_slop)
        self.ts.registerCallback(self._on_synced)

        # RELIABLE(기본 QoS)로 최대 480KB(4만 포인트)짜리 클라우드를 이 빈도로 보내면
        # publish() 자체가 (프로파일링 실측) 최대 100ms까지 걸려 CPU를 잡아먹는다 —
        # 입력 구독과 동일하게 BEST_EFFORT로 맞춰서 이 비용을 없앤다.
        self.pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'Synchronizing {list(input_topics)} -> /{output_topic} in frame "{self.target_frame}" '
            f'(slop={sync_slop}s, queue={sync_queue_size}; '
            f'max pts/source: {self.max_points}, bounds +-{self.bounds_xy}m)'
        )

    def _on_synced(self, *msgs: PointCloud2):
        clouds = []
        latest_stamp = None
        for topic, cap, msg in zip(self.input_topics, self.max_points, msgs):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, msg.header.stamp)
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'TF lookup failed for {msg.header.frame_id}: {ex}')
                continue

            local = _fast_read_xyz(msg)
            if local.shape[0] == 0:
                continue

            local = _stride_downsample(local, cap)

            t = transform.transform.translation
            R = _quat_to_rotation_matrix(transform.transform.rotation)
            world = local @ R.T + np.array([t.x, t.y, t.z])

            in_bounds = (np.abs(world[:, 0]) <= self.bounds_xy) & (np.abs(world[:, 1]) <= self.bounds_xy)
            clouds.append(world[in_bounds])

            if latest_stamp is None or (msg.header.stamp.sec, msg.header.stamp.nanosec) > latest_stamp:
                latest_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        if not clouds:
            return

        header = Header()
        header.stamp.sec, header.stamp.nanosec = latest_stamp
        header.frame_id = self.target_frame

        merged = np.concatenate(clouds, axis=0)
        self.pub.publish(_make_cloud_xyz32(header, merged))


def main():
    rclpy.init()
    node = LidarFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

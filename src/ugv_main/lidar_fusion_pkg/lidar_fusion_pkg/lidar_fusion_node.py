import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros


def _quat_to_rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


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
    """Fuses each source's most recent scan into one PointCloud2 on a timer.

    Tried ApproximateTimeSynchronizer here first (requiring all 3 sources'
    headers to land within a small slop of each other), but measured actual
    publish periods are ~3-7s per source (heavy sample counts make each
    Gazebo ray sensor slow) and not mutually correlated at all -- a tight
    sync essentially never finds a match, so /fused_pointcloud2 went silent.
    Given that, each source's latest message is kept independently (no
    history/accumulation -- just overwrite) and world-transformed once, at
    its own capture timestamp, in the subscription callback. A timer just
    republishes the union of whatever is currently the latest per source.
    """

    def __init__(self):
        super().__init__('lidar_fusion_node')

        self.declare_parameter(
            'input_topics',
            ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'lidar3_PointCloud2'],
        )
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('output_topic', 'fused_pointcloud2')
        self.declare_parameter('publish_rate', 5.0)
        # input_topics 순서와 1:1 대응하는 소스별 최대 포인트 수. 길이가 안 맞으면
        # 마지막 값을 나머지 소스에도 그대로 씀.
        self.declare_parameter('max_points_per_source', [50000])
        # world 프레임 기준 |x|, |y| 경계. 방 크기 밖 포인트는 손상된 값으로 간주하고 버림.
        self.declare_parameter('bounds_xy', 8.0)

        input_topics = list(self.get_parameter('input_topics').value)
        self.target_frame = self.get_parameter('target_frame').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        max_points_list = list(self.get_parameter('max_points_per_source').value)
        if len(max_points_list) < len(input_topics):
            max_points_list += [max_points_list[-1]] * (len(input_topics) - len(max_points_list))
        self.max_points = {topic: int(n) for topic, n in zip(input_topics, max_points_list)}
        self.bounds_xy = self.get_parameter('bounds_xy').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # topic -> (t_sec, world_xyz ndarray) for that source's latest message only
        self.latest = {}
        for topic in input_topics:
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, topic=topic: self._on_cloud(topic, msg),
                qos_profile_sensor_data,
            )

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_timer(1.0 / publish_rate, self.publish_fused)

        self.get_logger().info(
            f'Fusing {list(input_topics)} -> /{output_topic} in frame "{self.target_frame}" '
            f'(max pts/source: {self.max_points}, bounds +-{self.bounds_xy}m)'
        )

    def _on_cloud(self, topic, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF lookup failed for {msg.header.frame_id}: {ex}')
            return

        local = _fast_read_xyz(msg)
        if local.shape[0] == 0:
            return

        local = _stride_downsample(local, self.max_points[topic])

        t = transform.transform.translation
        R = _quat_to_rotation_matrix(transform.transform.rotation)
        world = local @ R.T + np.array([t.x, t.y, t.z])

        self.latest[topic] = (_stamp_to_sec(msg.header.stamp), world)

    def publish_fused(self):
        if not self.latest:
            return

        world_chunks = [w for _, w in self.latest.values()]
        latest_stamp_sec = max(t for t, _ in self.latest.values())

        world_points = np.concatenate(world_chunks, axis=0)
        in_bounds = (np.abs(world_points[:, 0]) <= self.bounds_xy) & (np.abs(world_points[:, 1]) <= self.bounds_xy)
        world_points = world_points[in_bounds]

        header = Header()
        header.stamp.sec = int(latest_stamp_sec)
        header.stamp.nanosec = int(round((latest_stamp_sec % 1.0) * 1e9))
        header.frame_id = self.target_frame

        self.pub.publish(_make_cloud_xyz32(header, world_points))


def main():
    rclpy.init()
    node = LidarFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

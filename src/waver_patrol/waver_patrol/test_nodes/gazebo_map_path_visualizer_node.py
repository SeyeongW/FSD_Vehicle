from __future__ import annotations

import math
import os
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from waver_patrol.autonomy_common import yaw_from_quaternion


class GazeboMapPathVisualizerNode(Node):
    """Gazebo-only map/path publisher for the Waver operator panel.

    역할:
      - `ugv_gazebo/maps/map.yaml`을 `/map` OccupancyGrid로 반복 발행한다.
      - mission manager의 `/waver/active_nav_goal`과 `/odom`을 이용해 `/plan`,
        `/local_plan`을 만들어 리모콘 UI와 RViz에서 실시간 경로를 볼 수 있게 한다.
      - 실제 Nav2 planner가 아니며, 실차 launch에서는 사용하지 않는다.

    안전:
      - `/cmd_vel`을 발행하지 않는다.
      - Gazebo 사전검증에서 시각화 공백을 메우는 노드일 뿐, 실차 판단 경로에 넣지 않는다.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_map_path_visualizer_node")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("local_path_topic", "/local_plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("active_goal_topic", "/waver/active_nav_goal")
        self.declare_parameter("current_waypoint_topic", "/waver/current_waypoint")
        self.declare_parameter("dynamic_obstacle_topic", "/waver/dynamic_obstacle_map")
        self.declare_parameter("enable_dynamic_obstacle_detour", False)
        self.declare_parameter("avoidance_corridor_radius_m", 0.55)
        self.declare_parameter("avoidance_offset_m", 0.85)
        self.declare_parameter("obstacle_timeout_sec", 0.8)
        self.declare_parameter("pause_map_when_mapping_active", True)
        self.declare_parameter("mapping_active_topic", "/waver/mapping_active")
        self.declare_parameter("map_apply_state_topic", "/waver/map_apply_state")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("local_path_length_m", 1.2)
        self.declare_parameter("local_path_points", 16)
        self.declare_parameter("global_path_points", 80)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            map_qos,
        )
        self.global_path_pub = self.create_publisher(
            NavPath,
            str(self.get_parameter("global_path_topic").value),
            10,
        )
        self.local_path_pub = self.create_publisher(
            NavPath,
            str(self.get_parameter("local_path_topic").value),
            10,
        )

        self.odom: Odometry | None = None
        self.goal: PoseStamped | None = None
        self.obstacle: PointStamped | None = None
        self.last_obstacle_time = 0.0
        self.pause_fixed_map = False
        self.map_msg = self.load_map(str(self.get_parameter("map_yaml").value))

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("active_goal_topic").value),
            self.goal_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("current_waypoint_topic").value),
            self.goal_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self.obstacle_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("mapping_active_topic").value),
            self.mapping_active_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("map_apply_state_topic").value),
            self.map_apply_state_callback,
            10,
        )
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.2)
        self.create_timer(1.0 / rate, self.publish_tick)
        self.get_logger().warn("gazebo_map_path_visualizer_node is Gazebo/test visualization only")

    def odom_callback(self, msg: Odometry) -> None:
        # 역할: 로봇 현재 위치를 path 시작점으로 사용한다.
        self.odom = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        # 역할: mission/Nav2 목표를 path 끝점으로 사용한다.
        self.goal = msg

    def obstacle_callback(self, msg: PointStamped) -> None:
        self.obstacle = msg
        self.last_obstacle_time = self._now()

    def mapping_active_callback(self, msg: Bool) -> None:
        if bool(self.get_parameter("pause_map_when_mapping_active").value) and msg.data:
            self.pause_fixed_map = True

    def map_apply_state_callback(self, msg: String) -> None:
        state = msg.data.upper()
        if "OLD_MAP_UNAPPLIED" in state or "MAPPING_STARTED" in state:
            self.pause_fixed_map = True

    def publish_tick(self) -> None:
        now = self.get_clock().now().to_msg()
        if self.map_msg is not None and not self.pause_fixed_map:
            self.map_msg.header.stamp = now
            self.map_pub.publish(self.map_msg)
        if self.odom is None:
            return
        self.global_path_pub.publish(self.make_global_path(now))
        self.local_path_pub.publish(self.make_local_path(now))

    def make_global_path(self, stamp) -> NavPath:
        # 역할: odom에서 active goal까지 직선 preview path를 만든다.
        msg = NavPath()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        sx = float(self.odom.pose.pose.position.x)
        sy = float(self.odom.pose.pose.position.y)
        if self.goal is not None:
            gx = float(self.goal.pose.position.x)
            gy = float(self.goal.pose.position.y)
        else:
            yaw = yaw_from_quaternion(
                float(self.odom.pose.pose.orientation.x),
                float(self.odom.pose.pose.orientation.y),
                float(self.odom.pose.pose.orientation.z),
                float(self.odom.pose.pose.orientation.w),
            )
            gx = sx + math.cos(yaw) * 2.0
            gy = sy + math.sin(yaw) * 2.0
        count = max(int(self.get_parameter("global_path_points").value), 2)
        waypoints = self.detour_waypoints(sx, sy, gx, gy)
        if len(waypoints) == 2:
            for i in range(count):
                ratio = i / float(count - 1)
                msg.poses.append(self.pose_stamped(stamp, sx + (gx - sx) * ratio, sy + (gy - sy) * ratio, "map"))
        else:
            first = max(2, count // 2)
            second = max(2, count - first)
            (sx, sy), (mx, my), (gx, gy) = waypoints
            for i in range(first):
                ratio = i / float(first - 1)
                msg.poses.append(self.pose_stamped(stamp, sx + (mx - sx) * ratio, sy + (my - sy) * ratio, "map"))
            for i in range(1, second):
                ratio = i / float(second - 1)
                msg.poses.append(self.pose_stamped(stamp, mx + (gx - mx) * ratio, my + (gy - my) * ratio, "map"))
        return msg

    def detour_waypoints(self, sx: float, sy: float, gx: float, gy: float) -> list[tuple[float, float]]:
        if not bool(self.get_parameter("enable_dynamic_obstacle_detour").value):
            return [(sx, sy), (gx, gy)]
        if self.obstacle is None or self._now() - self.last_obstacle_time > float(self.get_parameter("obstacle_timeout_sec").value):
            return [(sx, sy), (gx, gy)]
        ox = float(self.obstacle.point.x)
        oy = float(self.obstacle.point.y)
        vx = gx - sx
        vy = gy - sy
        length = math.hypot(vx, vy)
        if length < 1e-3:
            return [(sx, sy), (gx, gy)]
        wx = ox - sx
        wy = oy - sy
        along = (wx * vx + wy * vy) / length
        lateral = abs(vx * wy - vy * wx) / length
        if along < 0.0 or along > length or lateral > float(self.get_parameter("avoidance_corridor_radius_m").value):
            return [(sx, sy), (gx, gy)]
        nx = -vy / length
        ny = vx / length
        side = -1.0 if (vx * wy - vy * wx) > 0.0 else 1.0
        offset = float(self.get_parameter("avoidance_offset_m").value)
        return [(sx, sy), (ox + side * nx * offset, oy + side * ny * offset), (gx, gy)]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def make_local_path(self, stamp) -> NavPath:
        # 역할: 차체 앞쪽 짧은 local preview를 map frame에 표시한다.
        msg = NavPath()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        x = float(self.odom.pose.pose.position.x)
        y = float(self.odom.pose.pose.position.y)
        yaw = yaw_from_quaternion(
            float(self.odom.pose.pose.orientation.x),
            float(self.odom.pose.pose.orientation.y),
            float(self.odom.pose.pose.orientation.z),
            float(self.odom.pose.pose.orientation.w),
        )
        length = max(float(self.get_parameter("local_path_length_m").value), 0.1)
        count = max(int(self.get_parameter("local_path_points").value), 2)
        for i in range(count):
            d = length * i / float(count - 1)
            msg.poses.append(self.pose_stamped(stamp, x + math.cos(yaw) * d, y + math.sin(yaw) * d, "map"))
        return msg

    def pose_stamped(self, stamp, x: float, y: float, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def load_map(self, map_yaml: str) -> OccupancyGrid | None:
        # 역할: nav2 map_server 없이도 기존 ugv_gazebo 저장 map을 리모콘/RViz에 보여준다.
        if not map_yaml:
            self.get_logger().warn("map_yaml is empty; /map will not be published")
            return None
        yaml_path = Path(os.path.expanduser(map_yaml))
        if not yaml_path.is_file():
            self.get_logger().warn(f"map yaml not found: {yaml_path}")
            return None
        try:
            with yaml_path.open("r", encoding="utf-8") as stream:
                meta = yaml.safe_load(stream)
            image_path = Path(str(meta["image"]))
            if not image_path.is_absolute():
                image_path = yaml_path.parent / image_path
            width, height, pixels = read_pgm(image_path)
            resolution = float(meta.get("resolution", 0.05))
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            occupied_thresh = float(meta.get("occupied_thresh", 0.65))
            free_thresh = float(meta.get("free_thresh", 0.25))
            negate = int(meta.get("negate", 0))
            data: list[int] = []
            # ROS OccupancyGrid의 0번 row는 map 아래쪽이다. PGM row는 위쪽부터라 뒤집어 넣는다.
            for row in range(height - 1, -1, -1):
                start = row * width
                for value in pixels[start:start + width]:
                    color = 255 - value if negate else value
                    occ = (255 - color) / 255.0
                    if occ > occupied_thresh:
                        data.append(100)
                    elif occ < free_thresh:
                        data.append(0)
                    else:
                        data.append(-1)
            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.info.resolution = resolution
            msg.info.width = width
            msg.info.height = height
            msg.info.origin.position.x = float(origin[0])
            msg.info.origin.position.y = float(origin[1])
            msg.info.origin.orientation.w = 1.0
            msg.data = data
            self.get_logger().info(f"Loaded Gazebo map {yaml_path} ({width}x{height}, res={resolution})")
            return msg
        except Exception as exc:
            self.get_logger().warn(f"failed to load map yaml {yaml_path}: {exc}")
            return None


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    # 역할: 의존성을 늘리지 않고 P5/P2 PGM map 파일만 읽는다.
    raw = path.read_bytes()
    tokens: list[bytes] = []
    i = 0
    while len(tokens) < 4 and i < len(raw):
        while i < len(raw) and raw[i] in b" \t\r\n":
            i += 1
        if i < len(raw) and raw[i] == ord("#"):
            while i < len(raw) and raw[i] not in b"\r\n":
                i += 1
            continue
        start = i
        while i < len(raw) and raw[i] not in b" \t\r\n":
            i += 1
        if start < i:
            tokens.append(raw[start:i])
    if len(tokens) < 4:
        raise ValueError(f"invalid PGM header: {path}")
    magic = tokens[0]
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    while i < len(raw) and raw[i] in b" \t\r\n":
        i += 1
    if magic == b"P5":
        pixel_count = width * height
        payload = raw[i:i + pixel_count]
        if len(payload) != pixel_count:
            raise ValueError(f"PGM payload size mismatch: {path}")
        if max_value <= 0:
            raise ValueError(f"invalid PGM max value: {max_value}")
        if max_value == 255:
            return width, height, list(payload)
        return width, height, [int(v * 255 / max_value) for v in payload]
    if magic == b"P2":
        values = [int(v) for v in raw[i:].split()]
        if len(values) < width * height:
            raise ValueError(f"PGM payload size mismatch: {path}")
        return width, height, [int(v * 255 / max_value) for v in values[: width * height]]
    raise ValueError(f"unsupported PGM format {magic!r}: {path}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboMapPathVisualizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        shutdown_text = str(exc)
        shutdown_race = (
            "context is not valid" in shutdown_text
            or "destruction was requested" in shutdown_text
            or "Unable to convert call argument to Python object" in shutdown_text
        )
        if rclpy.ok() and not shutdown_race:
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()

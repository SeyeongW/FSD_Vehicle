from __future__ import annotations

import math
import os
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from waver_patrol.autonomy_common import yaw_from_quaternion
from waver_patrol.test_nodes.gazebo_map_path_visualizer_node import read_pgm


class GazeboLiveMappingNode(Node):
    """Gazebo-only live mapping preview and map apply helper.

    역할:
      - `ugv_world.world` 기반 Gazebo 검증에서 `/map`이 실제 SLAM처럼 변하는지 UI를 시험한다.
      - 기존 `ugv_gazebo/maps/map.yaml`을 ground-truth map으로 읽고, 시간에 따라 알려진 영역을 늘린다.
      - `SAVE_MAP` 명령을 받거나 reveal이 끝나면 `~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml`로 저장한다.
      - 저장 후 같은 `/map`에 full map을 계속 publish하여 리모콘 UI가 즉시 새 map을 보게 한다.

    주의:
      - 실제 SLAM 알고리즘이 아니다. 실차/현장 map 작성은 `ugv_slam`의 Cartographer/Gmapping을 쓴다.
      - 이 노드는 UI, 저장/적용 workflow, topic contract 검증 전용이다.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_live_mapping_node")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("local_path_topic", "/local_plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("mission_command_topic", "/waver/mission_command")
        self.declare_parameter("mapping_state_topic", "/waver/mapping_state")
        self.declare_parameter("map_apply_state_topic", "/waver/map_apply_state")
        self.declare_parameter("map_saved_path_topic", "/waver/map_saved_path")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("reveal_duration_sec", 12.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_save_on_complete", False)
        self.declare_parameter("auto_apply_on_save", False)
        self.declare_parameter("save_dir", "~/ros2_ws/FSD_Vehicle/maps")
        self.declare_parameter("save_basename", "waver_latest_map")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, str(self.get_parameter("map_topic").value), qos)
        self.global_path_pub = self.create_publisher(NavPath, str(self.get_parameter("global_path_topic").value), 10)
        self.local_path_pub = self.create_publisher(NavPath, str(self.get_parameter("local_path_topic").value), 10)
        self.mapping_state_pub = self.create_publisher(String, str(self.get_parameter("mapping_state_topic").value), 10)
        self.apply_state_pub = self.create_publisher(String, str(self.get_parameter("map_apply_state_topic").value), 10)
        self.saved_path_pub = self.create_publisher(String, str(self.get_parameter("map_saved_path_topic").value), 10)

        self.full_map = self.load_map(str(self.get_parameter("map_yaml").value))
        self.current_map: OccupancyGrid | None = None
        self.odom: Odometry | None = None
        self.mapping_active = bool(self.get_parameter("auto_start").value)
        self.mapping_start_time = self._now()
        self.applied_full_map = False
        self.saved_once = False
        self.last_progress = -1.0

        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("mission_command_topic").value),
            self.command_callback,
            10,
        )
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.2)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn("GazeboLiveMappingNode is simulation-only; use ugv_slam for real mapping")

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "START_MAPPING":
            self.mapping_active = True
            self.mapping_start_time = self._now()
            self.applied_full_map = False
            self.saved_once = False
            self.apply_state_pub.publish(String(data="MAPPING_STARTED"))
        elif command == "SAVE_MAP":
            self.save_current_map(apply_after=False)
        elif command in {"LOAD_MAP", "APPLY_MAP", "APPLY_FIXED_MAP", "START_LOCALIZATION"}:
            self.apply_full_map("MAP_FIXED_READY_BY_OPERATOR")

    def tick(self) -> None:
        if self.full_map is None:
            self.mapping_state_pub.publish(String(data="NO_SOURCE_MAP"))
            return
        now = self.get_clock().now().to_msg()
        progress = self.mapping_progress()
        if self.applied_full_map:
            msg = self.clone_map(self.full_map)
        else:
            msg = self.revealed_map(progress)
        msg.header.stamp = now
        self.current_map = msg
        self.map_pub.publish(msg)
        self.publish_paths(now)

        known = sum(1 for value in msg.data if value >= 0)
        total = max(len(msg.data), 1)
        self.mapping_state_pub.publish(
            String(
                data=(
                    f"{'SLAM_LIVE' if not self.applied_full_map else 'MAP_FIXED'} "
                    f"progress={progress:.3f} known_ratio={known / total:.3f} "
                    f"saved={self.saved_once}"
                )
            )
        )
        if progress >= 1.0 and not self.saved_once and bool(self.get_parameter("auto_save_on_complete").value):
            self.save_current_map(apply_after=bool(self.get_parameter("auto_apply_on_save").value))

    def mapping_progress(self) -> float:
        if not self.mapping_active:
            return 0.0
        duration = max(float(self.get_parameter("reveal_duration_sec").value), 1.0)
        return min(max((self._now() - self.mapping_start_time) / duration, 0.0), 1.0)

    def revealed_map(self, progress: float) -> OccupancyGrid:
        assert self.full_map is not None
        msg = self.clone_map(self.full_map)
        width = msg.info.width
        height = msg.info.height
        reveal_columns = int(width * max(0.03, progress))
        data = list(msg.data)
        for y in range(height):
            row = y * width
            for x in range(width):
                if x > reveal_columns:
                    data[row + x] = -1
        msg.data = data
        return msg

    def apply_full_map(self, state: str) -> None:
        self.applied_full_map = True
        self.mapping_active = False
        self.apply_state_pub.publish(String(data=state))

    def save_current_map(self, *, apply_after: bool) -> None:
        if self.current_map is None and self.full_map is None:
            self.apply_state_pub.publish(String(data="SAVE_MAP_FAILED no_map"))
            return
        msg = self.full_map if apply_after and self.full_map is not None else self.current_map
        if msg is None:
            self.apply_state_pub.publish(String(data="SAVE_MAP_FAILED no_current_map"))
            return
        save_dir = Path(os.path.expanduser(str(self.get_parameter("save_dir").value)))
        save_dir.mkdir(parents=True, exist_ok=True)
        base = str(self.get_parameter("save_basename").value).strip() or "waver_latest_map"
        pgm_path = save_dir / f"{base}.pgm"
        yaml_path = save_dir / f"{base}.yaml"
        self.write_pgm(msg, pgm_path)
        metadata = {
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": float(msg.info.resolution),
            "origin": [
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                0.0,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        yaml_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
        self.saved_once = True
        self.saved_path_pub.publish(String(data=str(yaml_path)))
        self.apply_state_pub.publish(String(data=f"MAP_SAVED path={yaml_path}"))
        if apply_after:
            self.apply_full_map(f"MAP_APPLIED path={yaml_path}")

    def publish_paths(self, stamp) -> None:
        if self.odom is None:
            return
        x = float(self.odom.pose.pose.position.x)
        y = float(self.odom.pose.pose.position.y)
        yaw = yaw_from_quaternion(
            float(self.odom.pose.pose.orientation.x),
            float(self.odom.pose.pose.orientation.y),
            float(self.odom.pose.pose.orientation.z),
            float(self.odom.pose.pose.orientation.w),
        )
        global_path = NavPath()
        global_path.header.stamp = stamp
        global_path.header.frame_id = "map"
        local_path = NavPath()
        local_path.header.stamp = stamp
        local_path.header.frame_id = "map"
        for i in range(24):
            d = 2.0 * i / 23.0
            global_path.poses.append(self.pose(stamp, x + math.cos(yaw) * d, y + math.sin(yaw) * d))
        for i in range(10):
            d = 0.8 * i / 9.0
            local_path.poses.append(self.pose(stamp, x + math.cos(yaw) * d, y + math.sin(yaw) * d))
        self.global_path_pub.publish(global_path)
        self.local_path_pub.publish(local_path)

    def pose(self, stamp, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        return msg

    def load_map(self, map_yaml: str) -> OccupancyGrid | None:
        yaml_path = Path(os.path.expanduser(map_yaml))
        if not yaml_path.is_file():
            self.get_logger().warn(f"source map yaml not found: {yaml_path}")
            return None
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
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
        return msg

    def clone_map(self, source: OccupancyGrid) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = source.header.frame_id or "map"
        msg.info = source.info
        msg.data = list(source.data)
        return msg

    def write_pgm(self, msg: OccupancyGrid, path: Path) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        lines = [f"P5\n# Waver Gazebo mapping preview\n{width} {height}\n255\n".encode()]
        pixels = bytearray()
        data = list(msg.data)
        for row in range(height - 1, -1, -1):
            start = row * width
            for value in data[start:start + width]:
                if value < 0:
                    pixels.append(205)
                elif value >= 50:
                    pixels.append(0)
                else:
                    pixels.append(254)
        lines.append(bytes(pixels))
        path.write_bytes(b"".join(lines))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboLiveMappingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()

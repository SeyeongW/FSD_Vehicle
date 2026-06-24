from __future__ import annotations

import math
import os
import shutil
import subprocess
import threading
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


class MappingWorkflowManagerNode(Node):
    """Small state bridge for the operator-panel mapping workflow.

    The UI remains a ROS command publisher. This node turns START_MAPPING,
    SAVE_MAP, and APPLY_FIXED_MAP/START_LOCALIZATION into map-state topics and
    an explicit map save action. It does not drive the robot directly.
    """

    def __init__(self) -> None:
        super().__init__("mapping_workflow_manager_node")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mapping_command_topic", "/waver/mapping_command")
        self.declare_parameter("mission_command_topic", "/waver/mission_command")
        self.declare_parameter("operator_command_topic", "/waver/operator_command")
        self.declare_parameter("save_dir", "~/ugv_ws/FSD_Vehicle/maps")
        self.declare_parameter("save_basename", "waver_latest_map")
        self.declare_parameter("map_saver_timeout_sec", 30.0)
        self.declare_parameter("use_internal_map_writer", True)
        self.declare_parameter("crop_unknown_border_on_save", True)
        self.declare_parameter("crop_margin_cells", 20)
        self.declare_parameter("save_fixed_extent_m", 0.0)
        self.declare_parameter("save_fixed_center_x", 0.0)
        self.declare_parameter("save_fixed_center_y", 0.0)
        self.declare_parameter("known_ratio_min_for_save", 0.01)
        self.declare_parameter("fixed_map_topic", "/map_fixed")
        self.declare_parameter("fixed_map_publish_period_sec", 1.0)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mapping_state_pub = self.create_publisher(String, "/waver/mapping_state", state_qos)
        self.map_apply_state_pub = self.create_publisher(String, "/waver/map_apply_state", state_qos)
        self.map_saved_path_pub = self.create_publisher(String, "/waver/map_saved_path", state_qos)
        self.current_map_source_pub = self.create_publisher(String, "/waver/current_map_source", state_qos)
        self.slam_map_received_pub = self.create_publisher(Bool, "/waver/slam_map_received", 10)
        self.ui_map_reset_pub = self.create_publisher(Bool, "/waver/ui_map_reset", 10)
        self.mapping_active_pub = self.create_publisher(Bool, "/waver/mapping_active", 10)
        self.mapping_progress_pub = self.create_publisher(Float32, "/waver/mapping_progress", 10)
        self.fault_pub = self.create_publisher(String, "/waver/mapping_fault_reason", 10)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.fixed_map_qos = map_qos
        self.fixed_map_pub = None

        self.last_map: OccupancyGrid | None = None
        self.fixed_map: OccupancyGrid | None = None
        self.last_map_time = 0.0
        self.last_fixed_map_pub_time = 0.0
        self.saved_yaml = ""
        self.mapping_active = False
        self.saving = False
        self.apply_requested_after_save = False
        self.last_command = ""
        self.last_command_time = 0.0

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mapping_command_topic").value),
            self.command_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_command_topic").value),
            self.command_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operator_command_topic").value),
            self.command_callback,
            10,
        )
        self.create_timer(0.5, self.tick)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.last_map = msg
        self.last_map_time = self._now()
        known_ratio = self.known_ratio(msg)
        self.mapping_progress_pub.publish(Float32(data=float(known_ratio)))
        if self.mapping_active:
            self.slam_map_received_pub.publish(Bool(data=True))
            self.mapping_state_pub.publish(String(data=f"SLAM_LIVE known_ratio={known_ratio:.3f}"))

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        now = self._now()
        if command == self.last_command and now - self.last_command_time < 0.25:
            return
        self.last_command = command
        self.last_command_time = now
        self.get_logger().info(f"mapping workflow command: {command}")
        if command == "START_MAPPING":
            self.mapping_active = True
            self.last_map = None
            self.fixed_map = None
            self.mapping_active_pub.publish(Bool(data=True))
            self.slam_map_received_pub.publish(Bool(data=False))
            self.ui_map_reset_pub.publish(Bool(data=True))
            self.current_map_source_pub.publish(String(data="SLAM_LIVE_MAP"))
            self.map_apply_state_pub.publish(String(data="OLD_MAP_UNAPPLIED_MAPPING_STARTED"))
            self.mapping_state_pub.publish(String(data="MAPPING_ACTIVE waiting_for_live_map"))
        elif command == "SAVE_MAP":
            self.start_save_thread()
        elif command in {"APPLY_FIXED_MAP", "APPLY_MAP", "LOAD_MAP", "START_LOCALIZATION"}:
            if self.saving:
                self.apply_requested_after_save = True
                self.map_apply_state_pub.publish(String(data="MAP_FIXED_PENDING_SAVE"))
                self.mapping_state_pub.publish(String(data="APPLY_FIXED_MAP queued until SAVE_MAP finishes"))
                return
            self.apply_fixed_map()
        elif command in {"STOP_MAPPING", "STOP", "EMERGENCY_STOP"}:
            self.mapping_active = False
            self.mapping_active_pub.publish(Bool(data=False))
            self.mapping_state_pub.publish(String(data=f"{command} accepted"))
            if command == "STOP_MAPPING":
                self.current_map_source_pub.publish(String(data="MAP_SAVED_NOT_APPLIED" if self.saved_yaml else "NONE"))
                self.map_apply_state_pub.publish(String(data="MAPPING_STOPPED_SAVE_OR_APPLY_REQUIRED"))
                self.mapping_state_pub.publish(String(data="MAPPING_STOPPED"))

    def start_save_thread(self) -> None:
        if self.saving:
            self.mapping_state_pub.publish(String(data="SAVE_MAP ignored; already saving"))
            return
        thread = threading.Thread(target=self.save_map, daemon=True)
        thread.start()

    def save_map(self) -> None:
        self.saving = True
        try:
            if self.last_map is None:
                self.map_apply_state_pub.publish(String(data="SAVE_MAP_FAILED no_live_map"))
                self.fault_pub.publish(String(data="no_live_map"))
                return
            known_ratio = self.known_ratio(self.last_map)
            if known_ratio < float(self.get_parameter("known_ratio_min_for_save").value):
                self.map_apply_state_pub.publish(
                    String(data=f"SAVE_MAP_FAILED known_ratio_low {known_ratio:.3f}")
                )
                self.fault_pub.publish(String(data="known_ratio_low"))
                return
            save_dir = Path(os.path.expanduser(str(self.get_parameter("save_dir").value)))
            save_basename = str(self.get_parameter("save_basename").value)
            save_dir.mkdir(parents=True, exist_ok=True)
            archive_dir = save_dir / "archive"
            archive_dir.mkdir(parents=True, exist_ok=True)
            target_base = save_dir / save_basename
            old_yaml = target_base.with_suffix(".yaml")
            old_pgm = target_base.with_suffix(".pgm")
            if old_yaml.exists():
                stamp = self.get_clock().now().to_msg()
                suffix = f"{stamp.sec}_{stamp.nanosec:09d}"
                shutil.copy2(old_yaml, archive_dir / f"{save_basename}_{suffix}.yaml")
                if old_pgm.exists():
                    shutil.copy2(old_pgm, archive_dir / f"{save_basename}_{suffix}.pgm")
            if bool(self.get_parameter("use_internal_map_writer").value):
                self.mapping_state_pub.publish(String(data=f"SAVE_MAP writing internal {target_base}"))
                self.write_saved_map(self.last_map, target_base)
            else:
                timeout = float(self.get_parameter("map_saver_timeout_sec").value)
                command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(target_base)]
                self.mapping_state_pub.publish(String(data=f"SAVE_MAP running {' '.join(command)}"))
                result = subprocess.run(command, check=False, timeout=timeout, capture_output=True, text=True)
                if result.returncode != 0:
                    detail = (result.stderr or result.stdout or "").strip()[-240:]
                    self.get_logger().error(f"SAVE_MAP_FAILED returncode={result.returncode}: {detail}")
                    self.map_apply_state_pub.publish(String(data=f"SAVE_MAP_FAILED returncode={result.returncode}"))
                    self.fault_pub.publish(String(data=f"map_saver_failed {detail}"))
                    return
            self.saved_yaml = str(old_yaml)
            self.map_saved_path_pub.publish(String(data=self.saved_yaml))
            self.current_map_source_pub.publish(String(data="SAVED_SLAM_MAP_NOT_APPLIED"))
            self.map_apply_state_pub.publish(String(data=f"MAP_SAVED_NOT_APPLIED {self.saved_yaml}"))
            self.mapping_state_pub.publish(String(data=f"SAVE_MAP_OK {self.saved_yaml}"))
            self.get_logger().info(f"SAVE_MAP_OK {self.saved_yaml}")
            if self.apply_requested_after_save:
                self.apply_requested_after_save = False
                self.apply_fixed_map()
        except Exception as exc:  # noqa: BLE001
            self.apply_requested_after_save = False
            self.map_apply_state_pub.publish(String(data="SAVE_MAP_FAILED exception"))
            self.fault_pub.publish(String(data=f"{type(exc).__name__}: {exc}"))
        finally:
            self.saving = False

    def apply_fixed_map(self) -> None:
        if not self.saved_yaml:
            default_yaml = Path(os.path.expanduser(str(self.get_parameter("save_dir").value))) / (
                str(self.get_parameter("save_basename").value) + ".yaml"
            )
            self.saved_yaml = str(default_yaml) if default_yaml.exists() else ""
        if not self.saved_yaml or not Path(self.saved_yaml).exists():
            self.map_apply_state_pub.publish(String(data="MAP_APPLY_FAILED no_saved_map"))
            self.fault_pub.publish(String(data="no_saved_map"))
            return
        try:
            fixed_map = self.load_saved_map(self.saved_yaml)
        except Exception as exc:  # noqa: BLE001
            self.map_apply_state_pub.publish(String(data="MAP_APPLY_FAILED load_error"))
            self.fault_pub.publish(String(data=f"map_load_failed {type(exc).__name__}: {exc}"))
            return
        self.mapping_active = False
        self.fixed_map = fixed_map
        if self.fixed_map_pub is None:
            self.fixed_map_pub = self.create_publisher(
                OccupancyGrid,
                str(self.get_parameter("fixed_map_topic").value),
                self.fixed_map_qos,
            )
        self.mapping_active_pub.publish(Bool(data=False))
        self.map_saved_path_pub.publish(String(data=self.saved_yaml))
        self.fixed_map_pub.publish(fixed_map)
        self.last_fixed_map_pub_time = self._now()
        self.current_map_source_pub.publish(String(data="FIXED_MAP_READY"))
        self.map_apply_state_pub.publish(String(data=f"MAP_FIXED_READY {self.saved_yaml}"))
        self.mapping_state_pub.publish(String(data="MAP_FIXED_READY"))
        self.get_logger().info(f"MAP_FIXED_READY {self.saved_yaml}")

    def tick(self) -> None:
        self.mapping_active_pub.publish(Bool(data=self.mapping_active))
        if self.last_map is None:
            self.mapping_progress_pub.publish(Float32(data=0.0))
        if not self.mapping_active and self.fixed_map is not None:
            period = float(self.get_parameter("fixed_map_publish_period_sec").value)
            now = self._now()
            if now - self.last_fixed_map_pub_time >= max(0.2, period):
                self.fixed_map.header.stamp = self.get_clock().now().to_msg()
                if self.fixed_map_pub is not None:
                    self.fixed_map_pub.publish(self.fixed_map)
                self.last_fixed_map_pub_time = now

    @staticmethod
    def known_ratio(msg: OccupancyGrid) -> float:
        total = len(msg.data)
        if total == 0:
            return 0.0
        known = sum(1 for value in msg.data if int(value) >= 0)
        return known / float(total)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def load_saved_map(self, yaml_path: str) -> OccupancyGrid:
        """Load a nav2_map_server YAML/PGM map and publish it as OccupancyGrid."""
        path = Path(os.path.expanduser(yaml_path)).resolve()
        with path.open("r", encoding="utf-8") as stream:
            meta = yaml.safe_load(stream) or {}
        image_name = str(meta.get("image", "")).strip()
        if not image_name:
            raise ValueError(f"map yaml has no image field: {path}")
        image_path = Path(image_name)
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        width, height, pixels = self.read_pgm(image_path)
        resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin, list) or len(origin) < 3:
            origin = [0.0, 0.0, 0.0]
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.25))

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = 0.0
        yaw = float(origin[2])
        grid.info.origin.orientation.z = math.sin(yaw * 0.5)
        grid.info.origin.orientation.w = math.cos(yaw * 0.5)
        data: list[int] = []
        # PGM files are top-left origin, OccupancyGrid is bottom-left origin.
        for row in range(height - 1, -1, -1):
            start = row * width
            for value in pixels[start : start + width]:
                color = 255 - int(value) if negate else int(value)
                occ = (255 - color) / 255.0
                if occ > occupied_thresh:
                    data.append(100)
                elif occ < free_thresh:
                    data.append(0)
                else:
                    data.append(-1)
        grid.data = data
        return grid

    def write_saved_map(self, msg: OccupancyGrid, target_base: Path) -> None:
        yaml_path = target_base.with_suffix(".yaml")
        pgm_path = target_base.with_suffix(".pgm")
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            raise RuntimeError("invalid occupancy grid dimensions")
        resolution = float(msg.info.resolution)
        yaw = self.quaternion_yaw(msg.info.origin.orientation)
        fixed_extent_m = float(self.get_parameter("save_fixed_extent_m").value)
        fixed_center_x = float(self.get_parameter("save_fixed_center_x").value)
        fixed_center_y = float(self.get_parameter("save_fixed_center_y").value)

        if fixed_extent_m > 0.0:
            if abs(yaw) > 1e-3:
                raise RuntimeError("fixed-extent map save requires yaw-aligned map origin")
            output_width = max(1, int(math.ceil(fixed_extent_m / resolution)))
            output_height = output_width
            output_origin_x = fixed_center_x - 0.5 * output_width * resolution
            output_origin_y = fixed_center_y - 0.5 * output_height * resolution
            source_origin_x = float(msg.info.origin.position.x)
            source_origin_y = float(msg.info.origin.position.y)

            with pgm_path.open("wb") as stream:
                stream.write(
                    f"P5\n# CREATOR: waver mapping workflow fixed extent\n{output_width} {output_height}\n255\n".encode(
                        "ascii"
                    )
                )
                for out_y in range(output_height - 1, -1, -1):
                    row = bytearray()
                    world_y = output_origin_y + (out_y + 0.5) * resolution
                    src_y = int(math.floor((world_y - source_origin_y) / resolution))
                    for out_x in range(output_width):
                        world_x = output_origin_x + (out_x + 0.5) * resolution
                        src_x = int(math.floor((world_x - source_origin_x) / resolution))
                        if 0 <= src_x < width and 0 <= src_y < height:
                            occupancy = int(msg.data[src_y * width + src_x])
                        else:
                            occupancy = -1
                        if occupancy < 0:
                            # Keep unknown inside the free/occupied thresholds.
                            # 205 is interpreted as free by this YAML's
                            # free_thresh=0.25 path, so use mid-gray.
                            pixel = 127
                        elif occupancy >= 65:
                            pixel = 0
                        else:
                            pixel = 254
                        row.append(pixel)
                    stream.write(row)
            payload = {
                "image": pgm_path.name,
                "mode": "trinary",
                "resolution": resolution,
                "origin": [
                    output_origin_x,
                    output_origin_y,
                    0.0,
                ],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
            yaml_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
            return

        min_x, min_y, max_x, max_y = 0, 0, width - 1, height - 1
        if bool(self.get_parameter("crop_unknown_border_on_save").value):
            known_indices = [i for i, value in enumerate(msg.data) if int(value) >= 0]
            if known_indices:
                xs = [i % width for i in known_indices]
                ys = [i // width for i in known_indices]
                margin = max(0, int(self.get_parameter("crop_margin_cells").value))
                min_x = max(min(xs) - margin, 0)
                max_x = min(max(xs) + margin, width - 1)
                min_y = max(min(ys) - margin, 0)
                max_y = min(max(ys) + margin, height - 1)
        cropped_width = max_x - min_x + 1
        cropped_height = max_y - min_y + 1
        with pgm_path.open("wb") as stream:
            stream.write(
                f"P5\n# CREATOR: waver mapping workflow\n{cropped_width} {cropped_height}\n255\n".encode(
                    "ascii"
                )
            )
            for y in range(max_y, min_y - 1, -1):
                row = bytearray()
                offset = y * width
                for value in msg.data[offset + min_x : offset + max_x + 1]:
                    occupancy = int(value)
                    if occupancy < 0:
                        # Keep unknown as unknown when loaded by map_server or
                        # by this node's load_saved_map().
                        pixel = 127
                    elif occupancy >= 65:
                        pixel = 0
                    else:
                        pixel = 254
                    row.append(pixel)
                stream.write(row)
        dx = float(min_x) * resolution
        dy = float(min_y) * resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        origin_x = float(msg.info.origin.position.x) + cos_yaw * dx - sin_yaw * dy
        origin_y = float(msg.info.origin.position.y) + sin_yaw * dx + cos_yaw * dy
        payload = {
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": resolution,
            "origin": [
                origin_x,
                origin_y,
                float(yaw),
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        yaml_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    @staticmethod
    def quaternion_yaw(q) -> float:
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def read_pgm(path: Path) -> tuple[int, int, list[int]]:
        with path.open("rb") as stream:
            magic = stream.readline().strip()
            if magic not in {b"P5", b"P2"}:
                raise ValueError(f"unsupported map image format {magic!r}; expected PGM P5/P2")

            def next_token() -> bytes:
                while True:
                    token = stream.readline()
                    if not token:
                        raise ValueError("unexpected EOF in PGM header")
                    token = token.strip()
                    if token.startswith(b"#") or not token:
                        continue
                    return token

            dims = next_token().split()
            while len(dims) < 2:
                dims.extend(next_token().split())
            width = int(dims[0])
            height = int(dims[1])
            max_value = int(next_token())
            if max_value <= 0:
                raise ValueError("invalid PGM max value")
            if magic == b"P5":
                raw = stream.read(width * height)
                if len(raw) < width * height:
                    raise ValueError("truncated PGM data")
                pixels = list(raw[: width * height])
            else:
                text = stream.read().split()
                if len(text) < width * height:
                    raise ValueError("truncated PGM data")
                pixels = [int(value) for value in text[: width * height]]
            if max_value != 255:
                pixels = [round((value / max_value) * 255.0) for value in pixels]
            return width, height, pixels


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingWorkflowManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()

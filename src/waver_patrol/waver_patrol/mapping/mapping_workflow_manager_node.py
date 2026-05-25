from __future__ import annotations

import os
import shutil
import subprocess
import threading
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
        self.declare_parameter("mission_command_topic", "/waver/mission_command")
        self.declare_parameter("save_dir", "~/ros2_ws/maps")
        self.declare_parameter("save_basename", "waver_latest_map")
        self.declare_parameter("map_saver_timeout_sec", 30.0)
        self.declare_parameter("known_ratio_min_for_save", 0.01)

        self.mapping_state_pub = self.create_publisher(String, "/waver/mapping_state", 10)
        self.map_apply_state_pub = self.create_publisher(String, "/waver/map_apply_state", 10)
        self.map_saved_path_pub = self.create_publisher(String, "/waver/map_saved_path", 10)
        self.mapping_active_pub = self.create_publisher(Bool, "/waver/mapping_active", 10)
        self.mapping_progress_pub = self.create_publisher(Float32, "/waver/mapping_progress", 10)
        self.mode_pub = self.create_publisher(String, "/waver/mode", 10)
        self.fault_pub = self.create_publisher(String, "/waver/mapping_fault_reason", 10)

        self.last_map: OccupancyGrid | None = None
        self.last_map_time = 0.0
        self.saved_yaml = ""
        self.mapping_active = False
        self.saving = False

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_command_topic").value),
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
            self.mapping_state_pub.publish(String(data=f"SLAM_LIVE known_ratio={known_ratio:.3f}"))

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "START_MAPPING":
            self.mapping_active = True
            self.mode_pub.publish(String(data="MAPPING_AUTO"))
            self.mapping_active_pub.publish(Bool(data=True))
            self.map_apply_state_pub.publish(String(data="OLD_MAP_UNAPPLIED_MAPPING_STARTED"))
            self.mapping_state_pub.publish(String(data="START_MAPPING accepted; waiting for live /map"))
        elif command == "SAVE_MAP":
            self.start_save_thread()
        elif command in {"APPLY_FIXED_MAP", "APPLY_MAP", "LOAD_MAP", "START_LOCALIZATION"}:
            self.apply_fixed_map()
        elif command in {"STOP_MAPPING", "STOP", "EMERGENCY_STOP"}:
            self.mapping_active = False
            self.mapping_active_pub.publish(Bool(data=False))
            self.mapping_state_pub.publish(String(data=f"{command} accepted"))

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
            timeout = float(self.get_parameter("map_saver_timeout_sec").value)
            command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(target_base)]
            self.mapping_state_pub.publish(String(data=f"SAVE_MAP running {' '.join(command)}"))
            result = subprocess.run(command, check=False, timeout=timeout, capture_output=True, text=True)
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or "").strip()[-180:]
                self.map_apply_state_pub.publish(String(data=f"SAVE_MAP_FAILED returncode={result.returncode}"))
                self.fault_pub.publish(String(data=f"map_saver_failed {detail}"))
                return
            self.saved_yaml = str(old_yaml)
            self.map_saved_path_pub.publish(String(data=self.saved_yaml))
            self.map_apply_state_pub.publish(String(data=f"MAP_SAVED_NOT_APPLIED {self.saved_yaml}"))
            self.mapping_state_pub.publish(String(data=f"SAVE_MAP_OK {self.saved_yaml}"))
        except Exception as exc:  # noqa: BLE001
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
        self.mapping_active = False
        self.mapping_active_pub.publish(Bool(data=False))
        self.map_saved_path_pub.publish(String(data=self.saved_yaml))
        self.map_apply_state_pub.publish(String(data=f"MAP_FIXED_READY {self.saved_yaml}"))
        self.mapping_state_pub.publish(String(data="MAP_FIXED_READY"))
        self.mode_pub.publish(String(data="STANDBY"))

    def tick(self) -> None:
        self.mapping_active_pub.publish(Bool(data=self.mapping_active))
        if self.last_map is None:
            self.mapping_progress_pub.publish(Float32(data=0.0))

    @staticmethod
    def known_ratio(msg: OccupancyGrid) -> float:
        total = len(msg.data)
        if total == 0:
            return 0.0
        known = sum(1 for value in msg.data if int(value) >= 0)
        return known / float(total)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingWorkflowManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

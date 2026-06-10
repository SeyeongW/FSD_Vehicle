from __future__ import annotations

import os
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


class StaticMapStatePublisherNode(Node):
    """Publish operator-panel map state for a Nav2 map_server-backed map.

    Nav2's map_server remains the single /map authority.  This node only
    advertises the UI state that says the current map is a saved/static map,
    using transient-local QoS so a remote panel launched later immediately
    receives the map source and apply status.
    """

    def __init__(self) -> None:
        super().__init__("static_map_state_publisher_node")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_path", "")
        self.declare_parameter("current_map_source_topic", "/waver/current_map_source")
        self.declare_parameter("map_apply_state_topic", "/waver/map_apply_state")
        self.declare_parameter("map_saved_path_topic", "/waver/map_saved_path")
        self.declare_parameter("mapping_state_topic", "/waver/mapping_state")
        self.declare_parameter("mapping_active_topic", "/waver/mapping_active")
        self.declare_parameter("slam_map_received_topic", "/waver/slam_map_received")
        self.declare_parameter("mapping_progress_topic", "/waver/mapping_progress")
        self.declare_parameter("publish_period_sec", 0.5)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.current_source_pub = self.create_publisher(
            String,
            str(self.get_parameter("current_map_source_topic").value),
            state_qos,
        )
        self.apply_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("map_apply_state_topic").value),
            state_qos,
        )
        self.saved_path_pub = self.create_publisher(
            String,
            str(self.get_parameter("map_saved_path_topic").value),
            state_qos,
        )
        self.mapping_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("mapping_state_topic").value),
            state_qos,
        )
        self.mapping_active_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("mapping_active_topic").value),
            state_qos,
        )
        self.slam_received_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("slam_map_received_topic").value),
            state_qos,
        )
        self.progress_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("mapping_progress_topic").value),
            state_qos,
        )

        self.map_received = False
        self.last_known_ratio = 0.0
        self.map_width = 0
        self.map_height = 0
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            map_qos,
        )
        period = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self.publish_state)
        self.publish_state()

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_received = int(msg.info.width) > 0 and int(msg.info.height) > 0
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.last_known_ratio = self.known_ratio(msg)
        if self.map_received:
            self.get_logger().info(
                "static map received: "
                f"{self.map_width}x{self.map_height} known_ratio={self.last_known_ratio:.3f}",
                throttle_duration_sec=5.0,
            )
        self.publish_state()

    def publish_state(self) -> None:
        map_path = self.map_path()
        self.mapping_active_pub.publish(Bool(data=False))
        self.slam_received_pub.publish(Bool(data=False))
        self.progress_pub.publish(Float32(data=float(self.last_known_ratio)))
        self.saved_path_pub.publish(String(data=map_path))

        if self.map_received:
            self.current_source_pub.publish(String(data="STATIC_MAP"))
            self.apply_state_pub.publish(String(data=f"MAP_FIXED_READY {map_path}".strip()))
            self.mapping_state_pub.publish(
                String(
                    data=(
                        "STATIC_MAP_READY "
                        f"width={self.map_width} height={self.map_height} "
                        f"known_ratio={self.last_known_ratio:.3f}"
                    )
                )
            )
            return

        if map_path and Path(map_path).expanduser().exists():
            self.current_source_pub.publish(String(data="STATIC_MAP_WAITING_FOR_MAP_SERVER"))
            self.apply_state_pub.publish(String(data=f"MAP_FIXED_WAITING_FOR_MAP {map_path}"))
        else:
            self.current_source_pub.publish(String(data="NONE"))
            self.apply_state_pub.publish(String(data=f"MAP_FIXED_MISSING {map_path}".strip()))
        self.mapping_state_pub.publish(String(data="STATIC_MAP_WAITING_FOR_MAP"))

    def map_path(self) -> str:
        value = str(self.get_parameter("map_path").value).strip()
        if not value:
            return ""
        return os.path.expanduser(os.path.expandvars(value))

    @staticmethod
    def known_ratio(msg: OccupancyGrid) -> float:
        total = len(msg.data)
        if total <= 0:
            return 0.0
        known = sum(1 for value in msg.data if int(value) >= 0)
        return known / float(total)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaticMapStatePublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

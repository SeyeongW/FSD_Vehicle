from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class MappingBackendManagerNode(Node):
    """Audit the actual Gazebo SLAM backend used by the operator UI flow."""

    def __init__(self) -> None:
        super().__init__("mapping_backend_manager_node")
        self.declare_parameter("scan_topic", "/scan_slam")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mapping_active_topic", "/waver/mapping_active")
        self.declare_parameter("mapping_state_topic", "/waver/mapping_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_min_hz", 3.0)
        self.declare_parameter("odom_min_hz", 5.0)
        self.declare_parameter("map_min_hz_when_active", 0.15)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("min_scan_finite_ratio", 0.03)

        self.scan_count = 0
        self.odom_count = 0
        self.map_count = 0
        self.scan_window_start = self._now()
        self.odom_window_start = self.scan_window_start
        self.map_window_start = self.scan_window_start
        self.scan_hz = 0.0
        self.odom_hz = 0.0
        self.map_hz = 0.0
        self.scan_finite_ratio = 0.0
        self.last_scan_time = 0.0
        self.last_odom_time = 0.0
        self.last_map_time = 0.0
        self.last_mapping_state = "UNKNOWN"
        self.mapping_active = False

        self.state_pub = self.create_publisher(String, "/waver/mapping_backend_state", 10)
        self.ready_pub = self.create_publisher(Bool, "/waver/mapping_backend_ready", 10)
        self.fault_pub = self.create_publisher(String, "/waver/mapping_backend_fault", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("mapping_active_topic").value),
            lambda msg: setattr(self, "mapping_active", bool(msg.data)),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mapping_state_topic").value),
            lambda msg: setattr(self, "last_mapping_state", msg.data.strip() or "UNKNOWN"),
            10,
        )
        self.create_timer(1.0, self.tick)

    def scan_callback(self, msg: LaserScan) -> None:
        finite = 0
        total = 0
        for raw in msg.ranges:
            total += 1
            value = float(raw)
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                finite += 1
        self.scan_finite_ratio = float(finite) / float(total) if total else 0.0
        self.scan_count += 1
        self.last_scan_time = self._now()

    def odom_callback(self, _msg: Odometry) -> None:
        self.odom_count += 1
        self.last_odom_time = self._now()

    def map_callback(self, _msg: OccupancyGrid) -> None:
        self.map_count += 1
        self.last_map_time = self._now()

    def tick(self) -> None:
        now = self._now()
        self.scan_hz, self.scan_count, self.scan_window_start = self._roll_hz(
            self.scan_count, self.scan_window_start, now
        )
        self.odom_hz, self.odom_count, self.odom_window_start = self._roll_hz(
            self.odom_count, self.odom_window_start, now
        )
        self.map_hz, self.map_count, self.map_window_start = self._roll_hz(
            self.map_count, self.map_window_start, now
        )
        faults: list[str] = []
        stale = float(self.get_parameter("stale_timeout_sec").value)
        if self.last_scan_time <= 0.0 or now - self.last_scan_time > stale:
            faults.append("scan_stale")
        if self.last_odom_time <= 0.0 or now - self.last_odom_time > stale:
            faults.append("odom_stale")
        if self.mapping_active and (self.last_map_time <= 0.0 or now - self.last_map_time > stale * 3.0):
            faults.append("map_stale")
        if self.scan_hz < float(self.get_parameter("scan_min_hz").value):
            faults.append(f"scan_hz_low:{self.scan_hz:.2f}")
        if self.odom_hz < float(self.get_parameter("odom_min_hz").value):
            faults.append(f"odom_hz_low:{self.odom_hz:.2f}")
        if self.mapping_active and self.map_hz < float(self.get_parameter("map_min_hz_when_active").value):
            faults.append(f"map_hz_low:{self.map_hz:.2f}")
        if self.scan_finite_ratio < float(self.get_parameter("min_scan_finite_ratio").value):
            faults.append(f"scan_sparse:{self.scan_finite_ratio:.3f}")
        if not self._tf_ok(str(self.get_parameter("odom_frame").value), str(self.get_parameter("base_frame").value)):
            faults.append("tf_odom_base_missing")
        if self.mapping_active and not self._tf_ok(str(self.get_parameter("map_frame").value), str(self.get_parameter("odom_frame").value)):
            faults.append("tf_map_odom_missing")

        ready = not faults
        state = (
            f"{'READY' if ready else 'BLOCKED'} "
            f"mapping_active={self.mapping_active} "
            f"mapping_state={self.last_mapping_state} "
            f"scan_hz={self.scan_hz:.2f} odom_hz={self.odom_hz:.2f} "
            f"map_hz={self.map_hz:.2f} finite_ratio={self.scan_finite_ratio:.3f}"
        )
        if faults:
            state += " faults=" + ",".join(faults)
            self.fault_pub.publish(String(data=",".join(faults)))
        self.state_pub.publish(String(data=state))
        self.ready_pub.publish(Bool(data=ready))

    @staticmethod
    def _roll_hz(count: int, window_start: float, now: float) -> tuple[float, int, float]:
        elapsed = max(1e-6, now - window_start)
        if elapsed < 1.0:
            return float(count) / elapsed, count, window_start
        return float(count) / elapsed, 0, now

    def _tf_ok(self, target_frame: str, source_frame: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
            return True
        except TransformException:
            return False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def close(self) -> None:
        try:
            self.tf_listener.unregister()
        except Exception:
            pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingBackendManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()

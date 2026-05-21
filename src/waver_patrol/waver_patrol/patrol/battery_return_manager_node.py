from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from waver_patrol.autonomy_common import load_yaml_file, quaternion_from_yaw, yaw_deg_to_rad


@dataclass
class ReturnPose:
    frame_id: str = "odom"
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class BatteryReturnManagerNode(Node):
    """Decide when Waver must return home because battery is low.

    전압 threshold는 실제 배터리 팩을 부하 상태에서 측정해 보정해야 한다.
    percentage와 voltage 중 하나라도 critical이면 복귀가 우선된다.
    """

    def __init__(self) -> None:
        super().__init__("battery_return_manager_node")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("voltage_topic", "/voltage")
        self.declare_parameter("return_home_active_topic", "/waver/return_home_active")
        self.declare_parameter("return_goal_topic", "/waver/return_goal")
        self.declare_parameter("return_home_arrived_topic", "/waver/return_home_arrived")
        self.declare_parameter("mission_reset_topic", "/waver/mission_reset")
        self.declare_parameter("battery_state_text_topic", "/waver/battery_state_text")
        self.declare_parameter("waypoint_file", "waypoints/waver_bird_patrol_demo.yaml")
        self.declare_parameter("prefer_charge_pose", True)
        self.declare_parameter("warning_percentage", 0.30)
        self.declare_parameter("critical_percentage", 0.20)
        self.declare_parameter("resume_percentage", 0.38)
        self.declare_parameter("warning_voltage", 9.8)
        self.declare_parameter("critical_voltage", 9.3)
        self.declare_parameter("resume_voltage", 10.2)
        self.declare_parameter("battery_stale_sec", 5.0)
        self.declare_parameter("stop_on_battery_stale", False)
        self.declare_parameter("home_frame_id", "odom")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("timer_hz", 2.0)

        self.last_percentage: float | None = None
        self.last_voltage: float | None = None
        self.last_percentage_time = 0.0
        self.last_voltage_time = 0.0
        self.return_active = False
        self.hold_at_home = False
        self.return_pose = self._load_return_pose()

        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("return_home_active_topic").value), 10)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("return_goal_topic").value), 10)
        self.text_pub = self.create_publisher(String, str(self.get_parameter("battery_state_text_topic").value), 10)
        self.create_subscription(BatteryState, str(self.get_parameter("battery_state_topic").value), self.battery_callback, 10)
        self.create_subscription(Float32, str(self.get_parameter("voltage_topic").value), self.voltage_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("return_home_arrived_topic").value), self.arrived_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("mission_reset_topic").value), self.reset_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 0.5), self.tick)

    def battery_callback(self, msg: BatteryState) -> None:
        now = self._now()
        if math.isfinite(float(msg.percentage)) and msg.percentage >= 0.0:
            self.last_percentage = float(msg.percentage)
            self.last_percentage_time = now
        if math.isfinite(float(msg.voltage)) and msg.voltage > 0.0:
            self.last_voltage = float(msg.voltage)
            self.last_voltage_time = now

    def voltage_callback(self, msg: Float32) -> None:
        if math.isfinite(float(msg.data)) and msg.data > 0.0:
            self.last_voltage = float(msg.data)
            self.last_voltage_time = self._now()

    def arrived_callback(self, msg: Bool) -> None:
        if msg.data and self.return_active:
            self.hold_at_home = True

    def reset_callback(self, msg: Bool) -> None:
        if msg.data:
            self.hold_at_home = False

    def tick(self) -> None:
        pct_state = self._percentage_state()
        volt_state = self._voltage_state()
        state = self._combined_state(pct_state, volt_state)
        if state in {"CRITICAL_RETURN_HOME", "BATTERY_STALE_STOP"}:
            self.return_active = True
        elif self.hold_at_home:
            self.return_active = True
            state = "HOLD_AT_HOME"
        elif state == "BATTERY_RECOVERED":
            self.return_active = False

        self.active_pub.publish(Bool(data=self.return_active))
        if self.return_active:
            self.goal_pub.publish(self._return_goal())
        self.text_pub.publish(String(data=f"{state} pct={self._fmt(self.last_percentage)} voltage={self._fmt(self.last_voltage)}"))

    def _percentage_state(self) -> str:
        if self.last_percentage is None or self._now() - self.last_percentage_time > float(self.get_parameter("battery_stale_sec").value):
            return "STALE"
        pct = self.last_percentage
        if pct <= float(self.get_parameter("critical_percentage").value):
            return "CRITICAL"
        if pct <= float(self.get_parameter("warning_percentage").value):
            return "WARN"
        if pct >= float(self.get_parameter("resume_percentage").value):
            return "RECOVERED"
        return "OK"

    def _voltage_state(self) -> str:
        if self.last_voltage is None or self._now() - self.last_voltage_time > float(self.get_parameter("battery_stale_sec").value):
            return "STALE"
        voltage = self.last_voltage
        if voltage <= float(self.get_parameter("critical_voltage").value):
            return "CRITICAL"
        if voltage <= float(self.get_parameter("warning_voltage").value):
            return "WARN"
        if voltage >= float(self.get_parameter("resume_voltage").value):
            return "RECOVERED"
        return "OK"

    def _combined_state(self, pct_state: str, volt_state: str) -> str:
        states = {pct_state, volt_state}
        if "CRITICAL" in states:
            return "CRITICAL_RETURN_HOME"
        if states == {"STALE"}:
            return "BATTERY_STALE_STOP" if bool(self.get_parameter("stop_on_battery_stale").value) else "BATTERY_STALE_WARN"
        if "WARN" in states:
            return "LOW_WARN"
        if states <= {"RECOVERED", "STALE"} and not self.hold_at_home:
            return "BATTERY_RECOVERED"
        return "BATTERY_OK"

    def _load_return_pose(self) -> ReturnPose:
        fallback = ReturnPose(
            str(self.get_parameter("home_frame_id").value),
            float(self.get_parameter("home_x").value),
            float(self.get_parameter("home_y").value),
            float(self.get_parameter("home_yaw").value),
        )
        try:
            data = load_yaml_file(str(self.get_parameter("waypoint_file").value))
        except Exception as exc:
            self.get_logger().warn(f"Could not read waypoint_file for return pose; using fallback: {exc}")
            return fallback
        frame = str(data.get("frame_id", fallback.frame_id))
        key = "charge" if bool(self.get_parameter("prefer_charge_pose").value) else "home"
        item = data.get(key) if isinstance(data.get(key), dict) else data.get("home")
        if not isinstance(item, dict):
            return fallback
        yaw = item.get("yaw", None)
        if yaw is None:
            yaw = yaw_deg_to_rad(float(item.get("yaw_deg", 0.0)))
        else:
            yaw = float(yaw)
        return ReturnPose(frame, float(item.get("x", fallback.x)), float(item.get("y", fallback.y)), float(yaw))

    def _return_goal(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.return_pose.frame_id
        msg.pose.position.x = self.return_pose.x
        msg.pose.position.y = self.return_pose.y
        qx, qy, qz, qw = quaternion_from_yaw(self.return_pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    @staticmethod
    def _fmt(value: float | None) -> str:
        return "none" if value is None else f"{value:.3f}"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BatteryReturnManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

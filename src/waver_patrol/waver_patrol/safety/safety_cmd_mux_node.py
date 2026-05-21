from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from waver_patrol.autonomy_common import clamp, make_twist, stop_twist, twist_is_finite


@dataclass
class ScanSectorState:
    front_min: float = math.inf
    rear_min: float = math.inf
    finite_points: int = 0
    last_time: float = 0.0
    adapter_state: str = "UNKNOWN"


class SafetyCmdMuxNode(Node):
    """Final Waver command mux.

    역할:
      - Waver 시스템에서 최종 `/cmd_vel`을 발행하는 유일한 노드다.
      - manual/autonomy 후보 명령을 모드, timeout, E-Stop, LaserScan hard-stop으로 필터링한다.
      - 2D LaserScan은 z값이 없으므로 공중 객체 판별에는 쓰지 않고 지상 장애물 안전정지에만 쓴다.
    """

    def __init__(self) -> None:
        super().__init__("safety_cmd_mux_node")
        self.declare_parameter("nav2_cmd_topic", "/waver/cmd_vel_nav2")
        self.declare_parameter("cmd_vel_auto_topic", "")
        self.declare_parameter("manual_cmd_vel_topic", "/waver/manual_cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("livox_scan_adapter_state_topic", "/waver/livox_scan_adapter_state")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("speed_limit_topic", "/waver/speed_limit")
        self.declare_parameter("angular_speed_limit_topic", "/waver/angular_speed_limit")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("mode_default", "AUTO")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("require_scan", True)
        self.declare_parameter("stop_on_adapter_degraded", True)
        self.declare_parameter("scan_stale_s", 0.5)
        self.declare_parameter("front_sector_deg", 55.0)
        self.declare_parameter("rear_sector_deg", 55.0)
        self.declare_parameter("hard_stop_distance_m", 0.45)
        self.declare_parameter("slow_down_distance_m", 1.2)
        self.declare_parameter("clear_distance_m", 1.8)
        self.declare_parameter("min_valid_scan_points", 20)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("max_linear_delta_per_tick", 0.04)
        self.declare_parameter("max_angular_delta_per_tick", 0.08)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("manual_override_timeout_sec", 0.5)
        self.declare_parameter("stop_on_unknown_state", True)
        self.declare_parameter("timer_hz", 20.0)

        self.mode = str(self.get_parameter("mode_default").value).strip().upper()
        self.estop = False
        self.external_stop = False
        self.speed_limit = float(self.get_parameter("max_linear_speed").value)
        self.angular_speed_limit = float(self.get_parameter("max_angular_speed").value)
        self.auto_cmd = stop_twist()
        self.manual_cmd = stop_twist()
        self.last_auto_time = 0.0
        self.last_manual_time = 0.0
        self.scan = ScanSectorState()
        self.last_out = stop_twist()
        self.last_scan_blocking_state = "SCAN_STALE_STOP"

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_out_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("safety_state_topic").value), 10)
        auto_topic = str(self.get_parameter("cmd_vel_auto_topic").value).strip()
        if not auto_topic:
            auto_topic = str(self.get_parameter("nav2_cmd_topic").value).strip()
        self.create_subscription(Twist, auto_topic, self.auto_callback, 10)
        self.create_subscription(Twist, str(self.get_parameter("manual_cmd_vel_topic").value), self.manual_callback, 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("livox_scan_adapter_state_topic").value), self.adapter_state_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_subscription(Float32, str(self.get_parameter("speed_limit_topic").value), self.speed_limit_callback, 10)
        self.create_subscription(Float32, str(self.get_parameter("angular_speed_limit_topic").value), self.angular_limit_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), self.mode_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def auto_callback(self, msg: Twist) -> None:
        if not twist_is_finite(msg):
            self.auto_cmd = stop_twist()
            self.last_auto_time = self._now()
            return
        self.auto_cmd = msg
        self.last_auto_time = self._now()

    def manual_callback(self, msg: Twist) -> None:
        if not twist_is_finite(msg):
            self.manual_cmd = stop_twist()
            self.last_manual_time = self._now()
            return
        self.manual_cmd = msg
        self.last_manual_time = self._now()

    def speed_limit_callback(self, msg: Float32) -> None:
        self.speed_limit = clamp(float(msg.data), 0.0, float(self.get_parameter("max_linear_speed").value))

    def angular_limit_callback(self, msg: Float32) -> None:
        self.angular_speed_limit = clamp(float(msg.data), 0.0, float(self.get_parameter("max_angular_speed").value))

    def mode_callback(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        self.mode = mode if mode else str(self.get_parameter("mode_default").value).strip().upper()

    def adapter_state_callback(self, msg: String) -> None:
        self.scan.adapter_state = msg.data.strip().upper()

    def scan_callback(self, msg: LaserScan) -> None:
        front_limit = math.radians(float(self.get_parameter("front_sector_deg").value) * 0.5)
        rear_limit = math.radians(float(self.get_parameter("rear_sector_deg").value) * 0.5)
        front_values: list[float] = []
        rear_values: list[float] = []
        finite_count = 0
        for i, raw in enumerate(msg.ranges):
            distance = float(raw)
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            finite_count += 1
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= front_limit:
                front_values.append(distance)
            if abs(abs(angle) - math.pi) <= rear_limit:
                rear_values.append(distance)
        self.scan.front_min = min(front_values) if front_values else math.inf
        self.scan.rear_min = min(rear_values) if rear_values else math.inf
        self.scan.finite_points = finite_count
        self.scan.last_time = self._now()

    def tick(self) -> None:
        now = self._now()
        cmd, state, immediate_stop = self._select(now)
        cmd = self._limit(cmd)
        if not immediate_stop:
            cmd = self._rate_limit(cmd)
        self.last_out = cmd
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))

    def _select(self, now: float) -> tuple[Twist, str, bool]:
        if self.estop:
            return stop_twist(), "EMERGENCY_STOP final_cmd_vel_zero", True
        if self.external_stop:
            return stop_twist(), "EXTERNAL_STOP final_cmd_vel_zero", True
        if self.mode in {"EMERGENCY", "DISABLED"}:
            return stop_twist(), f"MODE_{self.mode}_STOP final_cmd_vel_zero", True
        if self.mode not in {"AUTO", "PATROL", "TRACK_ONLY", "RETURN_HOME", "MANUAL", "STANDBY"}:
            if bool(self.get_parameter("stop_on_unknown_state").value):
                return stop_twist(), f"UNKNOWN_MODE_STOP mode={self.mode}", True

        selected = self._select_candidate(now)
        scan_state = self._scan_state(now, selected)
        if scan_state in {
            "SCAN_STALE_STOP",
            "SCAN_DEGRADED_STOP",
            "SCAN_ADAPTER_DEGRADED_STOP",
            "SCAN_HARD_STOP_FRONT",
            "SCAN_HARD_STOP_REAR",
        }:
            return stop_twist(), scan_state, True
        cmd, source_state, immediate = selected
        return cmd, f"{source_state} {scan_state}", immediate

    def _select_candidate(self, now: float) -> tuple[Twist, str, bool]:
        manual_fresh = now - self.last_manual_time <= float(self.get_parameter("manual_override_timeout_sec").value)
        manual_nonzero = abs(self.manual_cmd.linear.x) > 1e-5 or abs(self.manual_cmd.angular.z) > 1e-5
        if self.mode == "MANUAL":
            if not manual_fresh:
                return stop_twist(), "MANUAL_COMMAND_TIMEOUT_STOP", True
            return self.manual_cmd, "MANUAL_PASS", False
        if manual_fresh and manual_nonzero:
            return self.manual_cmd, "MANUAL_OVERRIDE", False
        if self.mode == "STANDBY":
            return stop_twist(), "STANDBY_STOP", True
        if now - self.last_auto_time > float(self.get_parameter("command_timeout_sec").value):
            return stop_twist(), "AUTO_COMMAND_TIMEOUT_STOP", True
        return self.auto_cmd, "AUTO_PASS", False

    def _scan_state(self, now: float, selected: tuple[Twist, str, bool]) -> str:
        if self._adapter_degraded():
            return "SCAN_ADAPTER_DEGRADED_STOP"
        if self.scan.last_time == 0.0 or now - self.scan.last_time > float(self.get_parameter("scan_stale_s").value):
            return "SCAN_STALE_STOP" if bool(self.get_parameter("require_scan").value) else "SCAN_STALE_IGNORED"
        if self.scan.finite_points < int(self.get_parameter("min_valid_scan_points").value):
            return "SCAN_DEGRADED_STOP" if bool(self.get_parameter("require_scan").value) else "SCAN_DEGRADED_WARN"
        cmd = selected[0]
        hard = float(self.get_parameter("hard_stop_distance_m").value)
        slow = float(self.get_parameter("slow_down_distance_m").value)
        clear = float(self.get_parameter("clear_distance_m").value)
        if cmd.linear.x > 0.0 and self.scan.front_min <= hard:
            self.last_scan_blocking_state = "SCAN_HARD_STOP_FRONT"
            return "SCAN_HARD_STOP_FRONT"
        if cmd.linear.x < 0.0 and self.scan.rear_min <= hard:
            self.last_scan_blocking_state = "SCAN_HARD_STOP_REAR"
            return "SCAN_HARD_STOP_REAR"
        if self.last_scan_blocking_state != "SCAN_CLEAR":
            if self.scan.front_min < clear or self.scan.rear_min < clear:
                if self.last_scan_blocking_state in {"SCAN_HARD_STOP_FRONT", "SCAN_HARD_STOP_REAR"}:
                    return self.last_scan_blocking_state
            self.last_scan_blocking_state = "SCAN_CLEAR"
        if cmd.linear.x > 0.0 and self.scan.front_min <= slow:
            return "SCAN_SLOW_FRONT"
        return "SCAN_CLEAR"

    def _adapter_degraded(self) -> bool:
        if not bool(self.get_parameter("stop_on_adapter_degraded").value):
            return False
        state = self.scan.adapter_state
        if not state or state == "UNKNOWN":
            return False
        bad_tokens = ("DEGRADED", "STALE", "FAILED", "EMPTY", "NO_POINTS", "NOT_ENABLED")
        return any(token in state for token in bad_tokens)

    def _limit(self, cmd: Twist) -> Twist:
        max_linear = min(float(self.get_parameter("max_linear_speed").value), self.speed_limit)
        max_angular = min(float(self.get_parameter("max_angular_speed").value), self.angular_speed_limit)
        if cmd.linear.x > 0.0 and self.scan.front_min <= float(self.get_parameter("slow_down_distance_m").value):
            max_linear = min(max_linear, 0.06)
        return make_twist(
            clamp(float(cmd.linear.x), -max_linear, max_linear),
            clamp(float(cmd.angular.z), -max_angular, max_angular),
        )

    def _rate_limit(self, cmd: Twist) -> Twist:
        linear_delta = float(self.get_parameter("max_linear_delta_per_tick").value)
        angular_delta = float(self.get_parameter("max_angular_delta_per_tick").value)
        return make_twist(
            self.last_out.linear.x + clamp(cmd.linear.x - self.last_out.linear.x, -linear_delta, linear_delta),
            self.last_out.angular.z + clamp(cmd.angular.z - self.last_out.angular.z, -angular_delta, angular_delta),
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyCmdMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.cmd_pub.publish(stop_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

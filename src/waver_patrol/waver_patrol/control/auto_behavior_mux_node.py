from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waver_patrol.autonomy_common import stop_twist, twist_is_finite


class AutoBehaviorMuxNode(Node):
    """Select one automatic behavior candidate and publish `/waver/cmd_vel_auto`.

    우선순위:
      AUTO: BATTERY_RETURN > TARGET_TRACK > WAYPOINT_PATROL > IDLE
      PATROL: BATTERY_RETURN > WAYPOINT_PATROL > IDLE
      TRACK_ONLY: BATTERY_RETURN > TARGET_TRACK > IDLE
      RETURN_HOME: BATTERY_RETURN > IDLE
    """

    def __init__(self) -> None:
        super().__init__("auto_behavior_mux_node")
        self.declare_parameter("cmd_vel_patrol_topic", "/waver/cmd_vel_patrol")
        self.declare_parameter("cmd_vel_target_track_topic", "/waver/cmd_vel_target_track")
        self.declare_parameter("cmd_vel_return_home_topic", "/waver/cmd_vel_return_home")
        self.declare_parameter("aerial_target_active_topic", "/waver/aerial_target_active")
        self.declare_parameter("return_home_active_topic", "/waver/return_home_active")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("cmd_vel_auto_topic", "/waver/cmd_vel_auto")
        self.declare_parameter("state_topic", "/waver/auto_behavior_state")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("target_active_hold_sec", 0.8)
        self.declare_parameter("return_home_active_hold_sec", 2.0)
        self.declare_parameter("default_mode", "AUTO")
        self.declare_parameter("timer_hz", 20.0)

        self.mode = str(self.get_parameter("default_mode").value).upper()
        self.return_active = False
        self.target_active = False
        self.last_return_active_time = 0.0
        self.last_target_active_time = 0.0
        self.commands: dict[str, tuple[Twist, float]] = {
            "BATTERY_RETURN": (stop_twist(), 0.0),
            "TARGET_TRACK": (stop_twist(), 0.0),
            "WAYPOINT_PATROL": (stop_twist(), 0.0),
        }

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_auto_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_patrol_topic").value), lambda m: self._cmd("WAYPOINT_PATROL", m), 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_target_track_topic").value), lambda m: self._cmd("TARGET_TRACK", m), 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_return_home_topic").value), lambda m: self._cmd("BATTERY_RETURN", m), 10)
        self.create_subscription(Bool, str(self.get_parameter("aerial_target_active_topic").value), self._target_active, 10)
        self.create_subscription(Bool, str(self.get_parameter("return_home_active_topic").value), self._return_active, 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), self._mode, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)

    def _cmd(self, name: str, msg: Twist) -> None:
        self.commands[name] = (msg if twist_is_finite(msg) else stop_twist(), self._now())

    def _target_active(self, msg: Bool) -> None:
        self.target_active = bool(msg.data)
        if self.target_active:
            self.last_target_active_time = self._now()

    def _return_active(self, msg: Bool) -> None:
        self.return_active = bool(msg.data)
        if self.return_active:
            self.last_return_active_time = self._now()

    def _mode(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        self.mode = mode if mode else str(self.get_parameter("default_mode").value).upper()

    def tick(self) -> None:
        behavior, reason = self._select_behavior()
        cmd = stop_twist()
        if behavior in self.commands:
            candidate, stamp = self.commands[behavior]
            if self._now() - stamp <= float(self.get_parameter("command_timeout_sec").value):
                cmd = candidate
            else:
                reason = f"{behavior.lower()}_command_timeout"
                behavior = "IDLE"
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=f"mode={self.mode} selected={behavior} reason={reason}"))

    def _select_behavior(self) -> tuple[str, str]:
        mode = self.mode or str(self.get_parameter("default_mode").value).upper()
        return_active = self.return_active or self._now() - self.last_return_active_time <= float(self.get_parameter("return_home_active_hold_sec").value)
        target_active = self.target_active or self._now() - self.last_target_active_time <= float(self.get_parameter("target_active_hold_sec").value)
        if mode in {"EMERGENCY", "DISABLED", "STANDBY", "MANUAL"}:
            return "IDLE", f"mode_{mode.lower()}_stops_auto"
        if mode == "RETURN_HOME":
            return ("BATTERY_RETURN", "return_home_mode") if return_active else ("IDLE", "return_home_mode_waiting")
        if return_active:
            return "BATTERY_RETURN", "return_home_active"
        if mode == "TRACK_ONLY":
            return ("TARGET_TRACK", "aerial_target_active") if target_active else ("IDLE", "no_target")
        if mode == "PATROL":
            return "WAYPOINT_PATROL", "target_ignored_by_patrol_mode"
        if mode == "AUTO":
            if target_active:
                return "TARGET_TRACK", "aerial_target_active"
            return "WAYPOINT_PATROL", "no_target"
        return "IDLE", "unknown_mode"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoBehaviorMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(stop_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


ACTION_DONE_STATES = {
    "SOUND_TASK_DONE",
    "MISSION_ACTION_DONE",
    "SOUND_TASK_BLOCKED_BY_CLASS",
    "SOUND_TASK_TIMEOUT",
    "TARGET_NOT_BIRD",
    "TARGET_CLASSIFIED_DRONE",
    "TARGET_CLASSIFIED_UNKNOWN",
    "TARGET_CLASSIFIED_IRRELEVANT",
    "CAMERA_ALIGN_FAILED",
}

WAIT_STATES = {"WAIT_TARGET_DEPARTURE"}


def normalize_mission_state(text: str) -> str:
    stripped = (text or "").strip()
    return stripped.split()[0].upper() if stripped else "UNKNOWN"


class TargetDepartureMonitorNode(Node):
    """Watch target departure after mission action and request patrol resume."""

    def __init__(self) -> None:
        super().__init__("target_departure_monitor_node")
        self.declare_parameter("departure_distance_m", 8.0)
        self.declare_parameter("min_wait_after_mission_sec", 1.0)
        self.declare_parameter("target_lost_timeout_sec", 3.0)
        self.declare_parameter("max_wait_departure_sec", 20.0)
        self.declare_parameter("require_mission_action_done", True)
        self.declare_parameter("use_lidar_range_topic", True)
        self.declare_parameter("use_pose_range_fallback", True)

        self.mission_state = "IDLE"
        self.lidar_state = "NO_TARGET"
        self.moving_valid = False
        self.range_m = math.nan
        self.last_target_time = 0.0
        self.action_done_time = 0.0

        self.departed_pub = self.create_publisher(Bool, "/waver/target_departed", 10)
        self.state_pub = self.create_publisher(String, "/waver/target_departure_state", 10)
        self.resume_pub = self.create_publisher(Bool, "/waver/mission_resume_request", 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(PointStamped, "/waver/aerial_target", self.aerial_target_callback, 10)
        self.create_subscription(Float32, "/waver/lidar_target_range_m", lambda m: setattr(self, "range_m", float(m.data)), 10)
        self.create_subscription(String, "/waver/lidar_target_state", lambda m: setattr(self, "lidar_state", m.data.strip().upper()), 10)
        self.create_subscription(PoseStamped, "/waver/bird_target_pose_base", self.pose_target_callback, 10)
        self.create_subscription(Bool, "/waver/moving_target_valid", lambda m: setattr(self, "moving_valid", bool(m.data)), 10)
        self.create_timer(0.2, self.tick)

    def mission_state_callback(self, msg: String) -> None:
        state = normalize_mission_state(msg.data)
        self.mission_state = state
        if state in ACTION_DONE_STATES:
            self.action_done_time = self._now()
        elif state not in WAIT_STATES:
            self.action_done_time = 0.0

    def aerial_target_callback(self, msg: PointStamped) -> None:
        self.last_target_time = self._now()
        if bool(self.get_parameter("use_pose_range_fallback").value) and not bool(self.get_parameter("use_lidar_range_topic").value):
            self.range_m = math.hypot(float(msg.point.x), float(msg.point.y))

    def pose_target_callback(self, msg: PoseStamped) -> None:
        self.last_target_time = self._now()
        if bool(self.get_parameter("use_pose_range_fallback").value) and not math.isfinite(self.range_m):
            self.range_m = math.hypot(float(msg.pose.position.x), float(msg.pose.position.y))

    def tick(self) -> None:
        departed, state = self.evaluate()
        self.departed_pub.publish(Bool(data=departed))
        self.resume_pub.publish(Bool(data=departed))
        self.state_pub.publish(String(data=state))

    def evaluate(self) -> tuple[bool, str]:
        now = self._now()
        if bool(self.get_parameter("require_mission_action_done").value):
            if self.action_done_time == 0.0:
                return False, f"WAITING mission_state={self.mission_state}"
            wait = now - self.action_done_time
            if wait < float(self.get_parameter("min_wait_after_mission_sec").value):
                return False, f"WAIT_AFTER_ACTION elapsed={wait:.2f}"
        elif self.action_done_time == 0.0:
            self.action_done_time = now
        if math.isfinite(self.range_m) and self.range_m >= float(self.get_parameter("departure_distance_m").value):
            return True, f"TARGET_DEPARTED range_m={self.range_m:.2f}"
        if self.last_target_time > 0.0 and now - self.last_target_time >= float(self.get_parameter("target_lost_timeout_sec").value):
            if any(token in self.lidar_state for token in ("LOST", "NO_TARGET")) or not self.moving_valid:
                return True, f"TARGET_LOST_TIMEOUT lidar_state={self.lidar_state}"
        if self.action_done_time > 0.0 and now - self.action_done_time >= float(self.get_parameter("max_wait_departure_sec").value):
            return True, "WAIT_TIMEOUT resume_allowed=true"
        return False, f"TARGET_NEAR range_m={self.range_m:.2f}"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetDepartureMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

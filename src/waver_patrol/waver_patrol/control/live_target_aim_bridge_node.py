from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


OBSERVATION_STATES = {
    "TARGET_REACHED",
    "CAMERA_ALIGN_TO_TARGET",
    "CAMERA_ALIGN_DONE",
    "TARGET_CLASSIFICATION_WAIT",
    "TARGET_CLASSIFIED_BIRD",
    "SOUND_TASK_REQUESTED",
    "SOUND_TASK_RUNNING",
    "WAIT_TARGET_DEPARTURE",
}


class LiveTargetAimBridgeNode(Node):
    """Publish camera aim poses only while the mission is observing a target."""

    def __init__(self) -> None:
        super().__init__("live_target_aim_bridge_node")
        self.declare_parameter("target_topic", "/waver/aerial_target")
        self.declare_parameter("dynamic_targets_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("lidar_target_state_topic", "/waver/lidar_target_state")
        self.declare_parameter("output_topic", "/waver/camera_aim_target_pose")
        self.declare_parameter("target_stale_timeout_sec", 0.8)

        self.target: PointStamped | None = None
        self.last_target_time = 0.0
        self.mission_state = "IDLE"
        self.lidar_state = "NO_TARGET"

        self.aim_pub = self.create_publisher(PoseStamped, str(self.get_parameter("output_topic").value), 10)
        self.state_pub = self.create_publisher(String, "/waver/live_target_aim_state", 10)
        self.create_subscription(PointStamped, str(self.get_parameter("target_topic").value), self.target_callback, 10)
        self.create_subscription(PoseArray, str(self.get_parameter("dynamic_targets_topic").value), self.pose_array_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_state_topic").value), lambda m: setattr(self, "mission_state", m.data.strip().upper()), 10)
        self.create_subscription(String, str(self.get_parameter("lidar_target_state_topic").value), lambda m: setattr(self, "lidar_state", m.data.strip().upper()), 10)
        self.create_timer(0.1, self.tick)

    def target_callback(self, msg: PointStamped) -> None:
        self.target = msg
        self.last_target_time = self._now()

    def pose_array_callback(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        point = PointStamped()
        point.header = msg.header
        point.point = msg.poses[0].position
        self.target_callback(point)

    def tick(self) -> None:
        allowed, reason = self.allowed_to_publish()
        if not allowed:
            self.state_pub.publish(String(data=reason))
            return
        pose = PoseStamped()
        pose.header = self.target.header
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = self.target.point
        pose.pose.orientation.w = 1.0
        self.aim_pub.publish(pose)
        self.state_pub.publish(String(data=f"AIM_TARGET_UPDATED state={self.mission_state} lidar={self.lidar_state}"))

    def allowed_to_publish(self) -> tuple[bool, str]:
        if self.mission_state not in OBSERVATION_STATES:
            return False, f"IDLE mission_state={self.mission_state}"
        if self.target is None:
            return False, "WAIT_TARGET"
        age = self._now() - self.last_target_time
        if age > float(self.get_parameter("target_stale_timeout_sec").value):
            return False, f"TARGET_STALE age_sec={age:.2f}"
        if any(token in self.lidar_state for token in ("LOST", "NO_TARGET", "TF_FAIL")):
            return False, f"TARGET_INVALID lidar_state={self.lidar_state}"
        return True, "OK"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LiveTargetAimBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

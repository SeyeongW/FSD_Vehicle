from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class MovingObjectGoalRelayNode(Node):
    """Publish a validation PoseStamped goal from transformed moving-object coordinates.

    역할:
      - `/waver/lidar_objects_map`이 실제 차량/자율주행 노드가 받아쓸 수 있는지 확인한다.
      - 기본 출력은 preview topic이며 Nav2나 mission goal을 직접 건드리지 않는다.
      - 4D radar mission command와 충돌하지 않도록 `/waver/object_mission_goal`로는 발행하지 않는다.
    """

    def __init__(self) -> None:
        super().__init__("moving_object_goal_relay_node")
        self.declare_parameter("input_pose_array_topic", "/waver/lidar_objects_map")
        self.declare_parameter("output_goal_topic", "/waver/lidar_object_goal_preview")
        self.declare_parameter("active_topic", "/waver/lidar_object_goal_preview_active")
        self.declare_parameter("state_topic", "/waver/lidar_object_goal_preview_state")
        self.declare_parameter("marker_topic", "/waver/lidar_object_goal_preview_marker")
        self.declare_parameter("goal_z_policy", "zero")
        self.declare_parameter("goal_yaw_policy", "face_from_origin")
        self.declare_parameter("min_goal_distance_m", 0.3)
        self.declare_parameter("max_goal_distance_m", 30.0)
        self.declare_parameter("publish_debug_marker", True)
        self.declare_parameter("stale_timeout_sec", 1.0)

        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("output_goal_topic").value), 10)
        self.active_pub = self.create_publisher(Bool, str(self.get_parameter("active_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.last_msg_time = 0.0
        self.create_subscription(PoseArray, str(self.get_parameter("input_pose_array_topic").value), self.pose_array_callback, 10)
        self.create_timer(0.2, self.timeout_tick)

    def pose_array_callback(self, msg: PoseArray) -> None:
        self.last_msg_time = self._now()
        candidate = self._select_candidate(msg)
        if candidate is None:
            self.active_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data=f"NO_VALID_GOAL frame={msg.header.frame_id} count={len(msg.poses)}"))
            return
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = candidate
        if str(self.get_parameter("goal_z_policy").value).strip().lower() == "zero":
            goal.pose.position.z = 0.0
        goal.pose.orientation = self._orientation_for_goal(candidate)
        self.goal_pub.publish(goal)
        self.active_pub.publish(Bool(data=True))
        if bool(self.get_parameter("publish_debug_marker").value):
            self.marker_pub.publish(self._make_marker(goal))
        self.state_pub.publish(
            String(
                data=(
                    f"GOAL_PREVIEW frame={goal.header.frame_id} "
                    f"x={goal.pose.position.x:.3f} y={goal.pose.position.y:.3f} z={goal.pose.position.z:.3f}"
                )
            )
        )

    def _select_candidate(self, msg: PoseArray) -> Pose | None:
        best: Pose | None = None
        best_distance = math.inf
        min_distance = float(self.get_parameter("min_goal_distance_m").value)
        max_distance = float(self.get_parameter("max_goal_distance_m").value)
        for pose in msg.poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            distance = math.hypot(x, y)
            if distance < min_distance or distance > max_distance:
                continue
            if distance < best_distance:
                best = pose
                best_distance = distance
        return best

    def _orientation_for_goal(self, pose: Pose):
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        policy = str(self.get_parameter("goal_yaw_policy").value).strip().lower()
        if policy == "face_from_origin":
            yaw = math.atan2(float(pose.position.y), float(pose.position.x))
            quat.z = math.sin(yaw * 0.5)
            quat.w = math.cos(yaw * 0.5)
        else:
            quat.w = 1.0
        return quat

    def _make_marker(self, goal: PoseStamped) -> MarkerArray:
        markers = MarkerArray()
        marker = Marker()
        marker.header = goal.header
        marker.ns = "waver_lidar_goal_preview"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.55
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(marker)
        return markers

    def timeout_tick(self) -> None:
        if self.last_msg_time and self._now() - self.last_msg_time > float(self.get_parameter("stale_timeout_sec").value):
            self.active_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data="GOAL_PREVIEW_STALE"))
            self.last_msg_time = 0.0

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MovingObjectGoalRelayNode()
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

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def safe_float(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except Exception:
        return default


class WaverSpatialDebugVizNode(Node):
    """RViz-only spatial debug overlay for the Gazebo bird patrol trial."""

    def __init__(self) -> None:
        super().__init__("waver_spatial_debug_viz_node")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("marker_period_sec", 0.2)
        self.declare_parameter("stale_timeout_sec", 3.0)
        self.declare_parameter("configured_offset_m", 2.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.configured_offset_m = float(self.get_parameter("configured_offset_m").value)

        self.robot: dict[str, Any] = {}
        self.bird: dict[str, Any] = {}
        self.lidar_target: dict[str, Any] = {}
        self.object_goal: dict[str, Any] = {}
        self.active_goal: dict[str, Any] = {}
        self.mission_state = "UNKNOWN"

        self.pub = self.create_publisher(MarkerArray, "/waver/spatial_debug_markers", 10)
        self.state_pub = self.create_publisher(String, "/waver/spatial_debug_viz_state", 10)

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/waver/gazebo_bird_kinematics", self.bird_callback, 10)
        self.create_subscription(PoseStamped, "/waver/lidar_target_pose_odom", self.lidar_target_callback, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.object_goal_callback, 10)
        self.create_subscription(PoseStamped, "/waver/active_nav_goal", self.active_goal_callback, 10)
        self.create_subscription(String, "/waver/mission_state", lambda m: setattr(self, "mission_state", m.data), 10)
        self.create_timer(float(self.get_parameter("marker_period_sec").value), self.publish_markers)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh(self, item: dict[str, Any]) -> bool:
        t = safe_float(item.get("time"))
        return math.isfinite(t) and self.now_sec() - t <= self.stale_timeout_sec

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.robot = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "time": self.now_sec()}

    def bird_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        nearest = str(payload.get("nearest_name") or "")
        chosen = None
        for bird in payload.get("birds", []):
            if str(bird.get("name", "")) == nearest:
                chosen = bird
                break
        if chosen is None:
            for bird in payload.get("birds", []):
                if bird.get("active") and not bird.get("hidden") and not bird.get("removed"):
                    chosen = bird
                    break
        if chosen is None:
            return
        if chosen.get("hidden") or chosen.get("removed"):
            return
        self.bird = {
            "name": str(chosen.get("name", "bird")),
            "x": safe_float(chosen.get("x")),
            "y": safe_float(chosen.get("y")),
            "z": safe_float(chosen.get("z"), 0.0),
            "state": str(chosen.get("state", "")),
            "time": self.now_sec(),
        }

    def lidar_target_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.lidar_target = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "time": self.now_sec()}

    def object_goal_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.object_goal = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "time": self.now_sec()}

    def active_goal_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.active_goal = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "time": self.now_sec()}

    def marker(
        self,
        marker_id: int,
        ns: str,
        marker_type: int,
        x: float,
        y: float,
        z: float,
        color: ColorRGBA,
        scale: tuple[float, float, float],
        text: str = "",
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color = color
        m.text = text
        m.lifetime.sec = 1
        return m

    @staticmethod
    def color(r: float, g: float, b: float, a: float = 0.9) -> ColorRGBA:
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def line(self, marker_id: int, ns: str, a: dict[str, Any], b: dict[str, Any], color: ColorRGBA) -> Marker:
        m = self.marker(marker_id, ns, Marker.LINE_STRIP, 0.0, 0.0, 0.0, color, (0.035, 0.035, 0.035))
        m.points = [
            Point(x=safe_float(a.get("x")), y=safe_float(a.get("y")), z=safe_float(a.get("z"), 0.05)),
            Point(x=safe_float(b.get("x")), y=safe_float(b.get("y")), z=safe_float(b.get("z"), 0.05)),
        ]
        return m

    def publish_markers(self) -> None:
        markers: list[Marker] = []
        if self.fresh(self.robot):
            markers.append(
                self.marker(1, "waver", Marker.CUBE, self.robot["x"], self.robot["y"], 0.18, self.color(0.1, 0.35, 1.0), (0.45, 0.32, 0.18))
            )
            markers.append(
                self.marker(2, "labels", Marker.TEXT_VIEW_FACING, self.robot["x"], self.robot["y"], 0.65, self.color(1.0, 1.0, 1.0), (0.0, 0.0, 0.22), "Waver")
            )
        if self.fresh(self.bird):
            markers.append(
                self.marker(10, "bird", Marker.SPHERE, self.bird["x"], self.bird["y"], self.bird["z"], self.color(1.0, 0.8, 0.05), (0.35, 0.35, 0.35))
            )
            markers.append(
                self.marker(
                    11,
                    "labels",
                    Marker.TEXT_VIEW_FACING,
                    self.bird["x"],
                    self.bird["y"],
                    self.bird["z"] + 0.45,
                    self.color(1.0, 0.95, 0.4),
                    (0.0, 0.0, 0.22),
                    f"{self.bird.get('name', 'bird')} {self.bird.get('state', '')}",
                )
            )
        if self.fresh(self.lidar_target):
            markers.append(
                self.marker(20, "lidar_target", Marker.SPHERE, self.lidar_target["x"], self.lidar_target["y"], self.lidar_target["z"], self.color(0.0, 1.0, 0.45), (0.25, 0.25, 0.25))
            )
        if self.fresh(self.object_goal):
            markers.append(
                self.marker(30, "object_goal", Marker.CYLINDER, self.object_goal["x"], self.object_goal["y"], 0.04, self.color(1.0, 0.25, 0.1), (0.22, 0.22, 0.08))
            )
        if self.fresh(self.active_goal):
            markers.append(
                self.marker(40, "active_goal", Marker.CYLINDER, self.active_goal["x"], self.active_goal["y"], 0.08, self.color(0.6, 0.1, 1.0), (0.28, 0.28, 0.12))
            )
        if self.fresh(self.active_goal) and self.fresh(self.lidar_target):
            markers.append(self.line(50, "goal_to_lidar", self.active_goal, self.lidar_target, self.color(1.0, 0.1, 0.1)))
        if self.fresh(self.robot) and self.fresh(self.active_goal):
            markers.append(self.line(51, "robot_to_goal", self.robot, self.active_goal, self.color(0.35, 0.75, 1.0)))

        self.pub.publish(MarkerArray(markers=markers))
        self.state_pub.publish(
            String(
                data=(
                    f"SPATIAL_DEBUG markers={len(markers)} mission={self.mission_state} "
                    f"robot={self.fresh(self.robot)} bird={self.fresh(self.bird)} lidar={self.fresh(self.lidar_target)}"
                )
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaverSpatialDebugVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

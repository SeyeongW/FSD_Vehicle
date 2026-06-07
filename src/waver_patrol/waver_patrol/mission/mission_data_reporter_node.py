from __future__ import annotations

import json
import os
import time
import urllib.request

import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class MissionDataReporterNode(Node):
    """Publish mission result reports as JSON without blocking driving."""

    def __init__(self) -> None:
        super().__init__("mission_data_reporter_node")
        self.declare_parameter("trial_id", "field_trial")
        self.declare_parameter("output_root", "~/ros2_ws2/FSD_Vehicle/experiment_results")
        self.declare_parameter("external_send_enabled", False)
        self.declare_parameter("http_post_url", "")
        self.declare_parameter("write_jsonl", True)

        self.state = "IDLE"
        self.target_class = "unknown"
        self.confidence = 0.0
        self.bird_confirmed = False
        self.target_height = 0.0
        self.target_range = 0.0
        self.target_velocity = 0.0
        self.target: PointStamped | None = None
        self.gimbal = Vector3()
        self.sound_state = "UNKNOWN"
        self.robot_pose = {}
        self.mission_id = 0

        root = os.path.expanduser(os.path.expandvars(str(self.get_parameter("output_root").value)))
        os.makedirs(root, exist_ok=True)
        self.jsonl_path = os.path.join(root, "mission_reports.jsonl")
        self.report_pub = self.create_publisher(String, "/waver/mission_report", 10)

        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(String, "/waver/target_class", lambda m: setattr(self, "target_class", m.data), 10)
        self.create_subscription(Float32, "/waver/target_confidence", lambda m: setattr(self, "confidence", float(m.data)), 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_subscription(PointStamped, "/waver/aerial_target", lambda m: setattr(self, "target", m), 10)
        self.create_subscription(Float32, "/waver/lidar_target_height_m", lambda m: setattr(self, "target_height", float(m.data)), 10)
        self.create_subscription(Float32, "/waver/lidar_target_range_m", lambda m: setattr(self, "target_range", float(m.data)), 10)
        self.create_subscription(Float32, "/waver/lidar_target_velocity_mps", lambda m: setattr(self, "target_velocity", float(m.data)), 10)
        self.create_subscription(Vector3, "/waver/camera_gimbal_feedback", lambda m: setattr(self, "gimbal", m), 10)
        self.create_subscription(String, "/waver/sound_state", lambda m: setattr(self, "sound_state", m.data), 10)
        self.create_subscription(String, "/waver/sound_alert_state", lambda m: setattr(self, "sound_state", m.data), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

    def mission_state_callback(self, msg: String) -> None:
        previous = self.state
        self.state = msg.data.strip().upper()
        if self.state != previous:
            if self.state in {"TARGET_CLASSIFIED_BIRD", "SOUND_TASK_DONE", "TARGET_NOT_BIRD", "RETURN_TO_INTERRUPTED_WAYPOINT", "RESUME_PATROL"}:
                self.publish_report(event_type=self.state)

    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.robot_pose = {
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
        }

    def odom_callback(self, msg: Odometry) -> None:
        if self.robot_pose:
            return
        self.robot_pose = {
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
        }

    def publish_report(self, event_type: str) -> None:
        if event_type in {"TARGET_CLASSIFIED_BIRD", "TARGET_NOT_BIRD"}:
            self.mission_id += 1
        report = {
            "timestamp": time.time(),
            "trial_id": str(self.get_parameter("trial_id").value),
            "mission_id": self.mission_id,
            "target_id": self.mission_id,
            "target_class": self.target_class,
            "target_confidence": self.confidence,
            "bird_confirmed": self.bird_confirmed,
            "target_height_m": self.target_height,
            "target_range_m": self.target_range,
            "target_velocity_mps": self.target_velocity,
            "robot_pose": self.robot_pose,
            "gimbal_pan": float(self.gimbal.x),
            "gimbal_tilt": float(self.gimbal.y),
            "sound_action_result": self.sound_state,
            "mission_state": self.state,
            "event_type": event_type,
        }
        text = json.dumps(report, separators=(",", ":"), ensure_ascii=False)
        self.report_pub.publish(String(data=text))
        if bool(self.get_parameter("write_jsonl").value):
            with open(self.jsonl_path, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        self.send_external(text)

    def send_external(self, text: str) -> None:
        if not bool(self.get_parameter("external_send_enabled").value):
            return
        url = str(self.get_parameter("http_post_url").value).strip()
        if not url:
            return
        try:
            request = urllib.request.Request(url, data=text.encode("utf-8"), headers={"Content-Type": "application/json"})
            urllib.request.urlopen(request, timeout=0.3).close()
        except Exception as exc:
            self.get_logger().warn(f"mission report external send failed: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionDataReporterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

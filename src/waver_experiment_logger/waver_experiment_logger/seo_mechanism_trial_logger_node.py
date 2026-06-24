from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


@dataclass
class TopicState:
    last_data: str = ""
    last_time: float = 0.0


class SeoMechanismTrialLoggerNode(Node):
    """Small Gazebo mechanism logger.

    This node is intentionally lightweight: it records whether the required
    mechanism stages appear on ROS topics during a Gazebo trial. It does not
    publish control commands.
    """

    def __init__(self) -> None:
        super().__init__("seo_mechanism_trial_logger_node")
        self.declare_parameter("output_root", "/home/chotaehyun/ros2_ws5/FSD_Vehicle/experiment_results/gazebo_bird_patrol")
        self.declare_parameter("trial_id", "gazebo_seo_bird_patrol")
        self.declare_parameter("run_id", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("summary_period_sec", 0.5)

        explicit_run_dir = os.path.expanduser(os.path.expandvars(str(self.get_parameter("run_dir").value))).strip()
        if explicit_run_dir:
            self.run_dir = explicit_run_dir
        else:
            root = os.path.expanduser(os.path.expandvars(str(self.get_parameter("output_root").value)))
            os.makedirs(root, exist_ok=True)
            run_id = str(self.get_parameter("run_id").value).strip()
            if not run_id:
                stamp = time.strftime("%Y%m%d_%H%M%S")
                trial_id = str(self.get_parameter("trial_id").value)
                run_id = f"{trial_id}_{stamp}"
            self.run_dir = os.path.join(root, run_id)
        os.makedirs(os.path.join(self.run_dir, "logs"), exist_ok=True)
        self.csv_path = os.path.join(self.run_dir, "logs", "mechanism_events.csv")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "ros_time_sec",
                "event",
                "detail",
                "mission_state",
                "mode",
                "cmd_linear",
                "cmd_angular",
                "odom_x",
                "odom_y",
                "dynamic_lock",
                "bird_confirmed",
                "sound_done",
            ],
        )
        self.writer.writeheader()

        self.mode = "UNKNOWN"
        self.mission_state = "UNKNOWN"
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.dynamic_lock = False
        self.bird_confirmed = False
        self.sound_done = False
        self.seen = {
            "patrol": False,
            "dynamic_target": False,
            "lock": False,
            "inspection_goal": False,
            "camera_align": False,
            "classification": False,
            "sound": False,
            "return_or_resume": False,
            "bird_removed": False,
        }

        self.state_pub = self.create_publisher(String, "/waver/seo_trial_logger_state", 10)
        self.create_subscription(String, "/waver/mode", self.mode_callback, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_state_callback, 10)
        self.create_subscription(String, "/waver/mission_event", lambda m: self.log_event("mission_event", m.data), 10)
        self.create_subscription(String, "/waver/object_mission_goal_state", self.object_goal_state_callback, 10)
        self.create_subscription(PoseArray, "/waver/elevated_dynamic_targets", self.targets_callback, 10)
        self.create_subscription(Bool, "/waver/dynamic_object_lock", self.lock_callback, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", lambda m: self.log_event("inspection_goal", f"x={m.pose.position.x:.2f} y={m.pose.position.y:.2f}"), 10)
        self.create_subscription(String, "/waver/camera_alignment_state", self.camera_callback, 10)
        self.create_subscription(String, "/waver/classification_state", self.classification_callback, 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", self.bird_callback, 10)
        self.create_subscription(String, "/waver/sound_alert_state", self.sound_callback, 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.sound_done_callback, 10)
        self.create_subscription(String, "/waver/gazebo_bird_removal_state", self.bird_removal_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_timer(float(self.get_parameter("summary_period_sec").value), self.summary_tick)
        self.get_logger().info(f"Mechanism event log: {self.csv_path}")

    def mode_callback(self, msg: String) -> None:
        self.mode = msg.data

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = msg.data
        state = msg.data.upper()
        if "PATROL" in state:
            self.seen["patrol"] = True
        if "CAMERA_ALIGN" in state:
            self.seen["camera_align"] = True
        if "SOUND" in state:
            self.seen["sound"] = True
        if "RETURN" in state or "RESUME" in state:
            self.seen["return_or_resume"] = True
        self.log_event("mission_state", msg.data)

    def targets_callback(self, msg: PoseArray) -> None:
        if msg.poses:
            self.seen["dynamic_target"] = True
            p = msg.poses[0].position
            self.log_event("dynamic_target", f"count={len(msg.poses)} x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")

    def object_goal_state_callback(self, msg: String) -> None:
        if "ACCEPT_INSPECTION_GOAL" in msg.data:
            self.seen["inspection_goal"] = True
        self.log_event("object_goal_state", msg.data)

    def lock_callback(self, msg: Bool) -> None:
        self.dynamic_lock = bool(msg.data)
        if self.dynamic_lock:
            self.seen["lock"] = True
        self.log_event("dynamic_lock", str(self.dynamic_lock))

    def camera_callback(self, msg: String) -> None:
        if msg.data:
            self.seen["camera_align"] = True
        self.log_event("camera_alignment", msg.data)

    def classification_callback(self, msg: String) -> None:
        if "BIRD" in msg.data.upper() or "CLASSIFICATION" in msg.data.upper():
            self.seen["classification"] = True
        self.log_event("classification", msg.data)

    def bird_callback(self, msg: Bool) -> None:
        self.bird_confirmed = bool(msg.data)
        if self.bird_confirmed:
            self.seen["classification"] = True
        self.log_event("bird_confirmed", str(self.bird_confirmed))

    def sound_callback(self, msg: String) -> None:
        if "SOUND" in msg.data.upper():
            self.seen["sound"] = True
        self.log_event("sound_state", msg.data)

    def sound_done_callback(self, msg: Bool) -> None:
        self.sound_done = bool(msg.data)
        if self.sound_done:
            self.seen["sound"] = True
        self.log_event("sound_done", str(self.sound_done))

    def bird_removal_callback(self, msg: String) -> None:
        if msg.data.startswith("REMOVED"):
            self.seen["bird_removed"] = True
            self.log_event("bird_removed", msg.data)
        else:
            self.log_event("bird_removal_state", msg.data)

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

    def summary_tick(self) -> None:
        done = ",".join(k for k, v in self.seen.items() if v)
        missing = ",".join(k for k, v in self.seen.items() if not v)
        self.state_pub.publish(String(data=f"SEEN={done or 'none'} MISSING={missing or 'none'} log={self.csv_path}"))
        self.csv_file.flush()

    def log_event(self, event: str, detail: str) -> None:
        self.writer.writerow(
            {
                "ros_time_sec": f"{self.get_clock().now().nanoseconds * 1e-9:.3f}",
                "event": event,
                "detail": detail,
                "mission_state": self.mission_state,
                "mode": self.mode,
                "cmd_linear": f"{self.cmd_linear:.3f}",
                "cmd_angular": f"{self.cmd_angular:.3f}",
                "odom_x": f"{self.odom_x:.3f}",
                "odom_y": f"{self.odom_y:.3f}",
                "dynamic_lock": self.dynamic_lock,
                "bird_confirmed": self.bird_confirmed,
                "sound_done": self.sound_done,
            }
        )

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeoMechanismTrialLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

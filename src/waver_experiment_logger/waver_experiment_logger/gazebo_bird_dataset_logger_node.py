from __future__ import annotations

import csv
import json
import math
import os
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32, String

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover
    Buffer = None
    TransformException = Exception
    TransformListener = None


def stamp_sec(msg: Any) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except Exception:
        return default


def quat_rotate(qx: float, qy: float, qz: float, qw: float, p: tuple[float, float, float]) -> tuple[float, float, float]:
    # Quaternion-vector multiplication written locally to avoid an extra runtime dependency.
    x, y, z = p
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(tf: TransformStamped, p: tuple[float, float, float]) -> tuple[float, float, float]:
    q = tf.transform.rotation
    t = tf.transform.translation
    rx, ry, rz = quat_rotate(q.x, q.y, q.z, q.w, p)
    return (rx + t.x, ry + t.y, rz + t.z)


@dataclass
class CsvFile:
    path: Path
    fields: list[str]
    handle: Any
    writer: csv.DictWriter

    def row(self, data: dict[str, Any]) -> None:
        out = {field: data.get(field, "") for field in self.fields}
        self.writer.writerow(out)
        self.handle.flush()


class GazeboBirdDatasetLoggerNode(Node):
    """Paper-grade, observation-only dataset logger for Gazebo bird patrol trials."""

    def __init__(self) -> None:
        super().__init__("gazebo_bird_dataset_logger_node")
        self.declare_parameter("output_root", str(Path.home() / "ros2_ws5/FSD_Vehicle/experiment_results/gazebo_bird_patrol"))
        self.declare_parameter("trial_id", "gazebo_bird_patrol")
        self.declare_parameter("experiment_id", "waver_gazebo_bird_patrol")
        self.declare_parameter("run_id", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("detector_mode", "lidar")
        self.declare_parameter("classifier_mode", "fake_gazebo")
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("save_images", True)
        self.declare_parameter("save_every_nth_image", 1)
        self.declare_parameter("image_topic", "/pt_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/pt_camera/camera_info")
        self.declare_parameter("bird_ground_truth_topic", "/bird/nearest_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pointcloud_topic", "/mid360_PointCloud2")
        self.declare_parameter("lidar_detection_topic", "/waver/elevated_dynamic_targets")
        self.declare_parameter("lidar_state_topic", "/waver/lidar_tracking_state")
        self.declare_parameter("mission_state_topic", "/waver/mission_state")
        self.declare_parameter("mission_event_topic", "/waver/mission_event")
        self.declare_parameter("camera_alignment_state_topic", "/waver/camera_alignment_state")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("sound_alert_state_topic", "/waver/sound_alert_state")
        self.declare_parameter("sound_task_done_topic", "/waver/sound_task_done")
        self.declare_parameter("bird_removal_state_topic", "/waver/gazebo_bird_removal_state")
        self.declare_parameter("safety_state_topic", "/waver/safety_state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("bird_bbox_dims_m", [1.0, 1.0, 1.0])
        self.declare_parameter("min_bbox_area_px", 4.0)
        self.declare_parameter("require_camera_info", True)
        self.declare_parameter("require_tf_for_projection", True)
        self.declare_parameter("write_coco", True)
        self.declare_parameter("write_yolo", True)
        self.declare_parameter("association_radius_m", 1.5)
        self.declare_parameter("summary_period_sec", 1.0)
        self.declare_parameter("camera_frame_convention", "auto")
        self.declare_parameter("world_frame_alias", "odom")
        self.declare_parameter("enable_gazebo_projection_fallback", True)
        self.declare_parameter("camera_mount_xyz_base", [0.039, 0.019, 0.231])
        self.declare_parameter("launch_args_yaml", "")

        self.detector_mode = str(self.get_parameter("detector_mode").value).strip().lower()
        self.classifier_mode = str(self.get_parameter("classifier_mode").value).strip().lower()
        self.run_dir = self.make_run_dir()
        self.make_dirs()
        self.csvs: dict[str, CsvFile] = {}
        self.open_csvs()

        self.camera_info: CameraInfo | None = None
        self.gt_pose: PoseStamped | None = None
        self.gt_used_for_decision = self.detector_mode in {"ground_truth", "fused"}
        self.odom: Odometry | None = None
        self.lidar_poses: list[PoseStamped] = []
        self.lidar_state = "UNKNOWN"
        self.mission_state = "UNKNOWN"
        self.target_class = "none"
        self.target_confidence = 0.0
        self.bird_confirmed = False
        self.sound_state = "UNKNOWN"
        self.bird_removal_state = "UNKNOWN"
        self.removed_bird_count = 0
        self.safety_state = "UNKNOWN"
        self.image_count = 0
        self.coco_images: list[dict[str, Any]] = []
        self.coco_annotations: list[dict[str, Any]] = []
        self.topic_stats: dict[str, dict[str, float]] = {}

        self.state_pub = self.create_publisher(String, "/waver/dataset_logger_state", 10)
        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self.image_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.camera_info_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("bird_ground_truth_topic").value), self.gt_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), lambda m: self.note_topic(str(self.get_parameter("pointcloud_topic").value), stamp_sec(m)), qos_profile_sensor_data)
        self.create_subscription(PoseArray, str(self.get_parameter("lidar_detection_topic").value), self.lidar_detection_callback, 10)
        self.create_subscription(String, str(self.get_parameter("lidar_state_topic").value), self.lidar_state_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_state_topic").value), self.mission_state_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mission_event_topic").value), self.mission_event_callback, 10)
        self.create_subscription(String, str(self.get_parameter("camera_alignment_state_topic").value), self.camera_alignment_callback, 10)
        self.create_subscription(String, str(self.get_parameter("target_class_topic").value), self.target_class_callback, 10)
        self.create_subscription(Float32, str(self.get_parameter("target_confidence_topic").value), self.target_confidence_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("bird_confirmed_topic").value), self.bird_confirmed_callback, 10)
        self.create_subscription(String, str(self.get_parameter("sound_alert_state_topic").value), self.sound_state_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("sound_task_done_topic").value), self.sound_done_callback, 10)
        self.create_subscription(String, str(self.get_parameter("bird_removal_state_topic").value), self.bird_removal_callback, 10)
        self.create_subscription(String, str(self.get_parameter("safety_state_topic").value), self.safety_callback, 10)

        self.write_metadata()
        self.create_timer(float(self.get_parameter("summary_period_sec").value), self.periodic_summary)
        self.get_logger().info(f"dataset logger writing to {self.run_dir}")

    def make_run_dir(self) -> Path:
        explicit = str(self.get_parameter("run_dir").value).strip()
        if explicit:
            return Path(os.path.expanduser(explicit)).resolve()
        run_id = str(self.get_parameter("run_id").value).strip()
        if not run_id:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            run_id = f"{str(self.get_parameter('trial_id').value)}_{stamp}"
        return (Path(os.path.expanduser(str(self.get_parameter("output_root").value))) / run_id).resolve()

    def make_dirs(self) -> None:
        for rel in [
            "metadata",
            "bags",
            "images/pt_camera",
            "annotations/yolo_labels",
            "annotations",
            "logs",
            "metrics",
            "plots",
        ]:
            (self.run_dir / rel).mkdir(parents=True, exist_ok=True)

    def open_csvs(self) -> None:
        specs = {
            "frame_index": ("annotations/frame_index.csv", ["frame_id", "stamp_sec", "image_path", "label_path", "coco_image_id", "gt_valid", "bbox_x", "bbox_y", "bbox_w", "bbox_h", "projection_reason"]),
            "ground_truth_3d": ("annotations/ground_truth_3d.csv", ["time_sec", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw", "used_for_decision", "detector_mode"]),
            "camera_projection_debug": ("annotations/camera_projection_debug.csv", ["time_sec", "image_id", "valid", "reason", "camera_frame", "gt_frame", "points_projected", "bbox_x", "bbox_y", "bbox_w", "bbox_h"]),
            "lidar_gt_association": ("annotations/lidar_gt_association.csv", ["time_sec", "lidar_count", "gt_x", "gt_y", "gt_z", "nearest_dist_m", "associated", "detector_mode", "gt_used_for_decision"]),
            "mission_events": ("logs/mission_events.csv", ["time_sec", "mission_state", "event"]),
            "lidar_detections": ("logs/lidar_detections.csv", ["time_sec", "count", "state", "nearest_gt_dist_m", "associated"]),
            "camera_alignment": ("logs/camera_alignment.csv", ["time_sec", "state"]),
            "classifier_events": ("logs/classifier_events.csv", ["time_sec", "target_class", "confidence", "bird_confirmed", "classifier_mode"]),
            "sound_events": ("logs/sound_events.csv", ["time_sec", "state", "done"]),
            "bird_removal_events": ("logs/bird_removal_events.csv", ["time_sec", "state", "bird_name", "removed_count", "remaining_count"]),
            "safety_check": ("logs/safety_check.csv", ["time_sec", "state"]),
            "experiment_summary": ("logs/experiment_summary.csv", ["time_sec", "mission_state", "lidar_state", "target_class", "confidence", "bird_confirmed", "sound_state", "bird_removal_state", "removed_bird_count", "safety_state", "image_count", "gt_used_for_decision"]),
            "paper_metrics_summary": ("metrics/paper_metrics_summary.csv", ["metric", "value"]),
        }
        for name, (rel, fields) in specs.items():
            path = self.run_dir / rel
            handle = open(path, "w", newline="")
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            self.csvs[name] = CsvFile(path, fields, handle, writer)

    def write_metadata(self) -> None:
        git_hash = ""
        try:
            git_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=Path.cwd(), text=True).strip()
        except Exception:
            git_hash = "unknown"
        metadata = {
            "experiment_id": str(self.get_parameter("experiment_id").value),
            "trial_id": str(self.get_parameter("trial_id").value),
            "run_dir": str(self.run_dir),
            "created_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "detector_mode": self.detector_mode,
            "classifier_mode": self.classifier_mode,
            "random_seed": int(self.get_parameter("random_seed").value),
            "git_hash": git_hash,
            "control_publishers": "none",
        }
        (self.run_dir / "metadata/run_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        launch_args_text = str(self.get_parameter("launch_args_yaml").value)
        try:
            launch_args = yaml.safe_load(launch_args_text) if launch_args_text.strip() else {}
        except Exception:
            launch_args = {"raw": launch_args_text}
        (self.run_dir / "metadata/launch_args.yaml").write_text(yaml.safe_dump(launch_args, sort_keys=True), encoding="utf-8")
        params = {name: self.param_to_python(self.get_parameter(name).value) for name in self._parameters.keys()}
        (self.run_dir / "metadata/parameter_snapshot.yaml").write_text(yaml.safe_dump(params, sort_keys=True), encoding="utf-8")
        manifest = {"run_dir": str(self.run_dir), "bags": "bags/", "topics": self.topic_list_for_manifest()}
        (self.run_dir / "metadata/replay_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=True), encoding="utf-8")

    def topic_list_for_manifest(self) -> list[str]:
        return [
            str(self.get_parameter("image_topic").value),
            str(self.get_parameter("camera_info_topic").value),
            str(self.get_parameter("bird_ground_truth_topic").value),
            str(self.get_parameter("pointcloud_topic").value),
            str(self.get_parameter("lidar_detection_topic").value),
            str(self.get_parameter("mission_state_topic").value),
            str(self.get_parameter("bird_removal_state_topic").value),
            str(self.get_parameter("cmd_vel_topic").value),
        ]

    @staticmethod
    def param_to_python(value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, (list, tuple)):
            return [GazeboBirdDatasetLoggerNode.param_to_python(v) for v in value]
        if isinstance(value, dict):
            return {str(k): GazeboBirdDatasetLoggerNode.param_to_python(v) for k, v in value.items()}
        return str(value)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.note_topic(str(self.get_parameter("camera_info_topic").value), stamp_sec(msg))

    def gt_callback(self, msg: PoseStamped) -> None:
        self.gt_pose = PoseStamped()
        self.gt_pose.header = msg.header
        self.gt_pose.pose = msg.pose
        if self.gt_pose.header.frame_id == "world":
            self.gt_pose.header.frame_id = str(self.get_parameter("world_frame_alias").value)
        self.note_topic(str(self.get_parameter("bird_ground_truth_topic").value), stamp_sec(msg))
        p = msg.pose.position
        q = msg.pose.orientation
        self.csvs["ground_truth_3d"].row(
            {
                "time_sec": self.now_sec(),
                "frame_id": self.gt_pose.header.frame_id,
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "qx": q.x,
                "qy": q.y,
                "qz": q.z,
                "qw": q.w,
                "used_for_decision": str(self.gt_used_for_decision).lower(),
                "detector_mode": self.detector_mode,
            }
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.note_topic(str(self.get_parameter("odom_topic").value), stamp_sec(msg))

    def lidar_detection_callback(self, msg: PoseArray) -> None:
        self.note_topic(str(self.get_parameter("lidar_detection_topic").value), stamp_sec(msg))
        self.lidar_poses = []
        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = pose
            self.lidar_poses.append(stamped)
        assoc = self.association()
        self.csvs["lidar_detections"].row(
            {
                "time_sec": self.now_sec(),
                "count": len(self.lidar_poses),
                "state": self.lidar_state,
                "nearest_gt_dist_m": assoc["nearest_dist_m"],
                "associated": assoc["associated"],
            }
        )
        self.csvs["lidar_gt_association"].row(
            {
                "time_sec": self.now_sec(),
                "lidar_count": len(self.lidar_poses),
                "gt_x": assoc["gt_x"],
                "gt_y": assoc["gt_y"],
                "gt_z": assoc["gt_z"],
                "nearest_dist_m": assoc["nearest_dist_m"],
                "associated": assoc["associated"],
                "detector_mode": self.detector_mode,
                "gt_used_for_decision": str(self.gt_used_for_decision).lower(),
            }
        )

    def lidar_state_callback(self, msg: String) -> None:
        self.lidar_state = msg.data
        self.note_topic(str(self.get_parameter("lidar_state_topic").value), self.now_sec())

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = msg.data
        self.csvs["mission_events"].row({"time_sec": self.now_sec(), "mission_state": self.mission_state, "event": "STATE"})

    def mission_event_callback(self, msg: String) -> None:
        self.csvs["mission_events"].row({"time_sec": self.now_sec(), "mission_state": self.mission_state, "event": msg.data})

    def camera_alignment_callback(self, msg: String) -> None:
        self.csvs["camera_alignment"].row({"time_sec": self.now_sec(), "state": msg.data})

    def target_class_callback(self, msg: String) -> None:
        self.target_class = msg.data
        self.write_classifier_row()

    def target_confidence_callback(self, msg: Float32) -> None:
        self.target_confidence = float(msg.data)
        self.write_classifier_row()

    def bird_confirmed_callback(self, msg: Bool) -> None:
        self.bird_confirmed = bool(msg.data)
        self.write_classifier_row()

    def write_classifier_row(self) -> None:
        self.csvs["classifier_events"].row(
            {
                "time_sec": self.now_sec(),
                "target_class": self.target_class,
                "confidence": self.target_confidence,
                "bird_confirmed": str(self.bird_confirmed).lower(),
                "classifier_mode": self.classifier_mode,
            }
        )

    def sound_state_callback(self, msg: String) -> None:
        self.sound_state = msg.data
        self.csvs["sound_events"].row({"time_sec": self.now_sec(), "state": self.sound_state, "done": ""})

    def sound_done_callback(self, msg: Bool) -> None:
        self.csvs["sound_events"].row({"time_sec": self.now_sec(), "state": self.sound_state, "done": str(bool(msg.data)).lower()})

    @staticmethod
    def parse_key_values(text: str) -> dict[str, str]:
        out: dict[str, str] = {}
        for match in re.finditer(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)", text):
            out[match.group(1)] = match.group(2)
        return out

    def bird_removal_callback(self, msg: String) -> None:
        self.bird_removal_state = msg.data
        fields = self.parse_key_values(msg.data)
        if msg.data.startswith("REMOVED"):
            self.removed_bird_count = max(
                self.removed_bird_count,
                int(safe_float(fields.get("removed_count", self.removed_bird_count), self.removed_bird_count)),
            )
        self.csvs["bird_removal_events"].row(
            {
                "time_sec": self.now_sec(),
                "state": msg.data,
                "bird_name": fields.get("bird", ""),
                "removed_count": fields.get("removed_count", self.removed_bird_count),
                "remaining_count": fields.get("remaining_count", ""),
            }
        )

    def safety_callback(self, msg: String) -> None:
        self.safety_state = msg.data
        self.csvs["safety_check"].row({"time_sec": self.now_sec(), "state": self.safety_state})

    def image_callback(self, msg: Image) -> None:
        self.note_topic(str(self.get_parameter("image_topic").value), stamp_sec(msg))
        self.image_count += 1
        every = max(1, int(self.get_parameter("save_every_nth_image").value))
        if self.image_count % every != 0:
            return
        image_id = self.image_count
        image_path = ""
        label_path = ""
        if bool(self.get_parameter("save_images").value):
            image = self.image_to_cv2(msg)
            if image is not None:
                image_path = f"images/pt_camera/frame_{image_id:06d}.png"
                cv2.imwrite(str(self.run_dir / image_path), image)
        proj = self.project_gt_bbox(image_id)
        if proj["valid"] and bool(self.get_parameter("write_yolo").value):
            label_path = f"annotations/yolo_labels/frame_{image_id:06d}.txt"
            self.write_yolo_label(self.run_dir / label_path, proj)
        if bool(self.get_parameter("write_coco").value):
            self.append_coco(image_id, msg, image_path, proj)
        self.csvs["frame_index"].row(
            {
                "frame_id": image_id,
                "stamp_sec": stamp_sec(msg),
                "image_path": image_path,
                "label_path": label_path,
                "coco_image_id": image_id,
                "gt_valid": str(proj["valid"]).lower(),
                "bbox_x": proj.get("bbox_x", ""),
                "bbox_y": proj.get("bbox_y", ""),
                "bbox_w": proj.get("bbox_w", ""),
                "bbox_h": proj.get("bbox_h", ""),
                "projection_reason": proj["reason"],
            }
        )
        self.csvs["camera_projection_debug"].row(
            {
                "time_sec": self.now_sec(),
                "image_id": image_id,
                "valid": str(proj["valid"]).lower(),
                "reason": proj["reason"],
                "camera_frame": proj.get("camera_frame", ""),
                "gt_frame": proj.get("gt_frame", ""),
                "points_projected": proj.get("points_projected", 0),
                "bbox_x": proj.get("bbox_x", ""),
                "bbox_y": proj.get("bbox_y", ""),
                "bbox_w": proj.get("bbox_w", ""),
                "bbox_h": proj.get("bbox_h", ""),
            }
        )
        self.write_coco_json()

    def image_to_cv2(self, msg: Image) -> np.ndarray | None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding.lower())
        if channels is None:
            return None
        try:
            image = data.reshape((msg.height, msg.width, channels)) if channels > 1 else data.reshape((msg.height, msg.width))
            if msg.encoding.lower() == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif msg.encoding.lower() == "rgba8":
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif msg.encoding.lower() == "bgra8":
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            return image
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return None

    def project_gt_bbox(self, image_id: int) -> dict[str, Any]:
        if self.camera_info is None:
            if bool(self.get_parameter("require_camera_info").value):
                return {"valid": False, "reason": "CAMERA_INFO_MISSING", "points_projected": 0}
            return {"valid": False, "reason": "CAMERA_INFO_OPTIONAL_MISSING", "points_projected": 0}
        if self.gt_pose is None:
            return {"valid": False, "reason": "GT_MISSING", "points_projected": 0}
        camera_frame = self.camera_info.header.frame_id
        gt_frame = self.gt_pose.header.frame_id
        if not camera_frame or not gt_frame:
            return {"valid": False, "reason": "FRAME_MISSING", "camera_frame": camera_frame, "gt_frame": gt_frame, "points_projected": 0}
        tf = None
        use_odom_fallback = False
        if camera_frame != gt_frame:
            if self.tf_buffer is None:
                use_odom_fallback = bool(self.get_parameter("enable_gazebo_projection_fallback").value)
            else:
                try:
                    tf = self.tf_buffer.lookup_transform(camera_frame, gt_frame, rclpy.time.Time())
                except TransformException as exc:
                    if bool(self.get_parameter("enable_gazebo_projection_fallback").value) and self.odom is not None and gt_frame == "odom":
                        use_odom_fallback = True
                    elif bool(self.get_parameter("require_tf_for_projection").value):
                        return {"valid": False, "reason": f"TF_FAIL {exc}", "camera_frame": camera_frame, "gt_frame": gt_frame, "points_projected": 0}
        dims = [safe_float(v, 1.0) for v in self.get_parameter("bird_bbox_dims_m").value]
        while len(dims) < 3:
            dims.append(1.0)
        center = self.gt_pose.pose.position
        corners = []
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    p = (center.x + sx * dims[0], center.y + sy * dims[1], center.z + sz * dims[2])
                    if tf is not None:
                        corners.append(transform_point(tf, p))
                    elif use_odom_fallback:
                        fallback_point = self.odom_point_to_camera_fallback(p)
                        if fallback_point is not None:
                            corners.append(fallback_point)
                    else:
                        corners.append(p)
        projected = []
        for p in corners:
            uv = self.project_camera_point(p)
            if uv is not None:
                projected.append(uv)
        if len(projected) < 4:
            return {"valid": False, "reason": "TOO_FEW_PROJECTED_CORNERS", "camera_frame": camera_frame, "gt_frame": gt_frame, "points_projected": len(projected)}
        xs = [p[0] for p in projected]
        ys = [p[1] for p in projected]
        width = int(self.camera_info.width)
        height = int(self.camera_info.height)
        x0 = max(0.0, min(xs))
        y0 = max(0.0, min(ys))
        x1 = min(float(width - 1), max(xs))
        y1 = min(float(height - 1), max(ys))
        bbox_w = max(0.0, x1 - x0)
        bbox_h = max(0.0, y1 - y0)
        area = bbox_w * bbox_h
        if area < float(self.get_parameter("min_bbox_area_px").value):
            return {"valid": False, "reason": "BBOX_TOO_SMALL", "camera_frame": camera_frame, "gt_frame": gt_frame, "points_projected": len(projected), "bbox_x": x0, "bbox_y": y0, "bbox_w": bbox_w, "bbox_h": bbox_h}
        reason = "OK_ODOM_CAMERA_FALLBACK" if use_odom_fallback and tf is None else "OK"
        return {"valid": True, "reason": reason, "camera_frame": camera_frame, "gt_frame": gt_frame, "points_projected": len(projected), "bbox_x": x0, "bbox_y": y0, "bbox_w": bbox_w, "bbox_h": bbox_h, "image_w": width, "image_h": height}

    def odom_point_to_camera_fallback(self, p_odom: tuple[float, float, float]) -> tuple[float, float, float] | None:
        if self.odom is None:
            return None
        robot = self.odom.pose.pose
        q = robot.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        dx = p_odom[0] - robot.position.x
        dy = p_odom[1] - robot.position.y
        dz = p_odom[2] - robot.position.z
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        x_base = c * dx - s * dy
        y_base = s * dx + c * dy
        z_base = dz
        mount = [safe_float(v, 0.0) for v in self.get_parameter("camera_mount_xyz_base").value]
        while len(mount) < 3:
            mount.append(0.0)
        return (x_base - mount[0], y_base - mount[1], z_base - mount[2])

    def project_camera_point(self, p: tuple[float, float, float]) -> tuple[float, float] | None:
        x, y, z = p
        convention = str(self.get_parameter("camera_frame_convention").value).strip().lower()
        if convention == "x_forward" or (convention == "auto" and x > 0.05 and not (z > 0.05 and abs(x / z) < 5.0 and abs(y / z) < 5.0)):
            cx, cy, cz = -y, -z, x
        else:
            cx, cy, cz = x, y, z
        if cz <= 0.05:
            return None
        k = self.camera_info.k
        u = k[0] * cx / cz + k[2]
        v = k[4] * cy / cz + k[5]
        if not all(math.isfinite(vv) for vv in (u, v)):
            return None
        return (u, v)

    def write_yolo_label(self, path: Path, proj: dict[str, Any]) -> None:
        width = max(1.0, safe_float(proj.get("image_w", self.camera_info.width if self.camera_info else 1)))
        height = max(1.0, safe_float(proj.get("image_h", self.camera_info.height if self.camera_info else 1)))
        x = safe_float(proj["bbox_x"])
        y = safe_float(proj["bbox_y"])
        w = safe_float(proj["bbox_w"])
        h = safe_float(proj["bbox_h"])
        xc = (x + 0.5 * w) / width
        yc = (y + 0.5 * h) / height
        line = f"0 {xc:.6f} {yc:.6f} {w / width:.6f} {h / height:.6f}\n"
        path.write_text(line, encoding="utf-8")

    def append_coco(self, image_id: int, msg: Image, image_path: str, proj: dict[str, Any]) -> None:
        self.coco_images.append({"id": image_id, "file_name": image_path, "width": int(msg.width), "height": int(msg.height)})
        if proj["valid"]:
            ann_id = len(self.coco_annotations) + 1
            bbox = [safe_float(proj["bbox_x"]), safe_float(proj["bbox_y"]), safe_float(proj["bbox_w"]), safe_float(proj["bbox_h"])]
            self.coco_annotations.append({"id": ann_id, "image_id": image_id, "category_id": 1, "bbox": bbox, "area": bbox[2] * bbox[3], "iscrowd": 0})

    def write_coco_json(self) -> None:
        data = {
            "images": self.coco_images,
            "annotations": self.coco_annotations,
            "categories": [{"id": 1, "name": "bird"}],
            "info": {"detector_mode": self.detector_mode, "classifier_mode": self.classifier_mode},
        }
        (self.run_dir / "annotations/coco_instances.json").write_text(json.dumps(data, indent=2), encoding="utf-8")

    def association(self) -> dict[str, Any]:
        if self.gt_pose is None:
            return {"gt_x": "", "gt_y": "", "gt_z": "", "nearest_dist_m": "", "associated": "false"}
        gt = self.gt_pose
        gx, gy, gz = gt.pose.position.x, gt.pose.position.y, gt.pose.position.z
        best = math.inf
        for lidar in self.lidar_poses:
            target = lidar
            if target.header.frame_id != gt.header.frame_id:
                target = self.transform_pose(target, gt.header.frame_id) or target
            d = math.sqrt(
                (target.pose.position.x - gx) ** 2
                + (target.pose.position.y - gy) ** 2
                + (target.pose.position.z - gz) ** 2
            )
            best = min(best, d)
        associated = best <= float(self.get_parameter("association_radius_m").value)
        return {"gt_x": gx, "gt_y": gy, "gt_z": gz, "nearest_dist_m": "" if best == math.inf else best, "associated": str(associated).lower()}

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped | None:
        if self.tf_buffer is None or pose.header.frame_id == target_frame:
            return pose
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rclpy.time.Time())
        except TransformException:
            return None
        p = pose.pose.position
        out = PoseStamped()
        out.header = pose.header
        out.header.frame_id = target_frame
        out.pose = pose.pose
        x, y, z = transform_point(tf, (p.x, p.y, p.z))
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.position.z = z
        return out

    def note_topic(self, topic: str, stamp: float) -> None:
        now = self.now_sec()
        stat = self.topic_stats.setdefault(topic, {"count": 0.0, "first": now, "last": now, "last_stamp": stamp})
        stat["count"] += 1.0
        stat["last"] = now
        stat["last_stamp"] = stamp

    def periodic_summary(self) -> None:
        self.csvs["experiment_summary"].row(
            {
                "time_sec": self.now_sec(),
                "mission_state": self.mission_state,
                "lidar_state": self.lidar_state,
                "target_class": self.target_class,
                "confidence": self.target_confidence,
                "bird_confirmed": str(self.bird_confirmed).lower(),
                "sound_state": self.sound_state,
                "bird_removal_state": self.bird_removal_state,
                "removed_bird_count": self.removed_bird_count,
                "safety_state": self.safety_state,
                "image_count": self.image_count,
                "gt_used_for_decision": str(self.gt_used_for_decision).lower(),
            }
        )
        self.write_topic_info()
        self.write_metrics()
        self.state_pub.publish(String(data=f"DATASET_LOGGING run_dir={self.run_dir} images={self.image_count} detector_mode={self.detector_mode}"))

    def write_topic_info(self) -> None:
        lines = []
        for name, types in self.get_topic_names_and_types():
            lines.append(f"{name}: {','.join(types)}")
        (self.run_dir / "metadata/topic_info.txt").write_text("\n".join(sorted(lines)) + "\n", encoding="utf-8")
        with open(self.run_dir / "metadata/topic_hz.csv", "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["topic", "count", "duration_sec", "hz", "last_stamp"])
            writer.writeheader()
            for topic, stat in sorted(self.topic_stats.items()):
                duration = max(1e-6, stat["last"] - stat["first"])
                writer.writerow({"topic": topic, "count": int(stat["count"]), "duration_sec": duration, "hz": stat["count"] / duration, "last_stamp": stat["last_stamp"]})

    def write_metrics(self) -> None:
        metrics = {
            "image_count": self.image_count,
            "coco_annotation_count": len(self.coco_annotations),
            "gt_rows": self.csvs["ground_truth_3d"].path.stat().st_size > 0,
            "lidar_detection_count_latest": len(self.lidar_poses),
            "removed_bird_count": self.removed_bird_count,
            "two_bird_removal_success": self.removed_bird_count >= 2,
            "five_bird_removal_success": self.removed_bird_count >= 5,
            "gt_used_for_decision": str(self.gt_used_for_decision).lower(),
            "detector_mode": self.detector_mode,
        }
        path_json = self.run_dir / "metrics/paper_metrics_summary.json"
        path_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        path_csv = self.csvs["paper_metrics_summary"]
        path_csv.handle.seek(0)
        path_csv.handle.truncate()
        path_csv.writer.writeheader()
        for k, v in metrics.items():
            path_csv.row({"metric": k, "value": v})

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> bool:
        try:
            self.write_topic_info()
            self.write_metrics()
            self.write_coco_json()
        except Exception:
            pass
        for csv_file in getattr(self, "csvs", {}).values():
            try:
                csv_file.handle.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboBirdDatasetLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

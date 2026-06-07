from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass
class YoloDetection:
    class_name: str
    confidence: float
    center_x: float = 0.0
    center_y: float = 0.0
    size_x: float = 0.0
    size_y: float = 0.0
    track_id: str = ""


class ExternalYoloBridgeNode(Node):
    """Adapt external YOLO results into the standard Waver bird topics."""

    def __init__(self) -> None:
        super().__init__("external_yolo_bridge_node")
        self.declare_parameter("input_detection_topic", "/external/yolo/detections")
        self.declare_parameter("input_json_topic", "/external/yolo/result_json")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("bird_class_names", ["bird"])
        self.declare_parameter("non_bird_class_names", ["person", "vehicle", "drone", "robot"])
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("require_camera_centered_for_mission", True)
        self.declare_parameter("camera_centered_topic", "/waver/camera_target_centered")

        self.detections_pub = self.create_publisher(Detection2DArray, "/waver/bird_detections_2d", 10)
        self.confirmed_pub = self.create_publisher(Bool, "/waver/bird_confirmed", 10)
        self.class_pub = self.create_publisher(String, "/waver/target_class", 10)
        self.conf_pub = self.create_publisher(Float32, "/waver/target_confidence", 10)
        self.state_pub = self.create_publisher(String, "/waver/target_classification_state", 10)
        self.latency_pub = self.create_publisher(Float32, "/waver/target_classification_latency_ms", 10)

        self.camera_centered = False
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter("input_detection_topic").value),
            self.detections_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("input_json_topic").value),
            self.json_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("camera_centered_topic").value),
            lambda m: setattr(self, "camera_centered", bool(m.data)),
            10,
        )

    def json_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_classification("unknown", 0.0, False, f"INVALID_JSON {exc}", 0.0)
            return
        detections = self.detections_from_json(payload)
        array = self.to_detection_array(detections)
        self.process_detection_array(array, start_time=self._extract_start_time(payload))

    def detections_callback(self, msg: Detection2DArray) -> None:
        self.process_detection_array(msg)

    def process_detection_array(self, msg: Detection2DArray, start_time: float | None = None) -> None:
        self.detections_pub.publish(msg)
        best_class, best_conf = self.best_classification(msg)
        bird = self.is_bird(best_class, best_conf)
        if not best_class:
            best_class = "unknown"
        state = "BIRD" if bird else ("NON_BIRD" if best_class not in {"unknown", "none"} else "UNKNOWN")
        if bool(self.get_parameter("require_camera_centered_for_mission").value) and not self.camera_centered:
            state = f"{state} CAMERA_NOT_CENTERED"
        latency_ms = self.latency_ms(msg, start_time)
        self.publish_classification(best_class, best_conf, bird, state, latency_ms)

    def detections_from_json(self, payload: dict) -> list[YoloDetection]:
        raw = payload.get("detections", payload.get("objects", []))
        if isinstance(raw, dict):
            raw = [raw]
        detections: list[YoloDetection] = []
        for item in raw if isinstance(raw, list) else []:
            if not isinstance(item, dict):
                continue
            bbox = item.get("bbox", item)
            detections.append(
                YoloDetection(
                    class_name=str(item.get("class_name", item.get("class", item.get("label", "unknown")))),
                    confidence=float(item.get("confidence", item.get("score", 0.0))),
                    center_x=float(bbox.get("center_x", bbox.get("cx", bbox.get("x", 0.0)))),
                    center_y=float(bbox.get("center_y", bbox.get("cy", bbox.get("y", 0.0)))),
                    size_x=float(bbox.get("size_x", bbox.get("w", bbox.get("width", 0.0)))),
                    size_y=float(bbox.get("size_y", bbox.get("h", bbox.get("height", 0.0)))),
                    track_id=str(item.get("track_id", item.get("id", ""))),
                )
            )
        return detections

    def to_detection_array(self, detections: list[YoloDetection]) -> Detection2DArray:
        array = Detection2DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = str(self.get_parameter("camera_frame").value)
        for detection in detections:
            det = Detection2D()
            det.header = array.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float(detection.center_x)
            det.bbox.center.position.y = float(detection.center_y)
            det.bbox.size_x = float(detection.size_x)
            det.bbox.size_y = float(detection.size_y)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = detection.class_name
            hyp.hypothesis.score = float(detection.confidence)
            det.results.append(hyp)
            array.detections.append(det)
        return array

    def best_classification(self, msg: Detection2DArray) -> tuple[str, float]:
        best_class = "unknown"
        best_conf = 0.0
        for det in msg.detections:
            for result in det.results:
                score = float(result.hypothesis.score)
                if score > best_conf:
                    best_conf = score
                    best_class = str(result.hypothesis.class_id).strip().lower() or "unknown"
        return best_class, best_conf

    def is_bird(self, class_name: str, confidence: float) -> bool:
        birds = {str(v).strip().lower() for v in self.get_parameter("bird_class_names").value}
        threshold = float(self.get_parameter("confidence_threshold").value)
        return class_name.strip().lower() in birds and confidence >= threshold

    def publish_classification(self, target_class: str, confidence: float, bird: bool, state: str, latency_ms: float) -> None:
        self.confirmed_pub.publish(Bool(data=bool(bird)))
        self.class_pub.publish(String(data=target_class if target_class else "unknown"))
        self.conf_pub.publish(Float32(data=float(confidence)))
        self.state_pub.publish(String(data=state))
        self.latency_pub.publish(Float32(data=float(latency_ms)))

    def latency_ms(self, msg: Detection2DArray, start_time: float | None = None) -> float:
        if start_time is not None:
            return max(0.0, (time.time() - start_time) * 1000.0)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            return 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, (now - stamp) * 1000.0)

    @staticmethod
    def _extract_start_time(payload: dict) -> float | None:
        for key in ("start_time", "image_time", "timestamp"):
            value = payload.get(key)
            if isinstance(value, (int, float)) and math.isfinite(value):
                return float(value)
        return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExternalYoloBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

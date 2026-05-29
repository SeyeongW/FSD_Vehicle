from __future__ import annotations

import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


class BirdDetectorNode(Node):
    """Camera bird detector wrapper.

    Real vehicle policy:
      - missing model or camera stale => bird_confirmed=false
      - mock backend is rejected when real_profile=true
      - this node never publishes navigation goals or /cmd_vel
    """

    def __init__(self) -> None:
        super().__init__("bird_detector_node")
        self.declare_parameter("real_profile", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("detections_topic", "/waver/bird_detections_2d")
        self.declare_parameter("confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("state_topic", "/waver/bird_detector_state")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("target_confidence_topic", "/waver/target_confidence")
        self.declare_parameter("target_classification_state_topic", "/waver/target_classification_state")
        self.declare_parameter("target_classification_latency_topic", "/waver/target_classification_latency_ms")
        self.declare_parameter("backend", "yolo")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("bird_class_names", ["bird"])
        self.declare_parameter("class_names_of_interest", ["bird", "drone"])
        self.declare_parameter("deterrence_class_names", ["bird"])
        self.declare_parameter("unknown_confidence_threshold", 0.50)
        self.declare_parameter(
            "class_aliases",
            ["airplane:irrelevant", "kite:unknown", "bird:bird", "drone:drone", "uav:drone"],
        )
        self.declare_parameter("publish_unknown_when_no_detection", True)
        self.declare_parameter("classification_requires_camera_alignment", True)
        self.declare_parameter("camera_centered_topic", "/waver/camera_target_centered")
        self.declare_parameter("nof_m_window", 5)
        self.declare_parameter("nof_m_required", 3)
        self.declare_parameter("camera_stale_sec", 1.0)
        self.declare_parameter("min_bbox_area_fraction", 0.001)
        self.declare_parameter("max_bbox_area_fraction", 0.8)
        self.declare_parameter("edge_margin_fraction", 0.05)
        self.declare_parameter("max_inference_hz", 10.0)

        backend = str(self.get_parameter("backend").value).strip().lower()
        if bool(self.get_parameter("real_profile").value) and backend == "mock_for_sim_only":
            raise RuntimeError("bird_detector_node: mock_for_sim_only backend is forbidden in real_profile")

        self.detections_pub = self.create_publisher(Detection2DArray, str(self.get_parameter("detections_topic").value), 10)
        self.confirmed_pub = self.create_publisher(Bool, str(self.get_parameter("confirmed_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.target_class_pub = self.create_publisher(String, str(self.get_parameter("target_class_topic").value), 10)
        self.target_confidence_pub = self.create_publisher(Float32, str(self.get_parameter("target_confidence_topic").value), 10)
        self.classification_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("target_classification_state_topic").value),
            10,
        )
        self.latency_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("target_classification_latency_topic").value),
            10,
        )

        self.camera_info: CameraInfo | None = None
        self.camera_centered = False
        self.last_image_time = 0.0
        self.last_inference_time = 0.0
        self.confirm_window: deque[bool] = deque(maxlen=max(int(self.get_parameter("nof_m_window").value), 1))
        self.model = None
        self.bridge = None
        self.model_error = ""
        self._load_backend()

        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.camera_info_callback, 10)
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self.image_callback, 5)
        self.create_subscription(
            Bool,
            str(self.get_parameter("camera_centered_topic").value),
            lambda m: setattr(self, "camera_centered", bool(m.data)),
            10,
        )
        self.create_timer(0.2, self.health_tick)

    def _load_backend(self) -> None:
        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            self.model_error = "MODEL_MISSING"
            return
        if not Path(model_path).exists():
            self.model_error = f"MODEL_NOT_FOUND path={model_path}"
            return
        backend = str(self.get_parameter("backend").value).strip().lower()
        if backend != "yolo":
            self.model_error = f"BACKEND_UNSUPPORTED backend={backend}"
            return
        try:
            from cv_bridge import CvBridge
            from ultralytics import YOLO

            self.bridge = CvBridge()
            self.model = YOLO(model_path)
            self.model_error = ""
        except Exception as exc:  # pragma: no cover - hardware/model dependent
            self.model = None
            self.bridge = None
            self.model_error = f"INFERENCE_BACKEND_UNAVAILABLE error={exc}"

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def image_callback(self, msg: Image) -> None:
        now = self._now()
        self.last_image_time = now
        if now - self.last_inference_time < 1.0 / max(float(self.get_parameter("max_inference_hz").value), 0.1):
            return
        self.last_inference_time = now
        if bool(self.get_parameter("classification_requires_camera_alignment").value) and not self.camera_centered:
            self._publish_empty(msg, "WAIT_CAMERA_ALIGNMENT", force_class="none")
            return
        if self.model is None or self.bridge is None:
            self._publish_empty(msg, self.model_error or "MODEL_MISSING")
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            inference_start = self._now()
            detections = self._run_yolo(cv_image, msg)
            latency_ms = max((self._now() - inference_start) * 1000.0, 0.0)
            target_class, best_confidence = self._best_classification(detections)
            if not detections.detections and bool(self.get_parameter("publish_unknown_when_no_detection").value):
                target_class = "unknown"
                best_confidence = 0.0
            unknown_threshold = float(self.get_parameter("unknown_confidence_threshold").value)
            if best_confidence < unknown_threshold and target_class not in {"unknown", "none"}:
                target_class = "unknown"
            confirmed = self._update_confirmation(target_class == "bird" and best_confidence >= float(self.get_parameter("confidence_threshold").value))
            self.detections_pub.publish(detections)
            self.confirmed_pub.publish(Bool(data=confirmed))
            self.target_class_pub.publish(String(data=target_class))
            self.target_confidence_pub.publish(Float32(data=float(best_confidence)))
            self.latency_pub.publish(Float32(data=float(latency_ms)))
            state = (
                f"CLASSIFIED target_class={target_class} confidence={best_confidence:.3f} "
                f"bird_confirmed={confirmed} detections={len(detections.detections)} latency_ms={latency_ms:.2f}"
            )
            self.state_pub.publish(
                String(data=state)
            )
            self.classification_state_pub.publish(String(data=state))
        except Exception as exc:  # pragma: no cover - hardware/model dependent
            self._publish_empty(msg, f"INFERENCE_ERROR error={exc}")

    def _run_yolo(self, cv_image, msg: Image) -> Detection2DArray:
        aliases = self.class_aliases()
        conf_threshold = float(self.get_parameter("confidence_threshold").value)
        min_area = float(self.get_parameter("min_bbox_area_fraction").value)
        max_area = float(self.get_parameter("max_bbox_area_fraction").value)
        edge = float(self.get_parameter("edge_margin_fraction").value)
        image_area = max(float(msg.width * msg.height), 1.0)
        output = Detection2DArray()
        output.header = msg.header
        results = self.model.predict(cv_image, verbose=False, conf=conf_threshold)
        for result in results:
            names = getattr(result, "names", {}) or {}
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0]) if getattr(box, "cls", None) is not None else -1
                class_name = str(names.get(cls_id, cls_id)).lower()
                normalized = aliases.get(class_name, class_name)
                score = float(box.conf[0]) if getattr(box, "conf", None) is not None else 0.0
                if normalized not in {"bird", "drone", "unknown", "irrelevant"} or score < conf_threshold:
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                w = max(x2 - x1, 0.0)
                h = max(y2 - y1, 0.0)
                area_fraction = (w * h) / image_area
                if area_fraction < min_area or area_fraction > max_area:
                    continue
                if x1 < edge * msg.width or y1 < edge * msg.height:
                    continue
                if x2 > (1.0 - edge) * msg.width or y2 > (1.0 - edge) * msg.height:
                    continue
                det = Detection2D()
                det.header = msg.header
                det.bbox = BoundingBox2D()
                det.bbox.center.x = (x1 + x2) * 0.5
                det.bbox.center.y = (y1 + y2) * 0.5
                det.bbox.size_x = w
                det.bbox.size_y = h
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = normalized
                hyp.hypothesis.score = score
                det.results.append(hyp)
                output.detections.append(det)
        return output

    def _publish_empty(self, image_msg: Image | None, state: str, *, force_class: str | None = None) -> None:
        out = Detection2DArray()
        if image_msg is not None:
            out.header = image_msg.header
        self.detections_pub.publish(out)
        self.confirm_window.append(False)
        self.confirmed_pub.publish(Bool(data=False))
        target_class = force_class
        if target_class is None:
            target_class = "unknown" if self.camera_centered and bool(self.get_parameter("publish_unknown_when_no_detection").value) else "none"
        self.target_class_pub.publish(String(data=target_class))
        self.target_confidence_pub.publish(Float32(data=0.0))
        self.latency_pub.publish(Float32(data=0.0))
        state_text = f"{state} target_class={target_class} bird_confirmed=false"
        self.state_pub.publish(String(data=state_text))
        self.classification_state_pub.publish(String(data=state_text))

    @staticmethod
    def _best_confidence(detections: Detection2DArray) -> float:
        best = 0.0
        for det in detections.detections:
            for result in det.results:
                best = max(best, float(result.hypothesis.score))
        return best

    @staticmethod
    def _best_classification(detections: Detection2DArray) -> tuple[str, float]:
        best_class = "none"
        best_score = 0.0
        for det in detections.detections:
            for result in det.results:
                score = float(result.hypothesis.score)
                if score >= best_score:
                    best_score = score
                    best_class = str(result.hypothesis.class_id).strip().lower() or "unknown"
        return best_class, best_score

    def class_aliases(self) -> dict[str, str]:
        aliases: dict[str, str] = {}
        for item in self.get_parameter("class_aliases").value:
            text = str(item).strip()
            if not text:
                continue
            if ":" in text:
                src, dst = text.split(":", 1)
            elif "=" in text:
                src, dst = text.split("=", 1)
            else:
                continue
            aliases[src.strip().lower()] = dst.strip().lower()
        for name in self.get_parameter("bird_class_names").value:
            aliases.setdefault(str(name).strip().lower(), "bird")
        aliases.setdefault("bird", "bird")
        aliases.setdefault("drone", "drone")
        aliases.setdefault("uav", "drone")
        return aliases

    def _update_confirmation(self, detected: bool) -> bool:
        self.confirm_window.append(bool(detected))
        required = max(int(self.get_parameter("nof_m_required").value), 1)
        return sum(1 for value in self.confirm_window if value) >= required

    def health_tick(self) -> None:
        if self.last_image_time == 0.0:
            self.confirmed_pub.publish(Bool(data=False))
            self.target_class_pub.publish(String(data="none"))
            self.target_confidence_pub.publish(Float32(data=0.0))
            state = "CAMERA_WAITING target_class=none bird_confirmed=false"
            self.state_pub.publish(String(data=state))
            self.classification_state_pub.publish(String(data=state))
            return
        if self._now() - self.last_image_time > float(self.get_parameter("camera_stale_sec").value):
            self.confirm_window.clear()
            self.confirmed_pub.publish(Bool(data=False))
            self.target_class_pub.publish(String(data="none"))
            self.target_confidence_pub.publish(Float32(data=0.0))
            state = "CAMERA_STALE target_class=none bird_confirmed=false"
            self.state_pub.publish(String(data=state))
            self.classification_state_pub.publish(String(data=state))

    def _now(self) -> float:
        return time.monotonic()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BirdDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

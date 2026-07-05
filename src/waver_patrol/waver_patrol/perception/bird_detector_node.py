from __future__ import annotations

import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from waver_patrol.perception.bird_classification import detector_model_state, normalize_class_name


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
        self.declare_parameter("bbox_center_error_topic", "/waver/camera_bbox_center_error_px")
        self.declare_parameter("bbox_center_error_signed_topic", "/waver/camera_bbox_center_error_signed_px")
        self.declare_parameter("backend", "yolo")
        self.declare_parameter("bird_backend", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("bird_model_path", "")
        self.declare_parameter("detector_required_for_real", True)
        self.declare_parameter("max_detector_latency_sec", 1.0)
        self.declare_parameter("accepted_bird_classes", ["bird"])
        self.declare_parameter("non_bird_classes", ["person", "vehicle", "robot", "drone", "irrelevant", "none"])
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("bird_class_names", ["bird"])
        self.declare_parameter("class_names_of_interest", ["bird", "drone"])
        self.declare_parameter("deterrence_class_names", ["bird"])
        self.declare_parameter("class_map_required", ["bird", "person", "vehicle", "drone", "unknown"])
        self.declare_parameter("unknown_confidence_threshold", 0.50)
        self.declare_parameter(
            "class_aliases",
            [
                "airplane:irrelevant",
                "kite:unknown",
                "bird:bird",
                "person:person",
                "human:person",
                "vehicle:vehicle",
                "car:vehicle",
                "truck:vehicle",
                "robot:vehicle",
                "drone:drone",
                "uav:drone",
                "unknown:unknown",
            ],
        )
        self.declare_parameter("publish_unknown_when_no_detection", True)
        self.declare_parameter("classification_requires_camera_alignment", False)
        self.declare_parameter("camera_centered_topic", "/waver/camera_target_centered")
        self.declare_parameter("nof_m_window", 5)
        self.declare_parameter("nof_m_required", 3)
        self.declare_parameter("camera_stale_sec", 1.0)
        self.declare_parameter("min_bbox_area_fraction", 0.001)
        self.declare_parameter("max_bbox_area_fraction", 0.8)
        self.declare_parameter("edge_margin_fraction", 0.05)
        self.declare_parameter("max_inference_hz", 10.0)

        backend_alias = str(self.get_parameter("bird_backend").value).strip().lower()
        backend = backend_alias or str(self.get_parameter("backend").value).strip().lower()
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
        self.bbox_error_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("bbox_center_error_topic").value),
            10,
        )
        self.bbox_error_signed_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("bbox_center_error_signed_topic").value),
            10,
        )

        self.camera_info: CameraInfo | None = None
        self.camera_info_time = 0.0
        self.camera_centered = False
        self.last_image_time = 0.0
        self.last_inference_time = 0.0
        self.last_latency_ms = 0.0
        self.last_inference_state = "READY"
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
            model_path = str(self.get_parameter("bird_model_path").value).strip()
        state = detector_model_state(model_path, required=bool(self.get_parameter("detector_required_for_real").value))
        if state != "MODEL_READY":
            self.model_error = state
            return
        backend_alias = str(self.get_parameter("bird_backend").value).strip().lower()
        backend = backend_alias or str(self.get_parameter("backend").value).strip().lower()
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
        self.camera_info_time = self._now()

    def image_callback(self, msg: Image) -> None:
        now = self._now()
        self.last_image_time = now
        if now - self.last_inference_time < 1.0 / max(float(self.get_parameter("max_inference_hz").value), 0.1):
            self.last_inference_state = "THROTTLED"
            return
        self.last_inference_time = now
        if bool(self.get_parameter("classification_requires_camera_alignment").value) and not self.camera_centered:
            self.last_inference_state = "READY"
            self._publish_empty(msg, "WAIT_CAMERA_ALIGNMENT", force_class="none")
            return
        if self.model is None or self.bridge is None:
            self.last_inference_state = "ERROR"
            self._publish_empty(msg, self.model_error or "MODEL_MISSING")
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            inference_start = self._now()
            detections = self._run_yolo(cv_image, msg)
            latency_ms = max((self._now() - inference_start) * 1000.0, 0.0)
            self.last_latency_ms = latency_ms
            self.last_inference_state = "READY"
            target_class, best_confidence = self._best_classification(detections)
            if not detections.detections:
                force_class = "unknown" if bool(self.get_parameter("publish_unknown_when_no_detection").value) else "none"
                self._publish_empty(msg, "CAMERA_NO_TARGET", force_class=force_class)
                return
            self._publish_bbox_center_error(detections, msg)
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
                self._state_text(
                    "CLASSIFIED",
                    target_class=target_class,
                    confidence=best_confidence,
                    bird_confirmed=confirmed,
                    detections=len(detections.detections),
                )
            )
            self.state_pub.publish(
                String(data=state)
            )
            self.classification_state_pub.publish(String(data=state))
        except Exception as exc:  # pragma: no cover - hardware/model dependent
            self.last_inference_state = "ERROR"
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
                normalized = normalize_class_name(class_name, aliases)
                score = float(box.conf[0]) if getattr(box, "conf", None) is not None else 0.0
                if normalized not in {"bird", "person", "vehicle", "drone", "unknown", "irrelevant"} or score < conf_threshold:
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
        state_text = self._state_text(state, target_class=target_class, confidence=0.0, bird_confirmed=False, detections=0)
        self.state_pub.publish(String(data=state_text))
        self.classification_state_pub.publish(String(data=state_text))

    def _model_state(self) -> str:
        if self.model is not None and self.bridge is not None and not self.model_error:
            return "MODEL_READY" if self._class_map_ok() else "MODEL_LOAD_ERROR"
        text = (self.model_error or "MODEL_MISSING").upper()
        if "MODEL_NOT_FOUND" in text:
            return "MODEL_NOT_FOUND"
        if "MODEL_MISSING" in text or "MODEL_PATH_EMPTY" in text:
            return "MODEL_MISSING"
        if "INFERENCE_BACKEND_UNAVAILABLE" in text:
            return "INFERENCE_BACKEND_UNAVAILABLE"
        return "MODEL_LOAD_ERROR"

    def _camera_state(self) -> str:
        if self.camera_info is None:
            return "CAMERA_INFO_MISSING"
        if self.last_image_time == 0.0 or self._now() - self.last_image_time > float(self.get_parameter("camera_stale_sec").value):
            return "CAMERA_STALE"
        return "CAMERA_OK"

    def _class_map_ok(self) -> bool:
        aliases = self.class_aliases()
        values = {str(v).strip().lower() for v in aliases.values()}
        required = {str(v).strip().lower() for v in self.get_parameter("class_map_required").value if str(v).strip()}
        return required.issubset(values)

    def _publish_bbox_center_error(self, detections: Detection2DArray, msg: Image) -> None:
        best_det: Detection2D | None = None
        best_score = -1.0
        for det in detections.detections:
            for result in det.results:
                score = float(result.hypothesis.score)
                if score > best_score:
                    best_score = score
                    best_det = det
        if best_det is None or msg.width <= 0:
            return
        signed_error_px = float(best_det.bbox.center.x) - float(msg.width) * 0.5
        self.bbox_error_signed_pub.publish(Float32(data=signed_error_px))
        self.bbox_error_pub.publish(Float32(data=abs(signed_error_px)))

    def _state_text(
        self,
        prefix: str,
        *,
        target_class: str = "none",
        confidence: float = 0.0,
        bird_confirmed: bool = False,
        detections: int = 0,
    ) -> str:
        fps = 0.0 if self.last_image_time == 0.0 else min(float(self.get_parameter("max_inference_hz").value), 999.0)
        return (
            f"{prefix} model_state={self._model_state()} camera_state={self._camera_state()} "
            f"inference_state={self.last_inference_state} latency_ms={self.last_latency_ms:.2f} fps={fps:.2f} "
            f"class_map_ok={str(self._class_map_ok()).lower()} target_class={target_class} "
            f"confidence={float(confidence):.3f} bird_confirmed={str(bool(bird_confirmed)).lower()} detections={int(detections)}"
        )

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
        aliases.setdefault("person", "person")
        aliases.setdefault("human", "person")
        aliases.setdefault("vehicle", "vehicle")
        aliases.setdefault("car", "vehicle")
        aliases.setdefault("truck", "vehicle")
        aliases.setdefault("robot", "vehicle")
        aliases.setdefault("drone", "drone")
        aliases.setdefault("uav", "drone")
        aliases.setdefault("unknown", "unknown")
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
            state = self._state_text("CAMERA_WAITING", target_class="none", bird_confirmed=False)
            self.state_pub.publish(String(data=state))
            self.classification_state_pub.publish(String(data=state))
            return
        if self._now() - self.last_image_time > float(self.get_parameter("camera_stale_sec").value):
            self.confirm_window.clear()
            self.confirmed_pub.publish(Bool(data=False))
            self.target_class_pub.publish(String(data="none"))
            self.target_confidence_pub.publish(Float32(data=0.0))
            state = self._state_text("CAMERA_STALE", target_class="none", bird_confirmed=False)
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

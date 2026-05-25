"""YOLO object detection node.

Prefers Hailo 8 (HailoRT) when available; falls back to CPU ultralytics so
the same node runs on the dev PC and on the Pi.
"""
from __future__ import annotations

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

try:
    from hailo_platform import (HEF, VDevice, FormatType, HailoStreamInterface,
                                InferVStreams, ConfigureParams,
                                InputVStreamParams, OutputVStreamParams)
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class HailoYoloBackend:
    """Minimal HailoRT inference wrapper for a YOLOv8-class .hef.

    Returns detections as a numpy array of shape (N, 6): [x1, y1, x2, y2, score, class_id].
    Coordinates are in the *input* image frame (the caller is responsible
    for letterbox/scale handling).
    """

    def __init__(self, hef_path: str, conf_thresh: float = 0.4):
        self.conf_thresh = conf_thresh
        self.hef = HEF(hef_path)
        self.vdevice = VDevice()
        cfg_params = ConfigureParams.create_from_hef(
            hef=self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.vdevice.configure(self.hef, cfg_params)[0]
        self.ng_params = self.network_group.create_params()
        in_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
        self.input_vstream_info = self.hef.get_input_vstream_infos()[0]
        h, w, _ = self.input_vstream_info.shape
        self.input_size = (w, h)
        self.streams = InferVStreams(self.network_group, in_params, out_params)

    def preprocess(self, bgr: np.ndarray):
        w, h = self.input_size
        resized = cv2.resize(bgr, (w, h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return np.expand_dims(rgb, axis=0).astype(np.uint8), (bgr.shape[1] / w, bgr.shape[0] / h)

    def infer(self, bgr: np.ndarray):
        tensor, (sx, sy) = self.preprocess(bgr)
        with self.streams as pipeline, self.network_group.activate(self.ng_params):
            raw = pipeline.infer({self.input_vstream_info.name: tensor})
        # The output layout depends on the compiled .hef. For yolov8n-style
        # outputs we expect a (1, N, 6) tensor of [x1, y1, x2, y2, score, cls].
        # If the model has a different layout, replace this block.
        out = next(iter(raw.values()))
        out = np.array(out).reshape(-1, 6)
        out = out[out[:, 4] >= self.conf_thresh]
        if out.size:
            out[:, [0, 2]] *= sx
            out[:, [1, 3]] *= sy
        return out


class UltralyticsBackend:
    def __init__(self, model_path: str, conf_thresh: float = 0.4):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

    def infer(self, bgr: np.ndarray):
        results = self.model(bgr, conf=self.conf_thresh, verbose=False)
        if not results:
            return np.zeros((0, 6), dtype=np.float32)
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return np.zeros((0, 6), dtype=np.float32)
        xyxy = r.boxes.xyxy.cpu().numpy()
        scores = r.boxes.conf.cpu().numpy().reshape(-1, 1)
        cls = r.boxes.cls.cpu().numpy().reshape(-1, 1)
        return np.hstack([xyxy, scores, cls]).astype(np.float32)


class YoloHailoNode(Node):
    def __init__(self):
        super().__init__('yolo_hailo_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('detections_topic', '/camera/detections')
        self.declare_parameter('hef_path', '/ros2_ws/ugv_ws/models/yolov8n.hef')
        self.declare_parameter('cpu_model_path', 'yolov8n.pt')
        self.declare_parameter('conf_thresh', 0.4)
        self.declare_parameter('target_classes', [0])  # COCO id 0 = person
        self.declare_parameter('force_cpu', False)

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        hef_path = self.get_parameter('hef_path').value
        cpu_model = self.get_parameter('cpu_model_path').value
        conf = float(self.get_parameter('conf_thresh').value)
        self.target_classes = set(int(c) for c in self.get_parameter('target_classes').value)
        force_cpu = bool(self.get_parameter('force_cpu').value)

        self.backend = self._select_backend(hef_path, cpu_model, conf, force_cpu)

        self.sub = self.create_subscription(CompressedImage, image_topic, self.image_cb, 5)
        self.pub = self.create_publisher(Detection2DArray, detections_topic, 5)

        self._last_log = 0.0
        self.get_logger().info(
            f'YOLO node ready (backend={type(self.backend).__name__}, '
            f'in={image_topic}, out={detections_topic})')

    def _select_backend(self, hef_path, cpu_model, conf, force_cpu):
        if not force_cpu and HAILO_AVAILABLE:
            try:
                return HailoYoloBackend(hef_path, conf)
            except Exception as e:
                self.get_logger().warn(f'Hailo init failed ({e}); falling back to CPU')
        if not ULTRALYTICS_AVAILABLE:
            raise RuntimeError('Neither HailoRT nor ultralytics is available')
        return UltralyticsBackend(cpu_model, conf)

    def image_cb(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        t0 = time.time()
        dets = self.backend.infer(bgr)
        dt = time.time() - t0

        out = Detection2DArray()
        out.header = msg.header
        h, w = bgr.shape[:2]
        for x1, y1, x2, y2, score, cls in dets:
            cls_id = int(cls)
            if self.target_classes and cls_id not in self.target_classes:
                continue
            d = Detection2D()
            d.header = msg.header
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(max(x2 - x1, 1.0))
            d.bbox.size_y = float(max(y2 - y1, 1.0))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = float(score)
            d.results.append(hyp)
            out.detections.append(d)

        self.pub.publish(out)

        now = time.time()
        if now - self._last_log > 2.0:
            self.get_logger().info(
                f'frame {w}x{h} -> {len(out.detections)} dets in {dt*1000:.1f} ms')
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloHailoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

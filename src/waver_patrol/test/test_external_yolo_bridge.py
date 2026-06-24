import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.ros_required

import rclpy

from waver_patrol.perception.external_yolo_bridge_node import ExternalYoloBridgeNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_yolo_bridge_bird_threshold_and_bbox():
    node = ExternalYoloBridgeNode()
    try:
        detections = node.detections_from_json(
            {"detections": [{"class_name": "bird", "confidence": 0.9, "bbox": {"cx": 10, "cy": 20, "w": 30, "h": 40}}]}
        )
        array = node.to_detection_array(detections)
        klass, conf = node.best_classification(array)
        assert klass == "bird"
        assert conf == pytest.approx(0.9)
        assert node.is_bird(klass, conf)
        assert array.detections[0].bbox.center.position.x == pytest.approx(10.0)
        assert array.detections[0].bbox.size_x == pytest.approx(30.0)
    finally:
        node.destroy_node()


def test_yolo_bridge_non_bird_false():
    node = ExternalYoloBridgeNode()
    try:
        detections = node.detections_from_json({"detections": [{"class": "person", "score": 0.95}]})
        array = node.to_detection_array(detections)
        klass, conf = node.best_classification(array)
        assert klass == "person"
        assert conf == pytest.approx(0.95)
        assert not node.is_bird(klass, conf)
    finally:
        node.destroy_node()

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_detector_publishes_bbox_error_for_robot_body_alignment():
    detector = (ROOT / "src/waver_patrol/waver_patrol/perception/bird_detector_node.py").read_text()
    gimbal = (ROOT / "src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py").read_text()

    assert "bbox_center_error_topic" in detector
    assert "/waver/camera_bbox_center_error_px" in detector
    assert "bbox_center_error_signed_topic" in detector
    assert "_publish_bbox_center_error" in detector
    assert "create_publisher(Float32" in detector
    assert "bbox_center_error_topic" in gimbal
    assert "ALIGNMENT_CENTERED_CONFIRMED" in gimbal
    assert "topic_only" in gimbal and "ALLOW" not in "SIMULATED_ALIGNMENT TOPIC_ONLY real_gimbal_output_enabled=false centered=true"

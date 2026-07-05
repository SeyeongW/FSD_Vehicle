from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_detector_does_not_require_camera_alignment_by_default():
    text = (ROOT / "src/waver_patrol/waver_patrol/perception/bird_detector_node.py").read_text()
    assert 'self.declare_parameter("classification_requires_camera_alignment", False)' in text
    assert "/waver/camera_bbox_center_error_px" in text
    assert "/waver/camera_bbox_center_error_signed_px" in text


def test_mission_sound_gate_still_requires_centered_fusion_dynamic_bird():
    text = (ROOT / "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py").read_text()
    for token in (
        "camera_not_centered",
        "sound_alert_requested class=bird",
        "bird_confirmed",
        "fusion_valid",
        "dynamic_valid",
    ):
        assert token in text

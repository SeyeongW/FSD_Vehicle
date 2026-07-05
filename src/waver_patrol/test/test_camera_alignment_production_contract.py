from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / "src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py"


def test_topic_only_production_cannot_claim_centered():
    text = NODE.read_text()
    assert "production_profile" in text
    assert "allow_simulated_centered" in text
    assert "ALIGNMENT_SIMULATED_ONLY" in text
    assert "ALIGNMENT_CENTERED_CONFIRMED" in text
    assert "bbox_centered" in text
    assert "topic_only" in text
    assert "SIMULATED_ALIGNMENT TOPIC_ONLY real_gimbal_output_enabled=false centered=true" not in text

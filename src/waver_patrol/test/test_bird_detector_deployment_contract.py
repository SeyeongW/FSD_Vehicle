from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / "src/waver_patrol/waver_patrol/perception/bird_detector_node.py"


def test_detector_fail_closed_state_is_machine_readable():
    text = NODE.read_text()
    for token in (
        "model_state=",
        "camera_state=",
        "inference_state=",
        "latency_ms=",
        "class_map_ok=",
        "MODEL_MISSING",
        "INFERENCE_BACKEND_UNAVAILABLE",
    ):
        assert token in text
    assert "create_publisher(Twist" not in text
    assert '"/cmd_vel"' not in text


def test_perception_requirements_do_not_force_generic_torch_on_jetson():
    req = (ROOT / "requirements-jetson-perception.txt").read_text()
    assert "ultralytics" in req
    assert "opencv-python-headless" in req
    assert "torch==" not in req

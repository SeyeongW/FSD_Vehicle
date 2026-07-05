from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_detector_probe_requires_warmup_class_map_and_latency():
    probe = (ROOT / "scripts/waver_bird_detector_probe.py").read_text()
    detector = (ROOT / "src/waver_patrol/waver_patrol/perception/bird_detector_node.py").read_text()

    for token in ("warmup_yolo", "warmup_inference_ok", "mean_latency_ms", "max_latency_ms", "class_map_ok"):
        assert token in probe
    for required in ("bird", "person", "vehicle", "drone", "unknown"):
        assert required in detector
    assert '"/cmd_vel"' not in detector

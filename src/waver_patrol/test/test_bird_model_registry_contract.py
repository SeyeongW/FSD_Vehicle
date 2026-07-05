from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_bird_model_registry_has_default_safe_class_policy():
    data = yaml.safe_load((ROOT / "config/perception/bird_model_registry.yaml").read_text())
    model = data["models"]["default"]
    assert model["backend"] == "ultralytics"
    assert "bird" in model["expected_classes"]
    assert "person" in model["expected_classes"]
    assert "vehicle" in model["expected_classes"]
    assert "drone" in model["expected_classes"]
    assert model["fallback_unknown_class"] is True
    assert model["max_latency_ms"] <= 300


def test_bird_detector_probe_reads_registry_contract():
    text = (ROOT / "scripts/waver_bird_detector_probe.py").read_text()
    assert "--registry" in text
    assert "--model-name" in text
    assert "expected_classes" in text
    assert "fallback_unknown_class" in text

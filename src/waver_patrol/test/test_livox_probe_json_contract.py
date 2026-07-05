from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_livox_probe_writes_numeric_quality_fields():
    probe = (ROOT / "scripts/waver_livox_mid360_probe.py").read_text()
    for token in (
        "pointcloud_type",
        "pointcloud_rate_hz",
        "scan_rate_hz",
        "point_count_mean",
        "point_count_min",
        "point_count_max",
        "timestamp_monotonic",
        "dropout_count",
        "has_fields_x_y_z",
        "scan_adapter_ready",
        "tf_livox_to_base_link_ready",
        "detected_topics",
        "suggested_remaps",
        "forbidden Livox topic typo detected",
    ):
        assert token in probe


def test_livox_probe_uses_alias_config_and_no_lider_typo_in_source_contract():
    probe = (ROOT / "scripts/waver_livox_mid360_probe.py").read_text()
    aliases = (ROOT / "config/sensors/livox_topic_aliases.yaml").read_text()
    assert "livox_topic_aliases.yaml" in probe
    assert "/livox/lidar" in aliases
    assert "/livox/lider" in aliases
    assert "forbidden_topics" in aliases

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PROFILE_DIR = ROOT / "config/real_profiles"


def load(name: str) -> dict:
    return yaml.safe_load((PROFILE_DIR / name).read_text())


def test_speed_tiers_are_explicit_and_evidence_gated():
    sensor = load("sensor_live.yaml")
    inspection = load("inspection_dry_run.yaml")
    supervised = load("supervised_bird_patrol.yaml")
    autonomous = load("autonomous_bird_patrol_locked.yaml")
    start = (ROOT / "scripts/waver_bird_patrol_field_start.sh").read_text()

    assert sensor["enable_waver_base_driver"] is False
    assert sensor["max_linear_speed"] == 0.0
    assert sensor["max_angular_speed"] == 0.0

    assert inspection["enable_waver_base_driver"] is True
    assert inspection["serial_port_required"] is True
    assert inspection["max_linear_speed"] <= 0.05
    assert inspection["max_angular_speed"] <= 0.20

    assert 0.08 <= supervised["max_linear_speed"] <= 0.12
    assert 0.25 <= supervised["max_angular_speed"] <= 0.35
    assert supervised["operator_confirmation_required"] is True
    assert supervised["collision_monitor_required"] is True
    assert supervised["blackbox_required"] is True

    assert autonomous["collision_monitor_required_for_autonomous"] is True
    assert "production speed tier requires WAVER_ACK_PRODUCTION_SPEED_EVIDENCE=1" in start


def test_profile_mode_mapping_is_stable():
    start = (ROOT / "scripts/waver_bird_patrol_field_start.sh").read_text()
    for token in (
        "sensor-live)",
        "lidar-tracking)",
        "detector-live|fusion-live|inspection-dry-run)",
        "supervised-deterrence|supervised-bird-patrol)",
        "autonomous-patrol)",
    ):
        assert token in start

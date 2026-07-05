from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "config/real_profiles/bird_patrol_production.yaml"


def load_profile() -> dict:
    return yaml.safe_load(PROFILE.read_text())


def test_bird_patrol_production_profile_contract():
    data = load_profile()
    assert data["profile"] == "bird_patrol_production"
    assert data["schema_version"] == "bird_patrol_profile_v1"
    assert data["source_of_truth"] == "nested_sections"
    assert data["compatibility_flat_aliases"] is True
    assert data["enable_bird_detector"] is True
    assert data["enable_bird_3d_fusion"] is True
    assert data["enable_target_goal_manager"] is True
    assert data["enable_sound_deterrent"] is True
    assert data["enable_sound_output"] is False
    assert data["require_camera_lidar_extrinsic"] is True
    assert data["require_detector_model"] is True
    assert data["max_linear_speed"] <= 0.05
    assert data["max_angular_speed"] <= 0.20
    assert data["pointcloud_topic"] == "/livox/lidar"
    assert set(data["required_external_ack_for_sound_output"]) >= {
        "WAVER_ACK_SOUND_HARDWARE",
        "WAVER_ACK_LOCAL_SOUND_LAW",
        "WAVER_ACK_OPERATOR_SUPERVISION",
    }


def test_nested_profile_keeps_product_capabilities_active_but_sound_safe():
    data = load_profile()
    assert data["mission"]["lidar_only_target_allows_inspection_only"] is True
    assert data["fusion"]["require_fusion_valid_before_sound"] is True
    assert data["alignment"]["allow_topic_only_alignment_in_production"] is False
    assert data["deterrence"]["enable_sound_deterrent"] is True
    assert data["deterrence"]["enable_sound_output"] is False
    assert data["navigation"]["collision_monitor_required_for_autonomous"] is True
    assert data["lidar"]["scan_topic"] == "/scan_safety"

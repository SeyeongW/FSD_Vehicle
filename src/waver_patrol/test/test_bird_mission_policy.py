from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
MISSION = ROOT / "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py"
TARGET = ROOT / "src/waver_patrol/waver_patrol/mission/target_goal_manager_node.py"


def test_sound_requires_bird_fusion_dynamic_and_centering():
    text = MISSION.read_text()
    for token in (
        "self.bird_confirmed",
        "self.bird_target_valid",
        "self.dynamic_valid",
        "self.camera_target_centered",
        "require_3d_fusion_valid_for_sound",
        "require_dynamic_valid_for_sound",
        "dynamic_valid_grace_sec_for_sound",
        "dynamic_valid_for_sound",
        "require_camera_centered_for_sound",
        "bird_3d_fusion_not_valid",
        "dynamic_target_not_valid",
        "camera_not_centered",
    ):
        assert token in text


def test_target_goal_manager_uses_standoff_and_allows_lidar_inspection_only():
    text = TARGET.read_text()
    assert "allow_lidar_dynamic_without_bird_confirmed_for_inspection" in text
    assert "inspection_goal_offset_distance_m" in text
    assert "TARGET_INSPECTION" in text
    assert "bird_3d_fusion_not_valid_for_inspection" in text


def test_gazebo_can_disable_fusion_sound_gate_but_real_keeps_it():
    gazebo_path = ROOT / "src/ugv_main/ugv_gazebo/param/bird_patrol/ugv_gazebo_bird_patrol_seo.yaml"
    if not gazebo_path.exists():
        pytest.skip("Gazebo-only profile is not part of the field release subset")
    gazebo = gazebo_path.read_text()
    real = (ROOT / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml").read_text()
    assert "require_3d_fusion_valid_for_sound: false" in gazebo
    assert "require_dynamic_valid_for_sound: true" in gazebo
    assert "dynamic_valid_grace_sec_for_sound: 8.0" in gazebo
    assert "require_camera_centered_for_sound: true" in gazebo
    assert "require_3d_fusion_valid_for_sound: true" in real
    assert "dynamic_valid_grace_sec_for_sound: 2.0" in real

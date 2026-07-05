from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "config/real_profiles/bird_patrol_production.yaml"


def test_production_nested_values_match_flat_compatibility_aliases():
    data = yaml.safe_load(PROFILE.read_text())
    pairs = {
        ("base", "enable_waver_base_driver"): "enable_waver_base_driver",
        ("base", "serial_port_required"): "serial_port_required",
        ("base", "enable_robot_localization"): "enable_robot_localization",
        ("base", "max_linear_speed"): "max_linear_speed",
        ("base", "max_angular_speed"): "max_angular_speed",
        ("navigation", "enable_nav2"): "enable_nav2",
        ("navigation", "enable_mission_patrol_manager"): "enable_mission_patrol_manager",
        ("navigation", "enable_auto_behavior_mux"): "enable_auto_behavior_mux",
        ("navigation", "enable_target_goal_manager"): "enable_target_goal_manager",
        ("lidar", "pointcloud_topic"): "pointcloud_topic",
        ("lidar", "scan_topic"): "scan_topic",
        ("camera", "camera_image_topic"): "camera_image_topic",
        ("camera", "camera_info_topic"): "camera_info_topic",
        ("perception", "enable_bird_detector"): "enable_bird_detector",
        ("fusion", "enable_bird_3d_fusion"): "enable_bird_3d_fusion",
        ("alignment", "enable_camera_alignment"): "enable_camera_alignment",
        ("deterrence", "enable_sound_deterrent"): "enable_sound_deterrent",
        ("deterrence", "enable_sound_output"): "enable_sound_output",
        ("mission", "inspection_standoff_m"): "inspection_standoff_m",
    }
    for (section, nested_key), flat_key in pairs.items():
        assert data[section][nested_key] == data[flat_key], f"{section}.{nested_key} != {flat_key}"

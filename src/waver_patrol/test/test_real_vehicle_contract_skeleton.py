from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_remote_ui_real_profile_does_not_publish_direct_cmd_vel_by_default():
    ui = text("src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py")
    assert 'declare_parameter("publish_direct_cmd_vel", False)' in ui
    assert 'self.profile == "real" and self.publish_direct_cmd_vel' in ui
    assert 'manual_cmd_vel_topic", "/waver/manual_cmd_vel"' in ui


def test_real_launch_has_serial_single_owner_guards():
    launch = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert "enable_waver_base_driver=true requires serial_port" in launch
    assert "forbids split serial nodes" in launch
    assert "split command and feedback serial owners are forbidden" in launch


def test_real_launch_disables_base_driver_tf_in_ekf_mode():
    launch = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert '"publish_tf"' in launch
    assert "'false' if '" in launch
    assert "odom_source" in launch
    assert "' == 'ekf'" in launch


def test_mission_manager_has_departure_pose_return_contract():
    mission = text("src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py")
    assert "interrupted_departure_pose" in mission
    assert "snapshot_departure_pose" in mission
    assert "RETURN_TO_DEPARTURE_POSE" in mission
    assert "RESUME_PATROL" in mission


def test_real_sound_output_is_disabled_without_ack():
    config = text("src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml")
    assert "require_bird_confirmed_for_sound: true" in config
    assert "require_camera_classification_before_sound: true" in config
    assert "enable_sound_output: false" in config
    assert "legal_safety_ack_required: true" in config
    assert "safety_ack: false" in config


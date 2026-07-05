import yaml
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROFILE_DIR = ROOT / "config/real_profiles"


def profile(name: str) -> dict:
    return yaml.safe_load((PROFILE_DIR / name).read_text())


def test_profile_yaml_is_source_of_truth_for_speed_limits_and_experimental_stack():
    wheel_on = profile("wheel_on_low_speed.yaml")
    bird = profile("bird_full_experimental.yaml")

    assert wheel_on["max_linear_speed"] <= 0.05
    assert wheel_on["max_angular_speed"] <= 0.20
    assert wheel_on["enable_bird_detector"] is False
    assert wheel_on["enable_bird_3d_fusion"] is False
    assert wheel_on["enable_sound_output"] is False

    assert bird["enable_bird_detector"] is True
    assert bird["enable_bird_3d_fusion"] is True
    assert bird["enable_sound_output"] is False
    assert set(bird["required_config"]) >= {"bird_model_path", "camera_image_topic", "camera_info_topic", "camera_lidar_extrinsic"}


def test_launch_and_field_defaults_do_not_conflict_with_ekf_policy():
    field = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text(errors="replace")
    real_launch = (ROOT / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py").read_text(errors="replace")
    indoor_launch = (ROOT / "src/waver_patrol/launch/waver_indoor_patrol_real.launch.py").read_text(errors="replace")

    assert 'ODOM_SOURCE="${ODOM_SOURCE:-${PROFILE_ODOM_SOURCE:-ekf}}"' in field
    assert "WAVER_REAL_PROFILE" in field
    assert 'DeclareLaunchArgument("odom_source", default_value="ekf")' in real_launch
    assert 'DeclareLaunchArgument("odom_source", default_value="ekf")' in indoor_launch


def test_real_bird_launch_defaults_keep_experimental_stack_disabled():
    launch = (ROOT / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py").read_text(errors="replace")
    for arg in (
        "enable_bird_detector",
        "enable_bird_3d_fusion",
        "enable_camera_gimbal_controller",
        "enable_sound_deterrent",
        "enable_target_departure_monitor",
        "enable_radar_command_bridge",
        "enable_target_goal_manager",
        "enable_experiment_logger",
    ):
        assert f'DeclareLaunchArgument("{arg}", default_value="false")' in launch

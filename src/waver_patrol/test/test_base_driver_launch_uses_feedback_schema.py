from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_real_launch_passes_base_feedback_schema_to_driver():
    launch = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert 'DeclareLaunchArgument("feedback_schema_path"' in launch
    assert '"feedback_schema_path": ParameterValue(LaunchConfiguration("feedback_schema_path"), value_type=str)' in launch


def test_lidar_nav_backend_passes_schema_path_into_real_launch():
    script = text("scripts/waver_field_lidar_nav_backend_start.sh")
    assert "FEEDBACK_SCHEMA_PATH" in script
    assert "feedback_schema_path:=${FEEDBACK_SCHEMA_PATH}" in script
    assert "config/waver_base_feedback_schema.yaml" in script


def test_base_driver_state_reports_feedback_and_timeout_fields():
    driver = text("src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py")
    for token in (
        "calibration_loaded=",
        "stop_burst_sent=",
        "cmd_timeout_active=",
        "odom_rate_hz=",
        "imu_rate_hz=",
        "voltage_status=",
        "ODOM_FEEDBACK_OK",
    ):
        assert token in driver

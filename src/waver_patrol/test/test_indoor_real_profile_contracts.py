from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_indoor_real_launch_has_safe_defaults():
    launch = text("src/waver_patrol/launch/waver_indoor_patrol_real.launch.py")
    assert 'DeclareLaunchArgument("default_mode", default_value="STANDBY")' in launch
    assert 'DeclareLaunchArgument("safety_max_linear_speed", default_value="0.05")' in launch
    assert 'DeclareLaunchArgument("safety_max_angular_speed", default_value="0.20")' in launch
    assert '"require_scan": "true"' in launch
    assert '"enable_test_publishers": "false"' in launch
    assert '"enable_deep_learning_stub": "false"' in launch
    assert 'DeclareLaunchArgument("include_existing_ugv_driver", default_value="false")' in launch
    assert 'DeclareLaunchArgument("enable_velocity_smoother", default_value="true")' in launch
    assert 'DeclareLaunchArgument("enable_battery_return", default_value="false")' in launch


def test_indoor_real_launch_disables_non_indoor_stacks_by_default():
    launch = text("src/waver_patrol/launch/waver_indoor_patrol_real.launch.py")
    for key in (
        "enable_bird_detector",
        "enable_bird_3d_fusion",
        "enable_camera_gimbal_controller",
        "enable_sound_deterrent",
        "enable_sound_output",
        "enable_target_departure_monitor",
        "enable_radar_command_bridge",
        "enable_target_goal_manager",
        "enable_auto_behavior_mux",
        "enable_experiment_logger",
        "enable_pointcloud_lidar_objects",
        "enable_moving_object_map_transform",
        "enable_moving_object_motion_filter",
        "enable_collision_monitor",
    ):
        assert f'"{key}": "false"' in launch
    assert '"enable_battery_return": LaunchConfiguration("enable_battery_return")' in launch


def test_real_launch_exposes_battery_return_and_rviz_passthrough():
    launch = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert 'DeclareLaunchArgument("enable_battery_return", default_value="true")' in launch
    assert '"enable_battery_return": LaunchConfiguration("enable_battery_return")' in launch
    assert 'DeclareLaunchArgument("enable_target_goal_manager", default_value="false")' in launch
    assert '"enable_target_goal_manager": LaunchConfiguration("enable_target_goal_manager")' in launch
    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in launch
    assert '"use_rviz": LaunchConfiguration("use_rviz")' in launch


def test_real_nav2_min_y_velocity_threshold_is_diff_drive_safe():
    config = text("src/waver_patrol/config/nav2_params_waver_real.yaml")
    assert "min_y_velocity_threshold: 0.001" in config
    assert "max_vel_y: 0.0" in config
    assert "vy_samples: 1" in config


def test_repo_cleanup_audit_and_docs_are_present():
    assert (ROOT / "scripts/waver_repo_cleanup_audit.py").exists()
    assert (ROOT / "src/waver_patrol/docs/repo_inventory.md").exists()
    assert (ROOT / "src/waver_patrol/docs/package_role_matrix.md").exists()
    assert (ROOT / "src/waver_patrol/docs/indoor_real_topic_contract.md").exists()
    assert (ROOT / "src/waver_patrol/docs/cleanup/stale_candidates.md").exists()
    assert (ROOT / "src/waver_patrol/docs/cleanup/cleanup_report.md").exists()
    assert (ROOT / "src/waver_patrol/scripts/waver_indoor_patrol_status.sh").exists()
    setup = text("src/waver_patrol/setup.py")
    assert "docs/cleanup" in setup


def test_indoor_runbook_mentions_tf_odom_rosbag_and_status_helper():
    runbook = text("src/waver_patrol/docs/indoor_real_patrol_runbook.md")
    assert "waver_indoor_patrol_status.sh --strict" in runbook
    assert "ros2 bag record" in runbook
    assert "tf2_echo map odom" in runbook
    assert "tf2_echo odom base_link" in runbook
    assert "Wheel-off positive `linear.x`" in runbook

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_real_config_waits_for_target_departure_and_returns_to_departure_pose():
    config = text("src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml")
    assert 'resume_policy: "RETURN_TO_DEPARTURE_POSE"' in config
    assert "wait_bird_clear_after_sound: true" in config
    assert 'target_departed_topic: "/waver/target_departed"' in config
    assert "target_departure_monitor_node:" in config


def test_real_mission_launch_includes_target_departure_monitor():
    launch = text("src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py")
    assert 'DeclareLaunchArgument("enable_target_departure_monitor"' in launch
    assert 'executable="target_departure_monitor_node"' in launch
    assert 'name="target_departure_monitor_node"' in launch


def test_real_mission_launch_routes_target_tracking_through_auto_behavior_mux():
    launch = text("src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py")
    assert 'DeclareLaunchArgument("enable_auto_behavior_mux"' in launch
    assert 'executable="auto_behavior_mux_node"' in launch
    assert '"cmd_vel_target_track_topic": cmd_vel_target_track_topic' in launch
    assert '"cmd_vel_auto_topic": cmd_vel_auto_topic' in launch
    assert '"cmd_vel_auto_topic": PythonExpression' in launch


def test_legacy_base_odometry_node_has_launch_condition_and_real_disable_path():
    bringup = text("src/ugv_main/ugv_bringup/launch/bringup_lidar.launch.py")
    nav = text("src/ugv_main/ugv_nav/launch/nav.launch.py")
    real = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")

    assert "enable_legacy_ugv_base_odometry_node" in bringup
    assert "condition=IfCondition(LaunchConfiguration('enable_legacy_ugv_base_odometry_node'))" in bringup
    assert "enable_legacy_ugv_base_odometry_node" in nav
    assert "enable_waver_base_driver" in real
    assert "enable_legacy_ugv_base_odometry_node" in real


def test_ekf_and_base_driver_real_topics_are_consistent():
    ekf = text("src/waver_patrol/config/ekf_waver_real.yaml")
    real = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    base_driver = text("src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py")

    assert "odom0: /odom_raw" in ekf
    assert "imu0: /imu/data_raw" in ekf
    assert "'/odom_raw' if '" in real
    assert '"publish_tf"' in real
    assert '"publish_legacy_float32_odom_raw": False' in real
    assert "publish_legacy_float32_odom_raw" in base_driver

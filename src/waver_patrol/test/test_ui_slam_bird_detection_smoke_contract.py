from __future__ import annotations

import pathlib


REPO = pathlib.Path(__file__).resolve().parents[3]


def text(rel: str) -> str:
    return (REPO / rel).read_text(encoding="utf-8")


def test_combined_ui_slam_bird_smoke_scripts_exist_and_check_required_topics() -> None:
    smoke = text("scripts/run_ui_slam_bird_detection_gazebo_smoke.sh")
    checker = text("scripts/check_ui_slam_bird_detection_result.py")

    required_topics = [
        "/waver/current_map_source",
        "/map",
        "/waver/mapping_path",
        "/waver/target_class",
        "/waver/target_confidence",
        "/waver/bird_confirmed",
        "/waver/bird_detector_state",
        "/waver/bird_fusion_state",
        "/waver/lidar_target_state",
        "/waver/camera_alignment_state",
        "/waver/camera_target_centered",
        "/waver/sound_alert_state",
        "/waver/sound_task_done",
    ]
    for topic in required_topics:
        assert topic in smoke

    for authority_topic in ["/map", "/cmd_vel", "/waver/mode"]:
        assert authority_topic in smoke
    for required_metric in [
        "current_map_source_slam_live",
        "bird_topics_visible",
        "bird_topics_fresh",
        "bird_detector_state_fresh",
        "bird_fusion_state_fresh",
        "mapping_path_visible",
        "no_patrol_conflict",
        "no_target_approach_without_arm",
        "no_sound_without_arm",
        "map_quality_pass",
    ]:
        assert required_metric in smoke
        assert required_metric in checker

    assert "PATROL_NAVIGATING" in smoke
    assert "SOUND_BLOCKED_MAPPING_MODE" in smoke
    assert "UI_SLAM_BIRD_DETECTION_SMOKE=PASS" in smoke


def test_combined_mapping_bird_launch_is_sim_only_and_keeps_dangerous_actions_off_by_default() -> None:
    launch = text("src/waver_patrol/launch/waver_gazebo_mapping_bird_detection.launch.py")

    assert "waver_gazebo_mapping_debug.launch.py" in launch
    assert "SIM_ONLY" in launch
    assert 'DeclareLaunchArgument("integrated_inspection_mode", default_value="false")' in launch
    assert 'DeclareLaunchArgument("arm_sound_deterrent", default_value="false")' in launch
    assert 'DeclareLaunchArgument("use_deterministic_bird_topics", default_value="true")' in launch
    assert "deterministic_bird_ui_status_publisher" in launch
    assert "/waver/bird_detector_state" in launch
    assert "/waver/bird_fusion_state" in launch


def test_pure_mapping_debug_remains_isolated_from_bird_stack() -> None:
    mapping_debug = text("src/waver_patrol/launch/waver_gazebo_mapping_debug.launch.py")
    mapping_mode = text("src/waver_patrol/launch/gazebo_mapping_mode.launch.py")

    assert "Bird/target pointcloud nodes are disabled to isolate" in mapping_debug
    assert '"enable_target_goal_manager": "false"' in mapping_mode
    assert '"enable_pointcloud_lidar_objects": "false"' in mapping_mode
    assert '"enable_moving_object_motion_filter": "false"' in mapping_mode
    assert '"enable_cluster_node": "false"' in mapping_mode

from __future__ import annotations

import pathlib


REPO = pathlib.Path(__file__).resolve().parents[3]


def test_remote_ui_keeps_slam_and_bird_status_plumbing_together() -> None:
    panel = (REPO / "src" / "ugv_main" / "ugv_tools" / "ugv_tools" / "waver_remote_panel.py").read_text(
        encoding="utf-8"
    )

    mapping_topics = [
        'self.declare_parameter("mapping_command_topic", "/waver/mapping_command")',
        'self.declare_parameter("ui_map_reset_topic", "/waver/ui_map_reset")',
        'self.declare_parameter("current_map_source_topic", "/waver/current_map_source")',
        'self.declare_parameter("map_topic", "/map")',
        'self.declare_parameter("fixed_map_topic", "/map_fixed")',
    ]
    bird_topics = [
        'self.declare_parameter("target_class_topic", "/waver/target_class")',
        'self.declare_parameter("target_confidence_topic", "/waver/target_confidence")',
        'self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")',
        'self.declare_parameter("bird_detector_state_topic", "/waver/bird_detector_state")',
        'self.declare_parameter("bird_fusion_state_topic", "/waver/bird_fusion_state")',
        'self.declare_parameter("camera_alignment_state_topic", "/waver/camera_alignment_state")',
        'self.declare_parameter("sound_mission_status_topic", "/waver/sound_alert_state")',
    ]

    for expected in mapping_topics + bird_topics:
        assert expected in panel

    assert 'self.mapping_command_pub = self.create_publisher(' in panel
    assert "self.ui_map_reset_callback" in panel
    assert "self.current_map_source_callback" in panel
    assert "self.set_timed_text_state" in panel
    assert "self.target_confidence_callback" in panel
    assert "self.bird_confirmed_callback" in panel
    assert '"bird_detector_state", "bird_detector_update_time"' in panel
    assert '"bird_fusion_state", "bird_fusion_update_time"' in panel

    assert 'self.map_apply_var.set(f"map source: {current_map_source}' in panel
    assert 'f"class={target_class}, conf={target_confidence:.2f}, bird={bird_confirmed}' in panel
    assert "detector={bird_detector_state" in panel
    assert "fusion={bird_fusion_state" in panel
    assert 'self.camera_var.set(f"camera: {camera}' in panel
    assert 'self.sound_var.set(f"sound: {sound}' in panel

    assert "publish_direct_cmd_vel" in panel
    assert 'self.declare_parameter("manual_cmd_vel_topic", "/waver/manual_cmd_vel")' in panel


def test_mapping_debug_documents_that_bird_stack_is_intentionally_isolated() -> None:
    mapping_debug = (REPO / "src" / "waver_patrol" / "launch" / "waver_gazebo_mapping_debug.launch.py").read_text(
        encoding="utf-8"
    )
    mapping_mode = (REPO / "src" / "waver_patrol" / "launch" / "gazebo_mapping_mode.launch.py").read_text(
        encoding="utf-8"
    )

    assert "Bird/target pointcloud nodes are disabled to isolate" in mapping_debug
    assert '"enable_target_goal_manager": "false"' in mapping_mode
    assert '"enable_pointcloud_lidar_objects": "false"' in mapping_mode
    assert '"gazebo_visualizer_publish_map": "false"' in mapping_mode

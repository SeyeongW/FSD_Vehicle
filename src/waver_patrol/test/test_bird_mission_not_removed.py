from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_core_bird_mission_files_are_present():
    required = [
        "src/waver_patrol/waver_patrol/perception/pointcloud_lidar_objects_node.py",
        "src/waver_patrol/waver_patrol/perception/lidar_aerial_motion_detector_node.py",
        "src/waver_patrol/waver_patrol/perception/moving_object_motion_filter_node.py",
        "src/waver_patrol/waver_patrol/perception/moving_object_map_transform_node.py",
        "src/waver_patrol/waver_patrol/mission/target_goal_manager_node.py",
        "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py",
        "src/waver_patrol/waver_patrol/perception/bird_detector_node.py",
        "src/waver_patrol/waver_patrol/perception/bird_3d_fusion_node.py",
        "src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py",
        "src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py",
        "src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py",
        "src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py",
    ]
    missing = [rel for rel in required if not (ROOT / rel).exists()]
    assert not missing


def test_production_launch_keeps_bird_pipeline_in_graph():
    launch = (ROOT / "src/waver_patrol/launch/bird_patrol_production.launch.py").read_text()
    for token in (
        "enable_bird_detector",
        "enable_bird_3d_fusion",
        "enable_camera_gimbal_controller",
        "enable_target_goal_manager",
        "enable_target_departure_monitor",
        "enable_sound_deterrent",
        "bird_mission_supervisor_node",
    ):
        assert token in launch
    assert '"enable_test_publishers": "false"' in launch
    assert '"enable_deep_learning_stub": "false"' in launch

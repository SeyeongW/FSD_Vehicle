from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_seo_camera_alignment_uses_mission_target_and_tf():
    launch = (ROOT / "src/ugv_main/ugv_gazebo/launch/bird_patrol/ugv_gazebo_bird_patrol_seo.launch.py").read_text()
    camera = (ROOT / "src/waver_seo_tracking/waver_seo_tracking/seo_camera_tilt_joint_node.py").read_text()
    tracker = (ROOT / "src/waver_seo_tracking/waver_seo_tracking/seo_observation_body_tracker_node.py").read_text()
    assert '"/waver/camera_aim_target_pose"' in launch
    assert '"base_footprint"' in launch
    assert "lookup_transform" in camera
    assert "lookup_transform" in tracker
    assert "min_turn_speed" in tracker

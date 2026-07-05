from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_release_checker_only_invokes_release_safe_self_test_wrapper():
    checker = (ROOT / "scripts/check_bird_mission_field_release.py").read_text()
    assert '["bash", "scripts/run_bird_mission_release_self_tests.sh"]' in checker
    self_test_body = checker.split("def self_test", 1)[1].split("def main", 1)[0]
    assert "python3 -m pytest" not in self_test_body
    assert "run_ui_slam_bird_detection_gazebo_smoke.sh" not in self_test_body
    assert "run_gazebo_lidar_spatial_response_smoke.sh" not in self_test_body


def test_release_wrapper_records_timeout_and_never_runs_gazebo_or_hardware():
    runner = (ROOT / "scripts/run_bird_mission_release_self_tests.sh").read_text()
    assert "WAVER_NO_HARDWARE=1" in runner
    assert "WAVER_BLOCK_SERIAL=1" in runner
    assert "WAVER_DISABLE_SOUND_OUTPUT=1" in runner
    assert "timeout --kill-after=5s" in runner
    assert "run_ui_slam_bird_detection_gazebo_smoke.sh" not in runner
    assert "run_gazebo_lidar_spatial_response_smoke.sh" not in runner

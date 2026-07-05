import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_bird_mission_release_can_be_generated_and_checked(tmp_path):
    archive = tmp_path / "waver_bird_mission_field_release.tar.gz"
    make = subprocess.run(
        ["python3", str(ROOT / "scripts/make_bird_mission_field_release.py"), "--output", str(archive)],
        text=True,
        capture_output=True,
        check=False,
    )
    assert make.returncode == 0, make.stdout + make.stderr
    check = subprocess.run(
        ["python3", str(ROOT / "scripts/check_bird_mission_field_release.py"), "--path", str(archive)],
        text=True,
        capture_output=True,
        check=False,
    )
    assert check.returncode == 0, check.stdout + check.stderr


def test_release_self_test_scripts_record_timeouts_and_step_reports():
    runner = (ROOT / "scripts/run_bird_mission_release_self_tests.sh").read_text()
    checker = (ROOT / "scripts/check_bird_mission_field_release.py").read_text()
    maker = (ROOT / "scripts/make_bird_mission_field_release.py").read_text()

    assert "RELEASE_SELF_TEST_TIMEOUT_TARGETED_PYTEST" in runner
    assert "timeout --kill-after=5s" in runner
    assert "latest_bird_mission_release_self_test_steps.jsonl" in runner
    assert '"steps": steps' in runner
    assert "--self-test-timeout-sec" in checker
    assert '"timed_out": timed_out' in checker
    assert "subprocess.TimeoutExpired" in checker
    assert '"status": "TIMEOUT"' in checker
    for token in (
        "src/waver_patrol/rviz/waver_field_operator.rviz",
        "docs/bird_patrol_field_profiles.md",
        "config/real_profiles/sensor_live.yaml",
        "config/real_profiles/supervised_bird_patrol.yaml",
        "config/real_profiles/autonomous_bird_patrol_locked.yaml",
    ):
        assert token in checker
        assert token in maker

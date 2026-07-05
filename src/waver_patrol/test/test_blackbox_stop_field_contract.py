from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_blackbox_remote_mode_records_required_bird_mission_topics():
    text = (ROOT / "scripts/waver_blackbox_recorder.sh").read_text()
    for token in (
        "--remote",
        "--host",
        "--user",
        "--container",
        "--workspace",
        "docker exec",
        "/waver/bird_fusion_state",
        "/waver/bird_target_pose_map",
        "/waver/sound_alert_state",
        "/waver/target_departed",
        "/waver/bird_mission_supervisor_state",
    ):
        assert token in text


def test_stop_all_zeroes_final_and_safety_cmd_and_is_not_unconditional():
    text = (ROOT / "scripts/waver_field_stop_all.sh").read_text()
    for token in (
        "/cmd_vel",
        "/waver/cmd_vel_safety",
        "/waver/manual_cmd_vel",
        "/waver/mission_command",
        "/waver/sound_command",
        "STOP_FAILED",
        "STOP_CONFIRMED",
    ):
        assert token in text
    assert "STOP_CONFIRMED_REMOTE\nREMOTE_STOP" not in text

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_lidar_nav_backend_passes_sound_output_and_ack_to_launch():
    text = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text()
    assert 'ENABLE_SOUND_OUTPUT="${ENABLE_SOUND_OUTPUT:-${PROFILE_ENABLE_SOUND_OUTPUT:-false}}"' in text
    assert 'SOUND_SAFETY_ACK="${SOUND_SAFETY_ACK:-false}"' in text
    assert "enable_sound_output:=${ENABLE_SOUND_OUTPUT}" in text
    assert "sound_safety_ack:=${SOUND_SAFETY_ACK}" in text
    assert "enable_sound_output:=false" not in text
    assert "sound_safety_ack:=false" not in text

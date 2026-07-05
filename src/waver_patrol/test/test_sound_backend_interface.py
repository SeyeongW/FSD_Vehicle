from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / "src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py"


def test_sound_backend_interface_exists_and_is_mockable():
    text = NODE.read_text()
    for token in (
        "class SoundBackend",
        "class DisabledSoundBackend",
        "class MockSoundBackend",
        "class AlsaSoundBackend",
        "class GpioSoundBackend",
        "class SerialSoundBackend",
        "WAVER_ACK_SOUND_HARDWARE",
        "WAVER_ACK_LOCAL_SOUND_LAW",
        "WAVER_ACK_OPERATOR_SUPERVISION",
        "SAFETY_STOP",
    ):
        assert token in text
    assert "subprocess.run" not in text
    assert "os.system" not in text

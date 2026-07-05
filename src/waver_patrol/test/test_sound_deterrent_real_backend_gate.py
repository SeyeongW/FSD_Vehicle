from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / "src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py"


def test_sound_real_output_requires_ack_backend_and_gates():
    text = NODE.read_text()
    for token in (
        "WAVER_ACK_SOUND_HARDWARE",
        "WAVER_ACK_LOCAL_SOUND_LAW",
        "WAVER_ACK_OPERATOR_SUPERVISION",
        "FUSION_NOT_VALID",
        "DYNAMIC_NOT_VALID",
        "CAMERA_NOT_CENTERED",
        "HARDWARE_BACKEND_DISABLED",
        "alsa",
        "gpio",
        "serial",
    ):
        assert token in text
    assert "subprocess.run" not in text
    assert "os.system" not in text

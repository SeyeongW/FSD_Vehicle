from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_livox_field_config_is_local_template_and_runtime_patched():
    template = ROOT / "config/sensors/livox_mid360_field.example.yaml"
    backend = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text()
    driver_config = ROOT / "src/livox_ros_driver2/config/MID360_config.json"

    assert template.exists()
    assert "livox_mid360_field.local.yaml" in backend
    assert "LIVOX_FIELD_CONFIG" in backend
    assert "LIVOX_CONFIG_HOST_IP" in backend
    if driver_config.exists():
        text = driver_config.read_text(errors="replace")
        assert "100.112." not in text
        assert "10.139." not in text

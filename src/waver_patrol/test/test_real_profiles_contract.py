import yaml
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROFILE_DIR = ROOT / "config/real_profiles"


def load(name: str) -> dict:
    return yaml.safe_load((PROFILE_DIR / name).read_text())


def test_real_profiles_exist_and_use_ekf_where_autonomy_runs():
    for name in (
        "indoor_nav_only.yaml",
        "lidar_nav_backend.yaml",
        "wheel_off_driver_check.yaml",
        "wheel_on_low_speed.yaml",
        "bird_full_experimental.yaml",
    ):
        assert (PROFILE_DIR / name).exists()

    for name in ("indoor_nav_only.yaml", "lidar_nav_backend.yaml", "wheel_on_low_speed.yaml", "bird_full_experimental.yaml"):
        profile = load(name)
        assert profile["odom_source"] == "ekf"
        assert profile["max_linear_speed"] <= 0.05
        assert profile["max_angular_speed"] <= 0.20


def test_profiles_keep_experimental_bird_sound_off_by_default():
    indoor = load("indoor_nav_only.yaml")
    lidar = load("lidar_nav_backend.yaml")
    wheel_on = load("wheel_on_low_speed.yaml")
    bird = load("bird_full_experimental.yaml")

    for profile in (indoor, lidar, wheel_on):
        assert profile["enable_bird_detector"] is False
        assert profile["enable_bird_3d_fusion"] is False
        assert profile["enable_sound_output"] is False
        assert "fake" in profile["forbidden_nodes"]

    assert bird["enable_bird_detector"] is True
    assert bird["enable_bird_3d_fusion"] is True
    assert bird["enable_sound_output"] is False
    assert "WAVER_ACK_SOUND_HARDWARE" in bird["required_external_ack"]
    assert "WAVER_ACK_LOCAL_SOUND_LAW" in bird["required_external_ack"]


def test_field_lidar_backend_defaults_to_ekf_and_production_mode():
    script = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text(errors="replace")
    assert 'ODOM_SOURCE="${ODOM_SOURCE:-ekf}"' in script
    assert 'WAVER_FIELD_MODE="${WAVER_FIELD_MODE:-production}"' in script
    assert "hot overlay disabled" in script

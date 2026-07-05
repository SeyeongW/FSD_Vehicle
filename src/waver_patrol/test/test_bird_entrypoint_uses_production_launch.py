import yaml
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_field_entrypoint_uses_product_launch_and_passes_core_args():
    entry = text("scripts/waver_bird_patrol_field_start.sh")
    backend = text("scripts/waver_field_lidar_nav_backend_start.sh")
    production = text("src/waver_patrol/launch/bird_patrol_production.launch.py")
    profile = yaml.safe_load((ROOT / "config/real_profiles/bird_patrol_production.yaml").read_text())

    assert "bird_patrol_production.launch.py" in entry
    assert "bird_patrol_production.launch.py" in backend
    assert 'PROFILE="${WAVER_BIRD_PATROL_PROFILE:-}"' in entry
    assert 'PROFILE="sensor_live"' in entry
    assert 'PROFILE="inspection_dry_run"' in entry
    assert 'PROFILE="supervised_bird_patrol"' in entry
    assert 'PROFILE="autonomous_bird_patrol_locked"' in entry
    assert 'DEFAULT_FIELD_READINESS_LEVEL="L2"' in entry
    assert 'DEFAULT_FIELD_READINESS_LEVEL="L5"' in entry
    assert "waver_real_bird_autonomy.launch.py" in production
    assert "CAMERA_LIDAR_EXTRINSIC" in entry
    assert "camera_lidar_extrinsic:=${CAMERA_LIDAR_EXTRINSIC}" in backend
    assert "bird_model_path:=${BIRD_MODEL_PATH}" in backend
    assert 'SCAN_TOPIC="${SCAN_TOPIC:-${PROFILE_SCAN_TOPIC:-/scan_safety}}"' in backend
    assert profile["lidar"]["scan_topic"] == "/scan_safety"


def test_start_field_backend_does_not_default_to_legacy_open_loop():
    script = text("scripts/waver_start_field_backend.sh")
    assert "waver_field_lidar_nav_backend_start.sh" in script
    assert "waver_field_docker_backend_start.sh" not in script
    assert "WAVER_REAL_PROFILE" in script

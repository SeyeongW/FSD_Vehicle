from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_default_backend_entrypoint_uses_strict_lidar_nav_backend():
    script = text("scripts/waver_start_field_backend.sh")
    assert "waver_field_lidar_nav_backend_start.sh" in script
    assert "waver_field_docker_backend_start.sh" not in script
    assert "FIELD_READINESS_LEVEL" in script
    assert "WAVER_REAL_PROFILE" in script
    assert "FIELD_BACKEND_READY=PASS" in script
    assert "FIELD_BACKEND_READY=FAIL" in script
    assert "READY=YES" not in script


def test_strict_backend_invokes_readiness_checker_and_profile():
    script = text("scripts/waver_field_lidar_nav_backend_start.sh")
    assert "config/real_profiles/${WAVER_REAL_PROFILE}.yaml" in script
    assert "waver_field_readiness_check.py" in script
    assert "--profile" in script
    assert "LIDAR_NAV_BACKEND_READY=PASS|PASS_LIMITED|FAIL" in script

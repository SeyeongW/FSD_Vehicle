from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_lidar_backend_has_no_unconditional_drive_ready_yes():
    script = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text(errors="replace")
    assert "LIDAR_NAV_BACKEND_READY=YES" not in script
    assert "waver_field_readiness_check.py" in script
    assert "LIDAR_NAV_BACKEND_READY=PASS" in script
    assert "LIDAR_NAV_BACKEND_READY=FAIL" in script


def test_lidar_backend_distinguishes_started_from_ready():
    script = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text(errors="replace")
    assert "LIDAR_NAV_BACKEND_STARTED=YES" in script
    assert "FIELD_READINESS=" in script
    assert "LIDAR_NAV_BACKEND_READY=PASS|PASS_LIMITED|FAIL" in script

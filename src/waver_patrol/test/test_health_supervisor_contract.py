from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_health_supervisor_is_passive_and_reports_topics():
    script = (ROOT / "scripts/waver_health_supervisor.py").read_text(errors="replace")
    assert "ros2 topic pub" not in script
    assert "/waver/safety_state" in script
    assert "/waver/base_driver_state" in script
    assert "WAVER_HEALTH_SUPERVISOR=" in script
    assert "--require-base-driver" in script

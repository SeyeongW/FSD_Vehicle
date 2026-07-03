from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_field_release_checker_requires_hardware_readiness_assets():
    checker = text("scripts/check_field_release.py")
    for token in (
        "README_REAL_VEHICLE.md",
        "docs/hardware_readiness_levels.md",
        "docs/hardware_acceptance_matrix.md",
        "scripts/waver_field_readiness_check.py",
        "scripts/waver_base_feedback_probe.py",
        "scripts/waver_motor_calibration_wizard.py",
        "scripts/waver_blackbox_recorder.sh",
        "scripts/waver_field_stop_all.sh",
        "config/real_profiles/wheel_on_low_speed.yaml",
    ):
        assert token in checker


def test_real_vehicle_readme_uses_readiness_levels_and_stop_policy():
    readme = text("README_REAL_VEHICLE.md")
    assert "L0_SOURCE_CHECK" in readme
    assert "L5_AUTONOMOUS_PATROL" in readme
    assert "safety_cmd_mux_node" in readme
    assert "waver_field_stop_all.sh" in readme
    assert "physical E-stop" in readme

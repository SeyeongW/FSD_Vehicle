import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_readiness_audit_does_not_promote_sim_to_real_autonomy():
    result = subprocess.run(
        ["python3", "scripts/generate_bird_mission_readiness_audit.py"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    audit = (ROOT / "reports/bird_mission_readiness_audit.md").read_text(encoding="utf-8")
    assert "BIRD_PATROL_SOURCE_READY" in audit
    assert "`UI_SLAM_MAPPING_SIM_READY`: YES" in audit or "`UI_SLAM_MAPPING_SIM_READY`: NOT_RUN" in audit
    assert (
        "`UI_SLAM_AND_BIRD_DETECTION_SIM_READY`: YES" in audit
        or "`UI_SLAM_AND_BIRD_DETECTION_SIM_READY`: NOT_RUN" in audit
    )
    assert "`BIRD_PATROL_AUTONOMOUS_READY`: NOT_RUN" in audit
    assert "Gazebo/UI simulation PASS must never promote real autonomous readiness" in audit
    assert "## Current Package Analyzed From Workspace/Zip" in audit
    assert "raw workspace zip is a backup format only" in audit
    assert "deploy artifact must be the clean field release tarball" in audit
    assert "handoff_reference_head" in audit
    assert "actual_package_head_at_audit_generation" in audit


def test_readiness_script_blocks_pass_degraded_autonomous_promotion():
    script = (ROOT / "scripts/generate_bird_mission_readiness_audit.py").read_text(encoding="utf-8")
    assert "PASS_DEGRADED must never promote" in script
    assert "status_is_degraded" in script
    assert "autonomous-patrol" in script
    assert "BIRD_PATROL_AUTONOMOUS_READY" in script
    assert "NO_LIVE_HARDWARE_EVIDENCE" in script


def test_hardware_probe_absence_keeps_sensor_live_not_ready():
    result = subprocess.run(
        ["python3", "scripts/generate_bird_mission_readiness_audit.py"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    audit = (ROOT / "reports/bird_mission_readiness_audit.md").read_text(encoding="utf-8")
    if "live hardware evidence present: NO" in audit:
        assert "`BIRD_PATROL_SENSOR_LIVE_READY`: NOT_RUN" in audit
        assert "`BIRD_PATROL_DETECTOR_READY`: NOT_RUN" in audit
        assert "`BIRD_PATROL_FUSION_READY`: NOT_RUN" in audit

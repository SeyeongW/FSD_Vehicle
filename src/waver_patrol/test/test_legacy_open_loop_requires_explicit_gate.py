import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_legacy_open_loop_backend_fails_without_explicit_gate():
    result = subprocess.run(
        ["bash", "scripts/waver_field_docker_backend_start.sh"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert "WAVER_ALLOW_LEGACY_OPEN_LOOP_MICRO_PATROL=1" in result.stdout + result.stderr
    assert "LEGACY_BACKEND_READY=FAIL" in result.stdout + result.stderr


def test_legacy_open_loop_defaults_are_fail_closed():
    script = text("scripts/waver_field_docker_backend_start.sh")
    assert 'PATROL_ALLOW_OPEN_LOOP="${PATROL_ALLOW_OPEN_LOOP:-false}"' in script
    assert "-p require_scan:=true" in script
    assert "-p stop_on_battery_fault:=true" in script
    assert "BACKEND_READY=YES" not in script
    assert "LEGACY_BACKEND_READY=PASS_LIMITED" in script
    assert "legacy supervised open-loop diagnostic only" in script

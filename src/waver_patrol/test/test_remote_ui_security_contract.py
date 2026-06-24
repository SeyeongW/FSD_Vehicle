import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_remote_bridge_has_no_hardcoded_field_password():
    forbidden_password = "1234" + "1234"
    paths = [
        "src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py",
        "scripts/waver_field_local_ui_start.sh",
        "scripts/waver_field_docker_backend_start.sh",
        "scripts/waver_field_lidar_nav_backend_start.sh",
    ]
    for path in paths:
        assert forbidden_password not in text(path), path


def test_remote_bridge_password_auth_requires_explicit_env_gate():
    ui = text("src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py")
    local_script = text("scripts/waver_field_local_ui_start.sh")
    backend_script = text("scripts/waver_field_docker_backend_start.sh")

    assert re.search(r"remote_bridge_password[\"']\s*,\s*\n\s*[\"']{2}", ui)
    assert "remote_bridge_key_filename" in ui
    assert "WAVER_ALLOW_PASSWORD_SSH" in ui
    assert "WAVER_ALLOW_PASSWORD_SSH" in local_script
    assert "WAVER_ALLOW_PASSWORD_SSH" in backend_script


def test_remote_bridge_uses_candidate_topics_not_final_cmd_vel():
    ui = text("src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py")
    assert "/waver/manual_cmd_vel" in ui
    assert "/waver/mode_cmd" in ui
    assert "/waver/mission_command" in ui
    assert "publish_direct_cmd_vel" in ui
    assert 'self.profile == "real" and self.publish_direct_cmd_vel' in ui

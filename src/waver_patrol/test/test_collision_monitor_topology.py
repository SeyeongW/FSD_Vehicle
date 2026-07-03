from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_current_real_topology_uses_safety_mux_as_final_cmd_vel_gate():
    cmd_check = text("src/waver_patrol/scripts/waver_cmd_chain_check.sh")
    readiness = text("scripts/waver_field_readiness_check.py")
    docs = text("docs/hardware_readiness_levels.md")

    assert "WAVER_ALLOW_COLLISION_MONITOR_FINAL" in cmd_check
    assert "final /cmd_vel publisher is not safety_cmd_mux_node" in cmd_check
    assert 'allowed_final_nodes = {"safety_cmd_mux_node"}' in readiness
    assert "WAVER_ALLOW_COLLISION_MONITOR_FINAL" in readiness
    assert "nav2_collision_monitor" in docs
    assert "future optional" in docs


def test_real_launches_do_not_enable_collision_monitor_by_default():
    indoor = text("src/waver_patrol/launch/waver_indoor_patrol_real.launch.py")
    real = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert '"enable_collision_monitor": "false"' in indoor
    assert 'DeclareLaunchArgument("enable_collision_monitor", default_value="false")' in real

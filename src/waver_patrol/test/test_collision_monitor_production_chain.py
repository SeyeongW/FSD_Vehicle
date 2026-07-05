from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_production_chain_can_route_safety_mux_through_collision_monitor():
    mission = text("src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py")
    real = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    production = text("src/waver_patrol/launch/bird_patrol_production.launch.py")
    collision_yaml = text("src/waver_patrol/config/collision_monitor_waver.yaml")

    assert "nav2_collision_monitor" in mission
    assert "collision_monitor" in mission
    assert "safety_cmd_vel_out_topic" in mission
    assert "/waver/cmd_vel_safety" in mission
    assert "cmd_vel_out_topic" in mission
    assert "enable_collision_monitor" in real
    assert "enable_collision_monitor" in production
    assert 'cmd_vel_in_topic: "/waver/cmd_vel_safety"' in collision_yaml
    assert 'cmd_vel_out_topic: "/cmd_vel"' in collision_yaml

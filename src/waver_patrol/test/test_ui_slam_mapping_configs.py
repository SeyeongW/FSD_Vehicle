from __future__ import annotations

import pathlib

import yaml


REPO = pathlib.Path(__file__).resolve().parents[3]


def test_ui_slam_gazebo_config_keeps_cmd_authority() -> None:
    path = REPO / "src" / "ugv_main" / "ugv_gazebo" / "param" / "ui_slam" / "gazebo.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    panel = data["waver_remote_panel"]["ros__parameters"]
    safety = data["safety_cmd_mux_node"]["ros__parameters"]
    workflow = data["mapping_workflow_manager_node"]["ros__parameters"]
    assert panel["publish_direct_cmd_vel"] is False
    assert panel["manual_cmd_vel_topic"] == "/waver/manual_cmd_vel"
    assert safety["cmd_vel_out_topic"] == "/cmd_vel"
    assert safety["nav2_cmd_topic"] == "/waver/cmd_vel_nav2"
    assert workflow["save_dir"] == "~/ugv_ws/FSD_Vehicle/maps"


def test_ui_slam_waypoints_are_4m_then_7m_square() -> None:
    path = REPO / "src" / "waver_patrol" / "waypoints" / "waver_4m_then_7m_square_patrol.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    waypoints = data["waypoints"]
    assert waypoints[0]["x"] == 4.0
    assert waypoints[0]["y"] == 0.0
    assert waypoints[1]["y"] - waypoints[4]["y"] == 7.0
    assert waypoints[1]["x"] - waypoints[2]["x"] == 7.0

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_remote_visualization_defaults_are_field_topics():
    launch = (ROOT / "src/waver_patrol/launch/remote_visualization.launch.py").read_text()
    rviz = (ROOT / "src/waver_patrol/rviz/waver_field_operator.rviz").read_text()
    rviz_script = (ROOT / "scripts/waver_field_rviz_start.sh").read_text()

    assert 'default_value="/scan_safety"' in launch
    assert 'default_value="/livox/lidar"' in launch
    assert 'default_value="map"' in launch
    assert "waver_field_operator.rviz" in launch
    assert "/scan_safety" in rviz
    assert "/livox/lidar" in rviz
    assert 'SCAN_TOPIC="${WAVER_SCAN_TOPIC:-/scan_safety}"' in rviz_script
    assert 'POINTCLOUD_TOPIC="${WAVER_POINTCLOUD_TOPIC:-/livox/lidar}"' in rviz_script


def test_operator_station_stays_local_only():
    operator = (ROOT / "scripts/waver_field_operator_station_start.sh").read_text()
    ui = (ROOT / "scripts/waver_field_local_ui_start.sh").read_text()
    panel = (ROOT / "src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py").read_text()

    assert "ros2 launch waver_patrol bird_patrol_production.launch.py" not in operator
    assert "publish_direct_cmd_vel:=false" in ui
    assert "/waver/manual_cmd_vel" in panel

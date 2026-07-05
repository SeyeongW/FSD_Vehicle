import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def test_local_operator_deps_checker_exists_and_masks_field_role():
    script = read("scripts/waver_check_local_operator_deps.py")
    assert "Local operator PC runs SSH, RViz, and waver_remote_panel only" in script
    assert "Secrets are intentionally not printed" in script
    result = subprocess.run(
        ["python3", "scripts/waver_check_local_operator_deps.py", "--check", "--json"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode in (0, 1), result.stdout + result.stderr
    assert "JETSON_PASS" not in result.stdout


def test_setup_local_pc_has_explicit_modes():
    script = read("scripts/waver_setup_local_pc.sh")
    assert "--check" in script
    assert "--install-minimal-ui" in script
    assert "--install-rviz" in script
    assert "--install-dev" in script
    assert "--require-ros" in script
    assert "waver_check_local_operator_deps.py" in script


def test_operator_station_checks_jetson_before_ui_by_default():
    script = read("scripts/waver_field_operator_station_start.sh")
    assert "WAVER_SKIP_JETSON_CHECK" in script
    assert "docker ps" in script
    assert "waver_field_local_ui_start.sh" in script
    assert "waver_field_rviz_start.sh" in script
    assert "safety_cmd_mux_node" in script
    assert "--rviz-only" in script
    assert "--ui-only" in script
    assert "--scan-topic" in script
    result = subprocess.run(
        ["bash", "scripts/waver_field_operator_station_start.sh", "--dry-run", "--rviz", "--ui"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OPERATOR_STATION_DRY_RUN=1" in result.stdout
    assert "waver_field_rviz_start.sh" in result.stdout
    assert "waver_field_local_ui_start.sh" in result.stdout


def test_rviz_launcher_is_visualization_only():
    script = read("scripts/waver_field_rviz_start.sh")
    assert "remote_visualization.launch.py" in script
    assert "waver_start_field_backend.sh" not in script
    assert "waver_base_driver" not in script
    assert "WAVER_SKIP_JETSON_CHECK" in script
    assert 'SCAN_TOPIC="${WAVER_SCAN_TOPIC:-/scan_safety}"' in script
    result = subprocess.run(
        [
            "bash",
            "scripts/waver_field_rviz_start.sh",
            "--dry-run",
            "--fixed-frame",
            "map",
            "--map-topic",
            "/map",
            "--scan-topic",
            "/scan_safety",
            "--pointcloud-topic",
            "/livox/lidar",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "DRY_RUN_CMD=" in result.stdout
    assert "/scan_safety" in result.stdout


def test_local_ui_dry_run_does_not_require_password_or_jetson():
    result = subprocess.run(
        ["bash", "scripts/waver_field_local_ui_start.sh", "--dry-run"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "LOCAL_UI_DRY_RUN=1" in result.stdout
    assert "DRY_RUN_UI_CMD=" in result.stdout
    assert "remote_bridge_enabled:=true" in result.stdout
    assert "masked" in result.stdout or "remote_bridge_password:=" not in result.stdout


def test_remote_visualization_launch_keeps_rviz_and_panel_independent():
    launch = read("src/waver_patrol/launch/remote_visualization.launch.py")
    assert "Local-operator launch only" in launch
    assert "waver_field_operator.rviz" in launch
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan_safety")' in launch
    assert '"scan_topic": LaunchConfiguration("scan_topic")' in launch
    assert '"pointcloud_topic": LaunchConfiguration("pointcloud_topic")' in launch
    assert 'DeclareLaunchArgument("use_rviz"' in launch
    assert 'DeclareLaunchArgument("use_operator_panel"' in launch
    assert "ExecuteProcess(" in launch
    assert "waver_remote_panel" in launch


def test_field_operator_rviz_uses_real_sensor_topics():
    rviz = read("src/waver_patrol/rviz/waver_field_operator.rviz")
    assert "/map" in rviz
    assert "/scan_safety" in rviz
    assert "/livox/lidar" in rviz
    assert "/waver/bird_target_marker" in rviz

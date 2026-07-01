from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_clean_graph_rejects_non_waver_cmd_vel_owner():
    script = text("src/waver_patrol/scripts/waver_clean_graph_check.sh")
    assert "publisher_nodes()" in script
    assert "/cmd_vel publisher is not the Waver safety gate" in script
    assert "grep -Ev '^safety_cmd_mux_node$'" in script


def test_cmd_chain_checks_publisher_nodes_not_subscribers():
    script = text("src/waver_patrol/scripts/waver_cmd_chain_check.sh")
    assert "publisher_nodes_from_file()" in script
    assert "PUB_NODES" in script
    assert "MODE_PUB_NODES" in script
    assert "WAVER_ALLOW_COLLISION_MONITOR_FINAL" in script
    assert "grep -q '^safety_cmd_mux_node$'" in script


def test_real_preflight_uses_real_livox_topic_and_publisher_nodes():
    script = text("src/waver_patrol/scripts/waver_real_preflight_check.sh")
    assert 'POINTCLOUD_TOPIC="${WAVER_POINTCLOUD_TOPIC:-${POINTCLOUD_TOPIC:-/livox/lidar}}"' in script
    assert 'check_hz "$POINTCLOUD_TOPIC" 3.0' in script
    assert 'publisher_nodes()' in script
    assert 'grep -q \'^safety_cmd_mux_node$\'' in script
    assert 'grep -q \'^mission_patrol_manager_node$\'' in script
    assert "WAVER_PREFLIGHT_REQUIRE_CAMERA" in script
    assert "WAVER_PREFLIGHT_REQUIRE_BIRD" in script
    assert "WAVER_PREFLIGHT_REQUIRE_BATTERY" in script
    assert "BATTERY_STALE_STOP" in script


def test_real_preflight_does_not_treat_normal_timeout_stop_as_fatal():
    script = text("src/waver_patrol/scripts/waver_real_preflight_check.sh")
    fatal_line = next(line for line in script.splitlines() if "grep -E 'EMERGENCY" in line)
    assert "STANDBY_STOP" not in fatal_line
    assert "AUTO_COMMAND_TIMEOUT_STOP" not in fatal_line
    assert "MANUAL_COMMAND_TIMEOUT_STOP" not in fatal_line
    assert "SCAN_.*STOP" in fatal_line


def test_keyboard_ctrl_defaults_to_manual_candidate_topic():
    keyboard = text("src/ugv_main/ugv_tools/ugv_tools/keyboard_ctrl.py")
    assert 'self.declare_parameter("cmd_vel_topic", "/waver/manual_cmd_vel")' in keyboard


def test_real_launch_expands_experiment_output_root_in_python():
    launch = text("src/waver_patrol/launch/waver_real_bird_autonomy.launch.py")
    assert 'default_experiment_output_root = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/experiment_results")' in launch
    assert 'default_value="$HOME/ros2_ws5/FSD_Vehicle/experiment_results"' not in launch


def test_cartographer_localization_uses_existing_package_name():
    launch = text("src/ugv_main/ugv_nav/launch/nav_bringup/cartographer_localization.launch.py")
    assert 'get_package_share_directory("ugv_cartographer")' not in launch
    assert 'default_value="cartographer"' in launch
    assert "cartographer_config_package" in launch


def test_real_lidar_nav_serial_auto_is_by_id_first_and_strict():
    script = text("scripts/waver_field_lidar_nav_backend_start.sh")
    assert "SERIAL_PORT_BY_ID_PATTERN" in script
    assert "multiple Waver by-id serial candidates" in script
    assert "SERIAL_PORT_ALLOW_TTYUSB_FALLBACK" in script
    assert "no Waver by-id serial port found" in script


def test_safety_mux_checks_ok_clear_after_hard_stop_logic():
    safety = text("src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py")
    scan_state = safety[safety.index("    def _scan_state") : safety.index("    def _adapter_degraded")]
    assert "self.scan.adapter_state.startswith(\"OK_CLEAR\")" not in scan_state
    assert scan_state.index("self.scan.finite_points") < scan_state.index("SCAN_HARD_STOP_FRONT")


def test_scan_quality_uses_indoor_env_names_and_adapter_strict_gate():
    script = text("src/waver_patrol/scripts/waver_scan_quality_check.sh")
    assert "WAVER_FRONT_SECTOR_DEG" in script
    assert "WAVER_REAR_SECTOR_DEG" in script
    assert "WAVER_HARD_STOP_DISTANCE_M" in script
    assert "WAVER_MIN_VALID_SCAN_POINTS" in script
    assert "ADAPTER_STATE_MISSING" in script
    assert "ADAPTER_STATE_SEEN" in script


def test_indoor_status_helper_does_not_publish_motion_commands():
    script = text("src/waver_patrol/scripts/waver_indoor_patrol_status.sh")
    assert "ros2 topic pub" not in script
    assert "/cmd_vel publisher must be safety_cmd_mux_node" in script
    assert "/waver/livox_scan_adapter_state" in script

import math

import pytest

pytest.importorskip("rclpy")

import rclpy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from waver_patrol.bridges.cmd_vel_to_json import CmdVelToJson
from waver_patrol.bridges.serial_cmd_vel_bridge import BridgeCore
from waver_patrol.comms.serial_json_client import FakeSerial, SerialJsonClient
from waver_patrol.control.auto_behavior_mux_node import AutoBehaviorMuxNode
from waver_patrol.control.target_body_tracker_node import TargetBodyTrackerNode
from waver_patrol.patrol.battery_return_manager_node import BatteryReturnManagerNode
from waver_patrol.safety.safety_cmd_mux_node import SafetyCmdMuxNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_auto_behavior_default_auto_prefers_target():
    node = AutoBehaviorMuxNode()
    try:
        node._target_active(Bool(data=True))
        selected, reason = node._select_behavior()
        assert selected == "TARGET_TRACK"
        assert reason == "aerial_target_active"
    finally:
        node.destroy_node()


def test_auto_behavior_patrol_ignores_target():
    node = AutoBehaviorMuxNode()
    try:
        node.mode = "PATROL"
        node._target_active(Bool(data=True))
        selected, reason = node._select_behavior()
        assert selected == "WAYPOINT_PATROL"
        assert "ignored" in reason
    finally:
        node.destroy_node()


def test_battery_voltage_critical_wins_even_if_percentage_high():
    node = BatteryReturnManagerNode()
    try:
        msg = BatteryState()
        msg.percentage = 0.90
        msg.voltage = 8.5
        node.battery_callback(msg)
        assert node._combined_state(node._percentage_state(), node._voltage_state()) == "CRITICAL_RETURN_HOME"
    finally:
        node.destroy_node()


def test_target_left_positive_angular_convention():
    node = TargetBodyTrackerNode()
    try:
        bearing_left = math.atan2(1.0, 3.0)
        bearing_right = math.atan2(-1.0, 3.0)
        assert bearing_left > 0.0
        assert bearing_right < 0.0
    finally:
        node.destroy_node()


def test_safety_estop_forces_zero_cmd():
    node = SafetyCmdMuxNode()
    try:
        node.estop = True
        cmd, state, immediate = node._select(node._now())
        assert immediate
        assert "EMERGENCY_STOP" in state
        assert cmd.linear.x == 0.0
        assert cmd.angular.z == 0.0
    finally:
        node.destroy_node()


def test_safety_scan_stale_stops_when_required():
    node = SafetyCmdMuxNode()
    try:
        assert node._scan_state(node._now(), (node.auto_cmd, "AUTO_PASS", False)) == "SCAN_STALE_STOP"
    finally:
        node.destroy_node()


def test_serial_bridge_timeout_sends_stop():
    fake = FakeSerial()
    client = SerialJsonClient(transport=fake)
    core = BridgeCore(client, CmdVelToJson(), rate_hz=50.0, cmd_timeout_s=0.01, stop_repeat=1)
    core.update_twist(0.2, 0.0)
    core.last_cmd_time -= 1.0
    command = core._safe_command()
    assert command.is_stop
    assert "TIMEOUT" in core.state()


def test_serial_bridge_estop_sends_stop():
    fake = FakeSerial()
    client = SerialJsonClient(transport=fake)
    core = BridgeCore(client, CmdVelToJson(), rate_hz=50.0, cmd_timeout_s=0.3, stop_repeat=1)
    core.update_twist(0.2, 0.0)
    core.set_estop(True)
    command = core._safe_command()
    assert command.is_stop
    assert "EMERGENCY_STOP" in core.state()

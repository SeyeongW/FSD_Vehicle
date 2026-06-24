import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.ros_required

import rclpy
from std_msgs.msg import String

from waver_patrol.mission.target_departure_monitor_node import TargetDepartureMonitorNode, normalize_mission_state


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_departure_requires_action_done_before_range():
    node = TargetDepartureMonitorNode()
    try:
        node.range_m = 10.0
        departed, state = node.evaluate()
        assert not departed
        assert "WAITING" in state
        node.action_done_time = node._now() - 2.0
        departed, state = node.evaluate()
        assert departed
        assert "TARGET_DEPARTED" in state
    finally:
        node.destroy_node()


def test_departure_lost_timeout():
    node = TargetDepartureMonitorNode()
    try:
        node.action_done_time = node._now() - 2.0
        node.last_target_time = node._now() - 5.0
        node.lidar_state = "LOST"
        departed, state = node.evaluate()
        assert departed
        assert "TARGET_LOST_TIMEOUT" in state
    finally:
        node.destroy_node()


def test_normalize_mission_state_uses_first_token():
    assert normalize_mission_state("SOUND_TASK_DONE mode=AUTO nav_goal=NONE") == "SOUND_TASK_DONE"
    assert normalize_mission_state(" WAIT_TARGET_DEPARTURE mode=AUTO") == "WAIT_TARGET_DEPARTURE"
    assert normalize_mission_state("") == "UNKNOWN"


def test_mission_state_suffix_sets_and_preserves_action_done_time():
    node = TargetDepartureMonitorNode()
    try:
        node.mission_state_callback(String(data="SOUND_TASK_DONE mode=AUTO nav_goal=NONE"))
        first_done_time = node.action_done_time
        assert first_done_time > 0.0

        node.mission_state_callback(String(data="WAIT_TARGET_DEPARTURE mode=AUTO"))
        assert node.action_done_time == first_done_time

        node.mission_state_callback(String(data="PATROL_NAVIGATING mode=AUTO"))
        assert node.action_done_time == 0.0
    finally:
        node.destroy_node()


def test_departure_range_after_min_wait_publishes_true_state():
    node = TargetDepartureMonitorNode()
    try:
        node.action_done_time = node._now() - 2.0
        node.range_m = 8.5
        departed, state = node.evaluate()
        assert departed
        assert "TARGET_DEPARTED" in state
    finally:
        node.destroy_node()

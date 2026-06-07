import pytest

pytest.importorskip("rclpy")

import rclpy

from waver_patrol.mission.target_departure_monitor_node import TargetDepartureMonitorNode


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

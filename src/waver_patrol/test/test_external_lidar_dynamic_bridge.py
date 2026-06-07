import pytest

pytest.importorskip("rclpy")

import rclpy
from geometry_msgs.msg import PointStamped

from waver_patrol.perception.external_lidar_dynamic_bridge_node import ExternalLidarDynamicBridgeNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def make_point(z=3.5, x=4.0, y=0.0):
    msg = PointStamped()
    msg.header.frame_id = "map"
    msg.point.x = x
    msg.point.y = y
    msg.point.z = z
    return msg


def test_lidar_bridge_separates_height_and_range():
    node = ExternalLidarDynamicBridgeNode()
    try:
        target = node.evaluate_target(make_point(z=3.2, x=4.0, y=3.0))
        assert target is not None
        assert target.valid
        assert target.height_m == pytest.approx(3.2)
        assert target.range_m == pytest.approx(5.0)
        assert "LOCKED" in target.state
    finally:
        node.destroy_node()


def test_lidar_bridge_rejects_low_height():
    node = ExternalLidarDynamicBridgeNode()
    try:
        target = node.evaluate_target(make_point(z=1.0, x=10.0, y=0.0))
        assert target is not None
        assert not target.valid
        assert "height_low" in target.state
    finally:
        node.destroy_node()

import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.ros_required

import rclpy
from geometry_msgs.msg import PointStamped

from waver_patrol.control.live_target_aim_bridge_node import LiveTargetAimBridgeNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_live_target_aim_only_observation_state():
    node = LiveTargetAimBridgeNode()
    try:
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.point.x = 3.0
        msg.point.z = 3.2
        node.target_callback(msg)
        node.lidar_state = "LOCKED"
        node.mission_state = "PATROL_NAVIGATING"
        assert not node.allowed_to_publish()[0]
        node.mission_state = "CAMERA_ALIGN_TO_TARGET"
        assert node.allowed_to_publish()[0]
        node.last_target_time -= 10.0
        ok, reason = node.allowed_to_publish()
        assert not ok
        assert "STALE" in reason
    finally:
        node.destroy_node()

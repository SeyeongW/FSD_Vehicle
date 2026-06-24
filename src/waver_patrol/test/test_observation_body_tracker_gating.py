import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.ros_required

import rclpy
from geometry_msgs.msg import PointStamped

from waver_patrol.control.target_observation_body_tracker_node import TargetObservationBodyTrackerNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_body_tracker_gated_to_observation():
    node = TargetObservationBodyTrackerNode()
    try:
        target = PointStamped()
        target.header.frame_id = "base_link"
        target.point.x = 3.0
        target.point.y = 1.0
        node.target_callback(target)
        node.moving_valid = True
        node.mode = "AUTO"
        node.mission_state = "PATROL_NAVIGATING"
        cmd, centered, state = node.compute_command()
        assert cmd.angular.z == 0.0
        assert "BLOCKED" in state
        node.mission_state = "CAMERA_ALIGN_TO_TARGET"
        cmd, centered, state = node.compute_command()
        assert cmd.angular.z > 0.0
        assert not centered
    finally:
        node.destroy_node()

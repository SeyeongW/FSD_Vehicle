import math

import pytest

pytest.importorskip("geometry_msgs")
pytestmark = pytest.mark.ros_required

from geometry_msgs.msg import PoseStamped

from waver_patrol.mission.mission_utils import offset_goal_from_target, pose_stamped, quaternion_to_yaw
from waver_patrol.mission.mission_patrol_manager_node import MissionPatrolManagerNode, NavGoalType
from waver_patrol.mission.radar_command_bridge_node import RadarMetrics


def test_offset_goal_stops_before_target():
    target = pose_stamped("map", 4.0, 0.0, 0.0, 0.6)
    goal = offset_goal_from_target(target, 1.5)
    assert goal.header.frame_id == "map"
    assert math.isclose(goal.pose.position.x, 2.5, abs_tol=1e-6)
    assert math.isclose(goal.pose.position.y, 0.0, abs_tol=1e-6)
    assert math.isclose(quaternion_to_yaw(goal.pose.orientation), 0.0, abs_tol=1e-6)


def test_offset_goal_faces_target_bearing():
    target = pose_stamped("map", 0.0, 3.0, 0.0, 0.7)
    goal = offset_goal_from_target(target, 1.0)
    assert math.isclose(goal.pose.position.y, 2.0, abs_tol=1e-6)
    assert math.isclose(quaternion_to_yaw(goal.pose.orientation), math.pi / 2.0, abs_tol=1e-6)


def test_radar_metrics_dataclass_defaults_safe():
    metrics = RadarMetrics()
    assert math.isnan(metrics.range_m)
    assert math.isnan(metrics.doppler_mps)


def test_pose_stamped_finite_fields():
    pose = pose_stamped("map", 1.0, -2.0, 0.3, 0.5)
    assert isinstance(pose, PoseStamped)
    assert pose.header.frame_id == "map"
    assert pose.pose.position.z == 0.5


def test_stale_patrol_nav2_callback_ignored_after_target_interrupt():
    node = MissionPatrolManagerNode.__new__(MissionPatrolManagerNode)
    node.active_goal_sequence = 7
    node.active_goal_type = NavGoalType.TARGET_INSPECTION

    assert node.stale_nav2_callback(6, NavGoalType.PATROL)
    assert not node.stale_nav2_callback(7, NavGoalType.TARGET_INSPECTION)


def test_stale_accepted_patrol_goal_is_canceled_after_target_interrupt():
    class FakeGoalHandle:
        accepted = True

        def __init__(self):
            self.cancel_requested = False

        def cancel_goal_async(self):
            self.cancel_requested = True

    class FakeFuture:
        def __init__(self, result):
            self._result = result

        def result(self):
            return self._result

    node = MissionPatrolManagerNode.__new__(MissionPatrolManagerNode)
    node.active_goal_sequence = 7
    node.active_goal_type = NavGoalType.TARGET_INSPECTION
    events = []
    node.publish_event = lambda event, detail: events.append((event, detail))

    stale_patrol_handle = FakeGoalHandle()
    node.goal_response_callback(FakeFuture(stale_patrol_handle), 6, NavGoalType.PATROL)

    assert stale_patrol_handle.cancel_requested
    assert events
    assert events[0][0] == "STALE_NAV2_GOAL_ACCEPTED_CANCEL_REQUESTED"

import math

from geometry_msgs.msg import PoseStamped

from waver_patrol.mission.mission_utils import offset_goal_from_target, pose_stamped, quaternion_to_yaw
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

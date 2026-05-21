import math

from waver_patrol.safety.collision_guard import CollisionConfig, CollisionGuard, ScanLike


def _scan(front_distance=2.0, stamp=0.0):
    ranges = [2.0] * 360
    ranges[180] = front_distance
    return ScanLike(ranges, angle_min=-math.pi, angle_increment=math.radians(1.0), stamp=stamp)


def test_collision_hard_stop():
    guard = CollisionGuard(CollisionConfig(min_valid_scan_points=10))
    decision = guard.evaluate(_scan(0.3), now=0.1)
    assert decision.action == "HARD_STOP"
    assert decision.should_stop


def test_scan_stale_stop():
    decision = CollisionGuard().evaluate(_scan(2.0, stamp=0.0), now=1.0)
    assert decision.action == "SENSOR_STALE"

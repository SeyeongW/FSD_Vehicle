from waver_patrol.safety.command import TwistCommand, WheelCommand
from waver_patrol.safety.speed_limiter import SpeedLimiter


def test_patrol_linear_speed_limit():
    limited = SpeedLimiter().limit_twist(TwistCommand(1.0, 2.0), mode="patrol")
    assert limited.linear_x == 0.20
    assert limited.angular_z == 0.65


def test_wheel_speed_scales():
    limited = SpeedLimiter().limit_wheels(WheelCommand(0.6, 0.3, source="manual"), max_abs=0.3)
    assert abs(limited.left) <= 0.3
    assert abs(limited.right) <= 0.3

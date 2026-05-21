from waver_patrol.safety.acceleration_limiter import AccelerationLimiter, AccelerationLimiterConfig
from waver_patrol.safety.command import WheelCommand


def test_delta_limited():
    limiter = AccelerationLimiter(AccelerationLimiterConfig(max_delta_per_tick=0.04))
    result = limiter.limit(WheelCommand(0.5, 0.5, source="manual"))
    assert result.command.left == 0.04
    assert result.command.right == 0.04
    assert result.events


def test_reverse_requires_neutral():
    limiter = AccelerationLimiter(AccelerationLimiterConfig(max_delta_per_tick=1.0))
    limiter.limit(WheelCommand(0.2, 0.2, source="manual"))
    result = limiter.limit(WheelCommand(-0.2, -0.2, source="manual"))
    assert result.inserted_neutral
    assert result.command.is_stop

from waver_patrol.safety.command import TwistCommand, WheelCommand
from waver_patrol.safety.runaway_guard import RunawayAction, RunawayGuard


def test_jump_limited():
    decision = RunawayGuard().evaluate(WheelCommand(0.5, 0.5, source="manual"))
    assert decision.action == RunawayAction.ACCEL_LIMIT
    assert decision.command.left == 0.04


def test_reverse_limited_or_neutral():
    guard = RunawayGuard()
    guard.evaluate(WheelCommand(0.2, 0.2, source="manual"))
    decision = guard.evaluate(WheelCommand(-0.5, -0.5, source="manual"))
    assert decision.command.is_stop or abs(decision.command.left) <= 0.04


def test_nan_command_estop():
    decision = RunawayGuard().evaluate_raw(float("nan"), 0.0, source="manual")
    assert decision.action == RunawayAction.EMERGENCY_STOP


def test_cmd_vel_spike_estop():
    decision = RunawayGuard().evaluate(
        WheelCommand(0.0, 0.0, source="nav2"),
        twist=TwistCommand(2.0, 0.0, source="nav2"),
    )
    assert decision.action == RunawayAction.EMERGENCY_STOP


def test_timestamp_stale_soft_stop():
    decision = RunawayGuard().evaluate(WheelCommand(0.1, 0.1, source="manual", timestamp=0.0), now=1.0)
    assert decision.action == RunawayAction.SOFT_STOP


def test_serial_reconnect_blocks_old_command():
    decision = RunawayGuard().evaluate(WheelCommand(0.1, 0.1, source="manual"), serial_reconnecting=True)
    assert decision.action == RunawayAction.SOFT_STOP

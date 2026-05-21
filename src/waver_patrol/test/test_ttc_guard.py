from waver_patrol.safety.ttc_guard import TtcGuard


def test_ttc_hard_stop():
    guard = TtcGuard()
    guard.update(2.0, now=0.0)
    decision = guard.update(0.5, now=1.0)
    assert decision.action == "EMERGENCY_STOP"
    assert decision.dynamic_obstacle

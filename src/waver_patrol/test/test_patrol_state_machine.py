from waver_patrol.patrol.patrol_state_machine import PatrolState, PatrolStateMachine


def test_manual_override_blocks_patrol():
    sm = PatrolStateMachine()
    assert sm.manual_override() == PatrolState.MANUAL_OVERRIDE
    assert sm.reset_manual(localization_ok=True) == PatrolState.PATROL_READY


def test_recovery_attempts_estop():
    sm = PatrolStateMachine(max_recovery_attempts=1)
    assert sm.recovery_failed() == PatrolState.RECOVERY
    assert sm.recovery_failed() == PatrolState.EMERGENCY_STOP

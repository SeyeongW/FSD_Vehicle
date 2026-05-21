from waver_patrol.planning.avoidance_path_generator import AvoidanceAction, AvoidanceInput, AvoidancePathGenerator
from waver_patrol.safety.collision_guard import SectorState


def test_left_arc_when_front_blocked_and_left_clear():
    sectors = {
        "front": SectorState(0.2, 0.2, 10),
        "front_left": SectorState(2.0, 2.0, 10),
        "left": SectorState(2.0, 2.0, 10),
        "front_right": SectorState(0.8, 0.8, 10),
        "right": SectorState(0.8, 0.8, 10),
        "rear": SectorState(2.0, 2.0, 10),
    }
    decision = AvoidancePathGenerator().decide(AvoidanceInput(sectors))
    assert decision.action == AvoidanceAction.ARC_LEFT


def test_recovery_attempts_exceeded_estop():
    decision = AvoidancePathGenerator().decide(AvoidanceInput({}, recovery_attempts=4, max_recovery_attempts=3))
    assert decision.action == AvoidanceAction.EMERGENCY_STOP

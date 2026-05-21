from waver_patrol.safety.collision_guard import CollisionDecision
from waver_patrol.safety.command import WheelCommand
from waver_patrol.safety.safety_supervisor import SafetySupervisor


def test_supervisor_collision_forces_stop():
    supervisor = SafetySupervisor()
    supervisor.update_command(WheelCommand(0.1, 0.1, source="manual"))
    decision = supervisor.evaluate(collision=CollisionDecision("HARD_STOP", {}))
    assert decision.command.is_stop


def test_supervisor_manual_source_selected():
    supervisor = SafetySupervisor()
    supervisor.update_command(WheelCommand(0.1, 0.1, source="nav2"))
    supervisor.update_command(WheelCommand(0.0, 0.1, source="manual"))
    decision = supervisor.evaluate()
    assert decision.command.source == "manual"

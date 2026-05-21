from waver_patrol.safety.command import WheelCommand
from waver_patrol.safety.command_mux import CommandMux


def test_manual_override_beats_nav2():
    mux = CommandMux()
    mux.submit(WheelCommand(0.1, 0.1, source="nav2"))
    mux.submit(WheelCommand(0.0, 0.2, source="manual"))
    result = mux.select()
    assert result.command.source == "manual"


def test_empty_mux_stops():
    assert CommandMux().select().command.is_stop

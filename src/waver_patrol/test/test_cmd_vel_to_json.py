from waver_patrol.bridges.cmd_vel_to_json import CmdVelToJson, CmdVelToJsonConfig
from waver_patrol.safety.command import TwistCommand


def test_cmd_vel_to_left_right():
    converter = CmdVelToJson(CmdVelToJsonConfig(linear_gain=1.0, angular_gain=0.5))
    command = converter.convert(TwistCommand(0.1, 0.0, source="nav2"))
    assert command.left > 0.0
    assert command.right > 0.0


def test_cmd_vel_turn_mapping():
    converter = CmdVelToJson(CmdVelToJsonConfig(linear_gain=1.0, angular_gain=0.5))
    command = converter.convert(TwistCommand(0.0, 0.2, source="nav2"))
    assert command.left < command.right

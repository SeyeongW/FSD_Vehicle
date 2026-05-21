import math

from waver_patrol.safety.command_sanitizer import CommandSanitizer, SanitizerConfig


def test_command_clamp_and_metadata():
    result = CommandSanitizer().sanitize(1.0, -1.0, source="manual", reason="test")
    assert result.valid
    assert result.command.left == 0.28
    assert result.command.right == -0.28
    assert result.command.source == "manual"
    assert result.events


def test_nan_inf_rejected_to_stop():
    result = CommandSanitizer().sanitize(math.nan, 0.1, source="manual")
    assert not result.valid
    assert result.command.is_stop
    assert result.command.source == "emergency_stop"


def test_stop_ignores_invert_and_swap():
    sanitizer = CommandSanitizer(SanitizerConfig(invert_left=True, invert_right=True, swap_left_right=True))
    result = sanitizer.sanitize(0.2, -0.2, source="manual", is_stop=True)
    assert result.command.left == 0.0
    assert result.command.right == 0.0

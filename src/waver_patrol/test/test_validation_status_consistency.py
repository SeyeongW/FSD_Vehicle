from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "src/waver_patrol/scripts/validation_status_utils.py"
spec = importlib.util.spec_from_file_location("validation_status_utils", SCRIPT)
status_utils = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(status_utils)


def test_timeout_after_expected_success_markers_promotes_to_explicit_pass() -> None:
    text = "\n".join(
        [
            "Spawn status: SpawnEntity: Successfully spawned",
            "keyboard_ctrl process has finished cleanly",
            "COMMAND_RC=124",
            "STATUS=SKIP_WITH_REASON timeout_before_scenario_finished",
        ]
    )
    result = status_utils.classify_validation_log(text, "keyboard_teleop_smoke")
    assert result["raw_status"].startswith("SKIP_WITH_REASON")
    assert result["final_status"] == "PASS"
    assert result["limitation"] == "timeout_after_expected_success_markers"


def test_fatal_pattern_cannot_be_pass() -> None:
    text = "\n".join(
        [
            "COMMAND_RC=0",
            "STATUS=PASS",
            "Failed to load plugin libros2_livox.so",
        ]
    )
    result = status_utils.classify_validation_log(text, "livox_mid360_scenario")
    assert result["final_status"] == "FAIL"


def test_livox_plugin_fallback_is_not_silent_pass() -> None:
    text = "\n".join(
        [
            "COMMAND_RC=0",
            "STATUS=PASS",
            "Failed to load plugin libros2_livox.so",
            "scan-mapper fallback active",
        ]
    )
    result = status_utils.classify_validation_log(text, "slam_mapping_smoke")
    assert result["final_status"] == "SKIP_WITH_REASON"
    assert "livox_plugin_missing" in result["limitation"]

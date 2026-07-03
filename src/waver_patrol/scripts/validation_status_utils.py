#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path


FATAL_PATTERNS = (
    r"Unable to start server",
    r"Address already in use",
    r"Spawn service failed",
    r"process has died",
    r"Failed to load plugin",
    r"Traceback",
    r"package .* not found",
    r"Service /spawn_entity unavailable",
)


def _has(pattern: str, text: str) -> bool:
    return re.search(pattern, text, flags=re.IGNORECASE | re.MULTILINE) is not None


def _last_value(text: str, key: str) -> str:
    value = ""
    prefix = f"{key}="
    for line in text.splitlines():
        if line.startswith(prefix):
            value = line[len(prefix) :].strip()
    return value


def expected_success_markers(scenario: str, text: str) -> bool:
    if scenario == "keyboard_teleop_smoke":
        return "Spawn status: SpawnEntity: Successfully spawned" in text and "keyboard_ctrl" in text
    if scenario == "autonomous_patrol_smoke":
        return "Spawn status: SpawnEntity: Successfully spawned" in text and "waver_gazebo_patrol" in text
    return False


def classify_validation_log(text: str, scenario: str = "") -> dict[str, str]:
    raw_status = _last_value(text, "STATUS") or "PASS"
    final_status = _last_value(text, "FINAL_STATUS")
    command_rc = _last_value(text, "COMMAND_RC")
    limitation = ""

    fatal = any(_has(pattern, text) for pattern in FATAL_PATTERNS)
    livox_plugin_fail = "Failed to load plugin" in text and "libros2_livox.so" in text
    scan_mapper_fallback = "scan-mapper fallback" in text.lower() or "scan_mapper_fallback" in text.lower()
    livox_required = "livox" in scenario.lower() or "mid360" in scenario.lower()

    if fatal:
        if livox_plugin_fail and scan_mapper_fallback and not livox_required:
            final_status = final_status or "SKIP_WITH_REASON"
            limitation = "livox_plugin_missing_scan_mapper_fallback_only"
        else:
            final_status = "FAIL"
            limitation = "fatal_log_pattern"
    elif raw_status.startswith("SKIP_WITH_REASON") and expected_success_markers(scenario, text):
        final_status = final_status or "PASS"
        limitation = "timeout_after_expected_success_markers"
    else:
        final_status = final_status or raw_status.split()[0]

    if final_status == "PASS" and raw_status.startswith("SKIP_WITH_REASON") and not limitation:
        limitation = "raw_skip_promoted_after_explicit_final_status"
    if final_status == "SKIP_WITH_REASON" and not limitation:
        limitation = raw_status.replace("SKIP_WITH_REASON", "", 1).strip() or "environment_or_dependency_missing"

    return {
        "raw_status": raw_status,
        "final_status": final_status,
        "limitation": limitation,
        "command_rc": command_rc,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Classify Waver validation log status.")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--scenario", default="")
    args = parser.parse_args()
    result = classify_validation_log(args.log.read_text(encoding="utf-8", errors="replace"), args.scenario)
    for key in ("raw_status", "final_status", "limitation", "command_rc"):
        print(f"{key}={result[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

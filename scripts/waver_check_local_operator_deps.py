#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CORE_COMMANDS = ("bash", "git", "python3", "ssh", "scp", "rsync")
MINIMAL_COMMANDS = CORE_COMMANDS + ("sshpass",)
RVIZ_COMMANDS = MINIMAL_COMMANDS + ("ros2", "rviz2")
DEV_COMMANDS = RVIZ_COMMANDS + ("colcon",)
MINIMAL_MODULES = ("paramiko", "tkinter")
DEV_MODULES = MINIMAL_MODULES + ("pytest",)


def command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def module_exists(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def check(commands: tuple[str, ...], modules: tuple[str, ...], require_ros: bool) -> dict[str, object]:
    missing_commands = [name for name in commands if not command_exists(name)]
    missing_modules = [name for name in modules if not module_exists(name)]
    ros_setup = Path("/opt/ros/humble/setup.bash")
    if require_ros and not ros_setup.exists():
        missing_commands.append("/opt/ros/humble/setup.bash")
    status = "PASS" if not missing_commands and not missing_modules else "FAIL"
    return {
        "status": status,
        "workspace": str(ROOT),
        "missing_commands": missing_commands,
        "missing_python_modules": missing_modules,
        "ros_humble_setup_exists": ros_setup.exists(),
        "notes": [
            "Local operator PC runs SSH, RViz, and waver_remote_panel only.",
            "Jetson Docker owns Nav2, SLAM, Livox/camera/base drivers, and final /cmd_vel.",
            "Secrets are intentionally not printed by this checker.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Check local Waver operator station dependencies.")
    parser.add_argument("--check", action="store_true", help="alias for --mode minimal-ui")
    parser.add_argument("--mode", choices=("minimal-ui", "rviz", "dev"), default="minimal-ui")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument("--require-ros", action="store_true", help="require /opt/ros/humble/setup.bash")
    args = parser.parse_args()
    mode = "minimal-ui" if args.check else args.mode
    if mode == "dev":
        payload = check(DEV_COMMANDS, DEV_MODULES, args.require_ros)
    elif mode == "rviz":
        payload = check(RVIZ_COMMANDS, MINIMAL_MODULES, True)
    else:
        payload = check(MINIMAL_COMMANDS, MINIMAL_MODULES, args.require_ros)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"WAVER_LOCAL_OPERATOR_DEPS={payload['status']}")
        if payload["missing_commands"]:
            print("MISSING_COMMANDS=" + ",".join(payload["missing_commands"]))
        if payload["missing_python_modules"]:
            print("MISSING_PYTHON_MODULES=" + ",".join(payload["missing_python_modules"]))
        print(f"ROS_HUMBLE_SETUP_EXISTS={payload['ros_humble_setup_exists']}")
        if payload["status"] != "PASS":
            print("INSTALL_HINT=bash scripts/waver_setup_local_pc.sh --install-minimal-ui")
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

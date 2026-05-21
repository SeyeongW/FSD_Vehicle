from __future__ import annotations

import subprocess

from waver_patrol.comms.port_conflict_detector import detect_process_conflicts, lsof_port
from waver_patrol.comms.port_detect import detect_ports


def _run(command: list[str]) -> str:
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    return result.stdout.strip() or result.stderr.strip()


def main() -> None:
    print("Waver stack inspection")
    print("ROS packages:")
    print(_run(["bash", "-lc", "source /opt/ros/humble/setup.bash 2>/dev/null; ros2 pkg list | grep -E 'ugv|nav2|slam|cartographer|rtabmap|ldlidar|usb_cam|tf2|waypoint' || true"]))
    print("Topics:")
    print(_run(["bash", "-lc", "source /opt/ros/humble/setup.bash 2>/dev/null; ros2 topic list || true"]))
    print("Detected serial ports:", ", ".join(detect_ports()) or "none")
    print("Process conflicts:")
    print("\n".join(detect_process_conflicts()) or "none")
    for port in ["/dev/ttyTHS0", "/dev/ttyUSB0", "/dev/ttyACM0"]:
        owners = lsof_port(port)
        print(f"{port}: {'; '.join(owners) if owners else 'free or missing'}")

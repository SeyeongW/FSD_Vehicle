#!/usr/bin/env bash
set -euo pipefail
cd "${WAVER_WS:-$HOME/ros2_ws}"
pwd
ls -la
find . -maxdepth 4 -type f | sort | head -300
ls -la /home/ws || true
ls -la /home/ws/ugv_ws || true
ls -la /home/jetson/ugv_ws || true
find "$HOME/ros2_ws/src/FSD_Vehicle/src" -maxdepth 5 -type f 2>/dev/null | sort | grep -E "slam|nav|bringup|keyboard|launch|rtabmap|cartographer|gmapping|amcl|emcl|teb|dwa|waypoint|behavior|base|cmd_vel" || true
bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null; source $HOME/ros2_ws/install/setup.bash 2>/dev/null; ros2 pkg list | grep -E 'ugv|nav2|slam|cartographer|rtabmap|ldlidar|usb_cam|depthai|tf2|robot_localization|waypoint' || true"
bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null; source $HOME/ros2_ws/install/setup.bash 2>/dev/null; ros2 topic list || true"
bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null; source $HOME/ros2_ws/install/setup.bash 2>/dev/null; ros2 node list || true"
ps aux | grep -E "app.py|ros2|bringup|ugv_driver|base|serial|ttyTHS0|ttyUSB|ttyACM" | grep -v grep || true
lsof /dev/ttyTHS0 2>/dev/null || true
lsof /dev/ttyUSB0 2>/dev/null || true
lsof /dev/ttyACM0 2>/dev/null || true

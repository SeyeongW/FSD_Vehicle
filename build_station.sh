#!/bin/bash
# ============================================================
# Raspberry Pi control-station build (hardware only)
# ------------------------------------------------------------
# Builds ONLY the real-hardware packages the control station needs:
#   - ugv_lidar_detection  (clustering / motion tracking / click-select)
#   - ugv_vision           (SIYI gimbal controller + YOLO nodes)
# ...plus their in-workspace dependencies (via --packages-up-to).
#
# Simulation / desktop packages are intentionally NOT built (the Pi image is
# headless and has no gazebo/nav2/slam deps).
# ============================================================
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[build_station] workspace: $WS_ROOT"
cd "$WS_ROOT"

source /opt/ros/humble/setup.bash

# ── 1. Build hardware packages + their workspace deps only ──
colcon build --symlink-install --packages-up-to \
    ugv_lidar_detection \
    ugv_vision

# ── 2. Auto-source for future shells ──
SETUP_BASH="$WS_ROOT/install/setup.bash"
grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "source $SETUP_BASH" ~/.bashrc \
    || echo "source $SETUP_BASH" >> ~/.bashrc

source "$SETUP_BASH"
echo "[build_station] done. Station packages ready."

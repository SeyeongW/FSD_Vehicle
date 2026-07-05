#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

MODE="check"
REQUIRE_ROS=false
for arg in "$@"; do
  case "${arg}" in
    --check) MODE="check" ;;
    --install|--install-minimal-ui) MODE="install-minimal-ui" ;;
    --install-rviz) MODE="install-rviz" ;;
    --install-dev) MODE="install-dev" ;;
    --require-ros) REQUIRE_ROS=true ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/waver_setup_local_pc.sh [--check] [--require-ros] [--install-minimal-ui|--install-rviz|--install-dev]

Modes:
  --check              Check local UI/SSH dependencies only; ROS is optional.
  --check --require-ros
                       Check local UI/SSH dependencies and ROS/RViz availability.
  --install-minimal-ui Install packages needed for SSH bridge + remote panel.
  --install-rviz       Install minimal UI packages plus RViz/ROS operator tools.
  --install-dev        Install RViz packages plus developer test tooling.

The local PC must not run the real robot backend. It runs SSH, RViz, and
waver_remote_panel. Jetson Docker owns Nav2, SLAM, Livox/camera/base drivers,
and final /cmd_vel.
EOF
      exit 0
      ;;
    *) echo "[SETUP_LOCAL][ERROR] unknown arg: ${arg}" >&2; exit 2 ;;
  esac
done

MINIMAL_APT_PACKAGES=(
  git
  openssh-client
  rsync
  sshpass
  python3-pip
  python3-tk
  python3-pygame
)

DEV_APT_PACKAGES=(
  python3-colcon-common-extensions
  python3-rosdep
  python3-pytest
  shellcheck
)

RVIZ_APT_PACKAGES=(
  ros-humble-rviz2
  ros-humble-rviz-common
)

install_python_requirements() {
  python3 -m pip install --user -r requirements-local-ui.txt
}

case "${MODE}" in
  check)
    if [ "${REQUIRE_ROS}" = "true" ]; then
      python3 scripts/waver_check_local_operator_deps.py --check --require-ros
    else
      python3 scripts/waver_check_local_operator_deps.py --check
    fi
    ;;
  install-minimal-ui)
    sudo apt update
    sudo apt install -y "${MINIMAL_APT_PACKAGES[@]}"
    install_python_requirements
    python3 scripts/waver_check_local_operator_deps.py --check
    ;;
  install-rviz)
    sudo apt update
    sudo apt install -y "${MINIMAL_APT_PACKAGES[@]}" "${RVIZ_APT_PACKAGES[@]}"
    install_python_requirements
    python3 scripts/waver_check_local_operator_deps.py --mode rviz --require-ros
    ;;
  install-dev)
    sudo apt update
    sudo apt install -y "${MINIMAL_APT_PACKAGES[@]}" "${RVIZ_APT_PACKAGES[@]}" "${DEV_APT_PACKAGES[@]}"
    install_python_requirements
    python3 scripts/waver_check_local_operator_deps.py --mode dev --require-ros
    ;;
esac

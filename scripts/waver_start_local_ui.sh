#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_LOCAL=false
for arg in "$@"; do
  case "$arg" in
    --build-local) BUILD_LOCAL=true ;;
    -h|--help) echo "Usage: bash scripts/waver_start_local_ui.sh [--build-local]"; exit 0 ;;
    *) echo "[WAVER_UI][ERROR] unknown arg: $arg" >&2; exit 2 ;;
  esac
done

# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[WAVER_UI][ERROR] /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi

if [ "${BUILD_LOCAL}" = "true" ] || [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[WAVER_UI] building local UI packages"
  cd "${ROOT}"
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-up-to ugv_tools waver_patrol
fi

python3 - <<'PY'
import importlib
for name in ("paramiko", "tkinter"):
    importlib.import_module(name)
PY

exec bash "${ROOT}/scripts/waver_field_local_ui_start.sh"

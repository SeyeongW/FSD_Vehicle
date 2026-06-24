#!/usr/bin/env bash
set -euo pipefail

INSTALL=false
for arg in "$@"; do
  case "$arg" in
    --install) INSTALL=true ;;
    -h|--help) echo "Usage: bash scripts/waver_setup_local_pc.sh [--install]"; exit 0 ;;
    *) echo "[SETUP_LOCAL][ERROR] unknown arg: $arg" >&2; exit 2 ;;
  esac
done

APT_PACKAGES=(
  git
  python3-pip
  python3-colcon-common-extensions
  python3-rosdep
  python3-tk
  python3-pygame
  openssh-client
  rsync
  sshpass
)

if [ "${INSTALL}" = "true" ]; then
  sudo apt update
  sudo apt install -y "${APT_PACKAGES[@]}"
  pip3 install -r requirements-local-ui.txt
  exit 0
fi

missing=()
for cmd in git python3 pip3 ssh scp rsync; do
  command -v "${cmd}" >/dev/null 2>&1 || missing+=("${cmd}")
done
python3 - <<'PY' >/tmp/waver_setup_local_pydeps.log 2>&1 || true
import importlib
for name in ("tkinter", "paramiko"):
    importlib.import_module(name)
    print(name)
PY
grep -q tkinter /tmp/waver_setup_local_pydeps.log || missing+=("python3-tk")
grep -q paramiko /tmp/waver_setup_local_pydeps.log || missing+=("paramiko")

if [ "${#missing[@]}" -gt 0 ]; then
  echo "WAVER_SETUP_LOCAL_PC=CHECK_FAILED"
  printf 'Missing: %s\n' "${missing[@]}"
  echo "Install with: bash scripts/waver_setup_local_pc.sh --install"
  exit 1
fi

echo "WAVER_SETUP_LOCAL_PC=PASS"

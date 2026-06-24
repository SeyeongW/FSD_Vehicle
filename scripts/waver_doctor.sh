#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"

CHECK_JETSON=false
CHECK_DOCKER=false
for arg in "$@"; do
  case "$arg" in
    --jetson) CHECK_JETSON=true ;;
    --docker) CHECK_DOCKER=true ;;
    --all) CHECK_JETSON=true; CHECK_DOCKER=true ;;
    -h|--help)
      echo "Usage: bash scripts/waver_doctor.sh [--jetson] [--docker] [--all]"
      exit 0
      ;;
  esac
done

missing=()
warnings=()
skipped=()

need_file() { [ -e "${ROOT}/$1" ] || missing+=("$1"); }
need_cmd() { command -v "$1" >/dev/null 2>&1 || missing+=("command:$1"); }
warn_cmd() { command -v "$1" >/dev/null 2>&1 || warnings+=("command:$1 not found"); }

need_file .env
need_file .env.example
need_file config/waver_field_env
need_file config/waver_field_env.example
need_file config/waver_field_env.local.example
need_file scripts/waver_field_env_load.sh
need_file scripts/waver_quickstart_field.sh
need_file scripts/waver_field_bootstrap_jetson.sh
need_file scripts/waver_start_field_backend.sh
need_file scripts/waver_start_local_ui.sh
need_file requirements-local-ui.txt

need_cmd bash
need_cmd git
need_cmd python3
need_cmd ssh
need_cmd scp
warn_cmd rsync
warn_cmd docker

if [ ! -f /opt/ros/humble/setup.bash ]; then
  warnings+=("/opt/ros/humble/setup.bash not found")
fi
command -v colcon >/dev/null 2>&1 || warnings+=("colcon not found")

python3 - <<'PY' >/tmp/waver_doctor_pydeps.log 2>&1 || true
import importlib
for name in ("paramiko", "tkinter"):
    importlib.import_module(name)
    print("PY_OK", name)
PY
if ! grep -q 'PY_OK paramiko' /tmp/waver_doctor_pydeps.log 2>/dev/null; then
  warnings+=("python paramiko missing; run: pip3 install -r requirements-local-ui.txt")
fi
if ! grep -q 'PY_OK tkinter' /tmp/waver_doctor_pydeps.log 2>/dev/null; then
  warnings+=("python tkinter missing; run: sudo apt install -y python3-tk")
fi

if ! waver_field_env_require >/tmp/waver_doctor_env.log 2>&1; then
  missing+=("field env required values")
  while IFS= read -r line; do warnings+=("${line}"); done < /tmp/waver_doctor_env.log
fi

env_domain="$(grep -E '^ROS_DOMAIN_ID=' "${ROOT}/.env" 2>/dev/null | tail -1 | cut -d= -f2- || true)"
field_domain="${ROS_DOMAIN_ID:-}"
if [ -n "${env_domain}" ] && [ "${env_domain}" != "${field_domain}" ]; then
  missing+=("ROS_DOMAIN_ID mismatch: .env=${env_domain}, field=${field_domain}")
fi

for f in "${ROOT}/.env" "${ROOT}/.env.example" "${ROOT}/config/waver_field_env" "${ROOT}/config/waver_field_env.example"; do
  if ! waver_secret_scan_file "${f}"; then
    missing+=("secret-like value in ${f#${ROOT}/}")
  fi
done

if [ -n "${JETSON_PASS:-}" ]; then
  warnings+=("JETSON_PASS is set in an override file; value is masked and not printed")
fi

if [ "${CHECK_JETSON}" = "true" ]; then
  waver_ssh_cmd
  if "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "echo JETSON_SSH_OK; hostname; uname -m; test -d '$(dirname "${JETSON_WS}")' || mkdir -p '$(dirname "${JETSON_WS}")'; ls /dev/serial/by-id 2>/dev/null || true" >/tmp/waver_doctor_jetson.log 2>&1; then
    sed 's/^/[JETSON] /' /tmp/waver_doctor_jetson.log
  else
    missing+=("Jetson SSH unavailable: ${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22}")
    tail -20 /tmp/waver_doctor_jetson.log 2>/dev/null | sed 's/^/[JETSON][ERR] /' || true
  fi
else
  skipped+=("Jetson SSH checks; run bash scripts/waver_doctor.sh --jetson")
fi

if [ "${CHECK_DOCKER}" = "true" ]; then
  if command -v docker >/dev/null 2>&1; then
    docker ps >/tmp/waver_doctor_docker.log 2>&1 || warnings+=("docker client found but daemon unavailable")
  else
    skipped+=("Docker checks; docker command not found")
  fi
else
  skipped+=("Docker checks; run bash scripts/waver_doctor.sh --docker")
fi

echo "WAVER_FIELD_ENV_SUMMARY_BEGIN"
waver_field_env_masked_summary
echo "WAVER_FIELD_ENV_SUMMARY_END"

if [ "${#missing[@]}" -gt 0 ]; then
  echo "WAVER_DOCTOR_RESULT=FAIL"
  echo "Missing:"
  printf -- '- %s\n' "${missing[@]}"
  echo "Warnings:"
  printf -- '- %s\n' "${warnings[@]:-none}"
  echo "Next steps:"
  echo "- Edit config/waver_field_env or create config/waver_field_env.local"
  echo "- Install local deps: sudo apt install -y python3-colcon-common-extensions rsync openssh-client python3-tk"
  echo "- Install UI deps: pip3 install -r requirements-local-ui.txt"
  exit 1
fi

echo "Warnings:"
printf -- '- %s\n' "${warnings[@]:-none}"
echo "Skipped:"
printf -- '- %s\n' "${skipped[@]:-none}"
echo "WAVER_DOCTOR_RESULT=PASS"

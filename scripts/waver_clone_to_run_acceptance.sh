#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RELEASE_MODE=false

while [ "$#" -gt 0 ]; do
  case "$1" in
    --release-mode) RELEASE_MODE=true; shift ;;
    -h|--help)
      echo "Usage: bash scripts/waver_clone_to_run_acceptance.sh [--release-mode]"
      exit 0
      ;;
    *) echo "[ACCEPTANCE][ERROR] unknown arg: $1" >&2; exit 2 ;;
  esac
done

failed=()
warnings=()
skipped=()

need_file() { [ -e "${ROOT}/$1" ] || failed+=("missing $1"); }

for f in \
  .env.example \
  config/waver_field_env.example \
  config/waver_field_env.local.example \
  scripts/waver_field_env_load.sh \
  scripts/waver_doctor.sh \
  scripts/waver_quickstart_field.sh \
  scripts/waver_field_bootstrap_jetson.sh \
  scripts/waver_start_field_backend.sh \
  scripts/waver_start_local_ui.sh \
  scripts/waver_clone_to_run_acceptance.sh \
  scripts/waver_docker_env_check.sh \
  scripts/waver_setup_local_pc.sh \
  scripts/waver_field_lidar_nav_backend_start.sh \
  docs/CLONE_TO_FIELD.md \
  docs/SETUP_LOCAL_PC.md \
  docs/SETUP_JETSON.md \
  docs/TROUBLESHOOTING_FIELD.md \
  README_REAL_VEHICLE.md \
  requirements-local-ui.txt; do
  need_file "${f}"
done

# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"

if [ "${RELEASE_MODE}" = "true" ] && [ -e "${ROOT}/config/waver_field_env" ]; then
  failed+=("release includes private/shared config/waver_field_env")
fi

if [ "${RELEASE_MODE}" != "true" ]; then
  waver_field_env_require || warnings+=("field env required values missing; normal for clean clone before quickstart")
fi

for f in .env .env.example config/waver_field_env config/waver_field_env.example config/waver_field_env.local; do
  if [ -f "${ROOT}/${f}" ]; then
    waver_secret_scan_file "${ROOT}/${f}" || failed+=("secret scan failed: ${f}")
  fi
done

if rg -n 'WAVER_FIELD_ENV_FILE:-\\$HOME/\\.waver_field_env|source /opt/ros/humble/install/setup.bash &&|ROS_DOMAIN_ID=27' "${ROOT}" \
  -g '!build/**' -g '!install/**' -g '!log/**' -g '!build_docker/**' -g '!install_docker/**' -g '!log_docker/**' -g '!.git/**' \
  -g '!scripts/waver_clone_to_run_acceptance.sh' >/tmp/waver_acceptance_grep.log; then
  failed+=("legacy field env/source/domain patterns remain; see /tmp/waver_acceptance_grep.log")
fi

if rg -n '10\\.139\\.225\\.150|10\\.63\\.240\\.150|/home/sw|/home/chotaehyun' "${ROOT}/scripts" "${ROOT}/docker" \
  -g '!waver_clone_to_run_acceptance.sh' \
  -g '!waver_create_home_field_env.sh' >/tmp/waver_acceptance_personal_grep.log; then
  failed+=("personal values are hardcoded in scripts/docker; see /tmp/waver_acceptance_personal_grep.log")
fi

bash -n "${ROOT}"/scripts/*.sh || failed+=("bash syntax check failed")
python3 -m compileall "${ROOT}/scripts" "${ROOT}/src/waver_patrol" "${ROOT}/src/ugv_main/ugv_tools" >/tmp/waver_acceptance_compileall.log 2>&1 || failed+=("python compileall failed")

quickstart_args=(
  "${ROOT}/scripts/waver_quickstart_field.sh"
  --dry-run
  --force
  --jetson-host 192.0.2.10
  --jetson-user waver
  --jetson-ws /home/waver/ros2_ws5/FSD_Vehicle
  --container fsd_dev_jetson
  --serial-port /dev/serial/by-id/usb-WAVER_BASE_TEST
)
bash "${quickstart_args[@]}" >/tmp/waver_acceptance_quickstart.log 2>&1 || failed+=("quickstart dry-run failed; see /tmp/waver_acceptance_quickstart.log")

if command -v docker >/dev/null 2>&1; then
  (cd "${ROOT}" && docker compose -f docker-compose.jetson.yml config >/tmp/waver_acceptance_compose.log) || failed+=("docker compose config failed")
else
  skipped+=("docker compose config; docker not installed")
fi

if [ -x "${ROOT}/scripts/waver_contract_check.py" ]; then
  python3 "${ROOT}/scripts/waver_contract_check.py" >/tmp/waver_acceptance_contract.log 2>&1 || warnings+=("contract checker reported issues; see /tmp/waver_acceptance_contract.log")
fi

if [ "${#failed[@]}" -gt 0 ]; then
  echo "WAVER_CLONE_TO_RUN_ACCEPTANCE=FAIL"
  echo "Failed:"
  printf -- '- %s\n' "${failed[@]}"
  echo "Warnings:"
  printf -- '- %s\n' "${warnings[@]:-none}"
  echo "Skipped:"
  printf -- '- %s\n' "${skipped[@]:-none}"
  exit 1
fi

echo "Warnings:"
printf -- '- %s\n' "${warnings[@]:-none}"
echo "Skipped:"
printf -- '- %s\n' "${skipped[@]:-none}"
echo "WAVER_CLONE_TO_RUN_ACCEPTANCE=PASS"

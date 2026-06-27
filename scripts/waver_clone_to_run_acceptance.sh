#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
failed=()
warnings=()
skipped=()

need_file() { [ -e "${ROOT}/$1" ] || failed+=("missing $1"); }

for f in \
  .env \
  .env.example \
  config/waver_field_env \
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
  docs/CLONE_TO_FIELD.md \
  docs/SETUP_LOCAL_PC.md \
  docs/SETUP_JETSON.md \
  docs/TROUBLESHOOTING_FIELD.md \
  requirements-local-ui.txt; do
  need_file "${f}"
done

# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require || failed+=("field env required values")

env_domain="$(grep -E '^ROS_DOMAIN_ID=' "${ROOT}/.env" | tail -1 | cut -d= -f2-)"
field_domain="${ROS_DOMAIN_ID:-}"
[ "${env_domain}" = "${field_domain}" ] || failed+=("ROS_DOMAIN_ID mismatch")

for f in .env .env.example config/waver_field_env config/waver_field_env.example; do
  waver_secret_scan_file "${ROOT}/${f}" || failed+=("secret scan failed: ${f}")
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

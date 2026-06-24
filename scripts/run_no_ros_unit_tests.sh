#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export WAVER_ALLOW_HARDWARE=0
export WAVER_NO_HARDWARE=1
export WAVER_BLOCK_SERIAL=1
export WAVER_DISABLE_SOUND_OUTPUT=1
export PYTHONPATH="$ROOT/src/waver_patrol:$ROOT/src/waver_experiment_logger:$ROOT/src/waver_seo_tracking:${PYTHONPATH:-}"

python3 -m pytest -q \
  src/waver_patrol/test/test_acceleration_limiter.py \
  src/waver_patrol/test/test_avoidance_path_generator.py \
  src/waver_patrol/test/test_bird_classification_contract.py \
  src/waver_patrol/test/test_cmd_vel_to_json.py \
  src/waver_patrol/test/test_collision_guard.py \
  src/waver_patrol/test/test_command_mux.py \
  src/waver_patrol/test/test_command_sanitizer.py \
  src/waver_patrol/test/test_hardware_free_contract_docs.py \
  src/waver_patrol/test/test_patrol_route.py \
  src/waver_patrol/test/test_patrol_state_machine.py \
  src/waver_patrol/test/test_real_launch_static_contracts.py \
  src/waver_patrol/test/test_real_vehicle_contract_skeleton.py \
  src/waver_patrol/test/test_remote_ui_security_contract.py \
  src/waver_patrol/test/test_runaway_guard.py \
  src/waver_patrol/test/test_safety_supervisor.py \
  src/waver_patrol/test/test_serial_json_fake.py \
  src/waver_patrol/test/test_source_archive_contract.py \
  src/waver_patrol/test/test_speed_limiter.py \
  src/waver_patrol/test/test_ttc_guard.py \
  src/waver_patrol/test/test_ui_slam_mapping_configs.py \
  src/waver_patrol/test/test_waypoint_store.py

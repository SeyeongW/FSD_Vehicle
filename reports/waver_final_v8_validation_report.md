# Waver Final v8 Validation Report

Date: 2026-06-30
Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
Branch: `jo`

## Summary

This pass added the required v8 readback/risk/plan/baseline documents, filled missing validation documentation, added no-motion Docker/SSH checks, added Gazebo functional validation wrappers, and ran repeated static, mock, build, and Gazebo validations.

No real motor command was executed. No serial-port motion test was run.

## Changed / Added In This v8 Pass

- `reports/waver_redteam_risk_review.md`
- `reports/waver_execution_plan.md`
- `reports/waver_baseline_inventory.md`
- `reports/pre_existing_git_status.txt`
- `reports/pre_existing_diff_stat.txt`
- `reports/pre_existing_changed_files.txt`
- `reports/waver_final_v8_validation_report.md`
- `src/waver_patrol/config/gazebo_functional_scenarios.yaml`
- `src/waver_patrol/docs/open_source_comparison_matrix.md`
- `src/waver_patrol/docs/remote_ui_feature_inventory.md`
- `src/waver_patrol/docs/keyboard_teleop_validation.md`
- `src/waver_patrol/docs/autonomous_patrol_validation.md`
- `src/waver_patrol/docs/slam_mapping_validation.md`
- `src/waver_patrol/docs/field_docker_ssh_runbook.md`
- `src/waver_patrol/scripts/waver_remote_ui_validation.sh`
- `src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh`
- `scripts/waver_field_docker_ssh_check.sh`
- `scripts/waver_full_readiness_loop.sh`

## Tests Run

| Command | Status | Key Output |
| --- | --- | --- |
| `python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch src/ugv_main/ugv_tools/ugv_tools` | PASS | `COMPILEALL_PASS` |
| `bash scripts/run_no_ros_unit_tests.sh` | PASS | `56 passed` |
| `python3 scripts/waver_contract_check.py` | PASS | `WAVER_CONTRACT_CHECK=PASS SCORE=0` |
| `bash scripts/waver_clone_to_run_acceptance.sh` | PASS | `WAVER_CLONE_TO_RUN_ACCEPTANCE=PASS` |
| `cd src/waver_patrol && PYTHONPATH=. python3 -m pytest -q test` | PASS | `76 passed, 10 skipped` |
| YAML parse check | PASS/SKIP | real/nav and Gazebo scenario YAML OK; optional `nav2_params_waver_indoor_safe.yaml` missing was reported as optional skip |
| `docker compose -f docker-compose.jetson.yml config` | PASS | compose config generated |
| `bash scripts/waver_docker_env_check.sh` | PASS | container not running locally, compose config OK |
| `bash scripts/waver_field_docker_ssh_check.sh --local-compose --jetson-compose --no-motion --cycles 2` | PASS_WITH_SKIP | local compose PASS; live Jetson SSH skipped because `WAVER_ENABLE_SSH_VALIDATION=1` was not set |
| `bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 2` | PASS | UI static bridge contract passed twice |
| Gazebo keyboard smoke, 2 cycles | PASS | spawn + keyboard script completion markers |
| Gazebo autonomous patrol smoke, 2 cycles | PASS | spawn + patrol helper markers |
| Gazebo SLAM mapping smoke, 2 cycles | PASS | saved map quality and static obstacle evidence PASS |
| Gazebo safety obstacle smoke, 2 cycles | PASS | static obstacle evidence PASS |
| `bash scripts/waver_full_readiness_loop.sh --cycles 2 --no-real-hardware` | PASS | 14/14 loop steps PASS |
| `python3 scripts/waver_repo_cleanup_audit.py` | PASS | audit completed; generated/cache and backup candidates reported |
| `python3 scripts/make_source_archive.py --dry-run --list` | PASS | generated/private artifact grep check passed |
| `colcon build --symlink-install --packages-select waver_patrol ugv_tools ugv_gazebo waver_experiment_logger waver_seo_tracking` | PASS | 5 packages finished |

## Gazebo Experiment Results

| Scenario | Status | Evidence |
| --- | --- | --- |
| `keyboard_teleop_smoke` | PASS x2 | `reports/gazebo_functional_validation/20260630_185044/` |
| `autonomous_patrol_smoke` | PASS x2 | `reports/gazebo_functional_validation/20260630_185209/` |
| `slam_mapping_smoke` | PASS x2 | `reports/gazebo_functional_validation/20260630_185338/` |
| `safety_obstacle_smoke` | PASS x2 | `reports/gazebo_functional_validation/20260630_185732/` |
| `remote_ui_smoke` | PASS x2 mock/static | `reports/remote_ui_validation/20260630_184352/` |

SLAM key evidence from the Gazebo smoke:

- map size: `600x600`
- occupied cells: `5630`
- free cells: `81739`
- known ratio: `0.2427`
- static obstacle best occupied count: `86`
- result: `PASS`

## Verification Cycles

Full readiness loop:

- Evidence root: `reports/full_readiness_loop/20260630_190110/`
- Cycle 1: compileall, no-ROS tests, contract check, clone-to-run, remote UI mock, Docker/SSH no-motion, Gazebo keyboard all PASS.
- Cycle 2: same steps all PASS.

## Cleanup Result

- Removed source-side Python `__pycache__` and `.pytest_cache` created during validation.
- Kept `build/`, `install/`, and `log/` as generated build outputs; they were not edited as source.
- Kept vendor/upstream-like large files under Livox/Gazebo paths.
- Kept ambiguous backup candidate `src/ugv_main/pcd_cluster_pkg/pcd_cluster_pkg/cluster_node_backup.py`; deletion is deferred pending owner/reference review.

## Failed Or Skipped Items

- Live Jetson SSH/Docker status was skipped in no-motion validation because `WAVER_ENABLE_SSH_VALIDATION=1` was not set.
- Optional `src/waver_patrol/config/nav2_params_waver_indoor_safe.yaml` is absent and was recorded as an optional YAML skip.
- Gazebo logs show `Failed to load plugin libros2_livox.so`; the SLAM smoke still passed through the available scan-mapper path. Real Mid-360/Gazebo Livox plugin readiness remains a manual follow-up if Livox simulation is required.

## Remaining Manual Checks Before Wheel-Off / Wheel-On

- Verify `/cmd_vel` publisher is exactly `safety_cmd_mux_node`.
- Verify `/waver/mode` publisher is exactly `mission_patrol_manager_node`.
- Verify `/scan` rate is at least 5 Hz.
- Verify scan front/rear minimum distances are sane.
- Verify Livox adapter state is not `DEGRADED`, `STALE`, `FAILED`, or `NO_POINTS`.
- Verify keyboard/manual commands pass through safety mux.
- Verify remote UI opens against the real backend in no-motion/safe state.
- Verify Docker container `fsd_dev_jetson` is running on Jetson.
- Verify `docker exec` can source ROS and `install_docker/setup.bash`.
- Verify wheel-off forward/reverse/turn directions.
- Verify odom direction matches physical motion.
- Verify `map -> odom -> base_link` TF exists.
- Verify AMCL initial pose and waypoint safety.
- Verify physical E-stop and `/waver/emergency_stop`.
- Verify serial owner count is one.
- Start rosbag before wheel-on.
- Keep first wheel-on speed at or below `0.05 m/s` linear and `0.20 rad/s` angular.

## Final Readiness Judgement

- Static package readiness: PASS.
- Remote UI mock/no-motion readiness: PASS.
- Gazebo keyboard/patrol smoke readiness: PASS.
- Gazebo SLAM/static obstacle smoke readiness: PASS through scan-mapper path.
- Field Docker/SSH no-motion readiness: PASS_WITH_SKIP for live Jetson status.
- Real wheel-on readiness: NOT PROVEN locally; requires the manual checks above.

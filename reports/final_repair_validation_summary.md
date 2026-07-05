# Waver ROS2_WS5 Final Repair Validation Summary

Generated: 2026-07-05 21:38 KST

## 2026-07-05 23:50 KST Prompt Addendum

- Readiness truthfulness audit was hardened:
  - `scripts/generate_bird_mission_readiness_audit.py`
  - `src/waver_patrol/test/test_readiness_truthfulness_contract.py`
  - `reports/bird_mission_readiness_audit.md`
- Honest top-level judgment remains `BIRD_PATROL_SOURCE_READY`.
- Simulation evidence is separated into:
  - `UI_SLAM_MAPPING_SIM_READY`
  - `SAVED_MAP_NAV2_SIM_READY`
  - `GAZEBO_BIRD_PATROL_MECHANISM_SIM_READY`
  - `UI_SLAM_AND_BIRD_DETECTION_SIM_READY`
- Real hardware readiness remains NOT_RUN/NO without live Jetson/Livox/camera/base/sound probe evidence.
- Operator station CLI was aligned with field runbook:
  - `waver_check_local_operator_deps.py --check`
  - `waver_setup_local_pc.sh --check [--require-ros]`
  - `waver_setup_local_pc.sh --install-rviz`
  - `waver_field_rviz_start.sh --dry-run`
  - `waver_field_local_ui_start.sh --dry-run`
  - `waver_field_operator_station_start.sh --dry-run --rviz --ui`
  - `waver_field_operator_station_start.sh --rviz-only`
  - `waver_field_operator_station_start.sh --ui-only`
- `waver_field_rviz_start.sh` now defaults to `/scan_safety`, `/livox/lidar`, and fixed frame `map`.
- `waver_field_local_ui_start.sh --dry-run` no longer requires password/key, ROS setup, local build, or reachable Jetson, and masks password output.
- Added `docs/RVIZ_FIELD_RUNBOOK.md`.
- Fresh validation:
  - compileall PASS
  - shell syntax PASS
  - no-ROS unit tests: 85 passed
  - focused pytest: 18 passed
  - release self-test: PASS, targeted pytest 41 passed
  - field release archive check: PASS
  - integrated Gazebo UI SLAM + bird smoke: PASS
- Fresh release archive generation/check: PASS.
  The exact tarball hash is intentionally reported outside the source archive
  handoff text to avoid self-referential archive hash churn.
- Fresh UI SLAM + bird smoke:
  - report `reports/ui_slam_bird_detection/latest.json`
  - elapsed `224.712s`
  - map quality PASS
  - `/map` publisher count `1`: `laser_scan_occupancy_mapper_node`
  - `/cmd_vel` publisher count `1`: `safety_cmd_mux_node`
  - `/waver/mode` publisher count `1`: `mission_patrol_manager_node`
  - bird topics fresh: true
  - no patrol/target/sound conflict while mapping: true

## Workspace

- Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- Branch: `jo`
- Head: `170c38b`
- Field bridge critical scripts kept intact:
  - `scripts/waver_field_docker_backend_start.sh`
  - `scripts/waver_field_local_ui_start.sh`
  - `scripts/waver_field_lidar_nav_backend_start.sh`
  - `scripts/waver_start_field_backend.sh`
- Validated bridge architecture:
  - local PC / remote UI
  - SSH
  - Jetson host
  - `docker exec`
  - Docker container `fsd_dev_jetson`
  - ROS 2 nodes
  - Waver USB serial

## Prompt Scope Covered

The 2026-07-05 repair pass focused on the attached final hardware/product prompt:

- source/release boundary and worktree inventory
- Livox Mid-360 runtime overlay and topic alias policy
- ROS_DOMAIN_ID policy set to production domain `0`
- field bridge regression for all critical bridge scripts
- production launch argument contract
- command-chain and collision-monitor dry-run contract
- probe-first readiness levels
- sound deterrent ACK/backend gating
- camera-LiDAR fusion/calibration gates
- release self-test wrapper stability
- Gazebo UI SLAM/Nav2 regression
- Gazebo bird patrol mechanism regression
- honest real-hardware readiness separation

## Important Fixes Made

- `scripts/waver_quality_gate.sh` no longer keeps ROS_DOMAIN_ID `77` as production default.
- `scripts/waver_worktree_inventory.py` now classifies `config/sensors/*`, `config/perception/*`, `scripts/waver_bird_*`, and release check/generation scripts as field-release relevant.
- `reports/worktree_inventory.md` now lists all four field bridge critical scripts, including `waver_field_local_ui_start.sh`.
- Livox field config was split into source-controlled examples and local runtime overlay policy.
- `/livox/lider` is only retained in the alias policy as a forbidden typo mapping to `/livox/lidar`.
- `scripts/waver_field_lidar_nav_backend_start.sh` no longer hard-codes sound output and ACK to false.
- `bird_detector_node.py` no longer requires camera alignment before detector inference.
- `mission_patrol_manager_node.py` keeps strict real sound gates and adds a short same-target dynamic-valid grace only where explicitly configured.
- Gazebo bird patrol profile allows simulator-specific dynamic grace while the real profile keeps strict fusion/dynamic/centered gates.
- SEO tracking camera/body alignment transforms target poses through TF instead of assuming frames match.
- Spatial response logger emits verifier-compatible paper metrics and command-chain authority evidence.
- `scripts/check_bird_mission_field_release.py --self-test` now records wrapper stdout/stderr tail evidence to `reports/release_self_test/check_bird_mission_field_release_self_test.json`.
- `scripts/check_remote_ui_slam_mapping_result.py` now guards PGM payload length and obstacle window bounds; this fixed a real out-of-range failure found during this pass.

## Static And Contract Validation

PASS:

- `python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch src/waver_patrol/scripts src/waver_seo_tracking src/waver_experiment_logger/waver_experiment_logger`
- `find scripts -maxdepth 1 -type f -name '*.sh' -print0 | xargs -0 -r bash -n`
- `bash scripts/run_no_ros_unit_tests.sh`
  - `85 passed`
- Targeted source pytest excluding release-generation recursion tests
  - `160 passed, 10 skipped`
- Release field/inventory focused pytest
  - `6 passed`
- `python3 scripts/waver_contract_check.py`
  - `PASS`, score `0`
- `python3 scripts/waver_launch_contract_check.py --profile config/real_profiles/bird_patrol_production.yaml`
  - `PASS`
- `python3 scripts/waver_field_bridge_regression_check.py --dry-run`
  - `PASS`
- `python3 scripts/waver_command_chain_check.py --dry-run --profile config/real_profiles/bird_patrol_production.yaml`
  - `PASS`
- `python3 scripts/waver_ros_network_check.py`
  - `PASS`, expected field `ROS_DOMAIN_ID=0`
- `python3 scripts/waver_bird_mission_readiness_check.py --mode source --profile config/real_profiles/bird_patrol_production.yaml --strict --no-hardware`
  - `PASS`
- `bash scripts/run_bird_mission_release_self_tests.sh`
  - `PASS`, targeted pytest `24 passed`
- `python3 scripts/generate_bird_mission_readiness_audit.py`
  - top-level judgment `BIRD_PATROL_SOURCE_READY`

## Release Validation

Command:

```bash
python3 scripts/make_bird_mission_field_release.py --output /tmp/waver_bird_mission_field_release.tar.gz
python3 scripts/check_bird_mission_field_release.py --path /tmp/waver_bird_mission_field_release.tar.gz --self-test
```

Result:

- `BIRD_MISSION_FIELD_RELEASE_CHECK=PASS`
- release size: `8.4M`
- bytes: `8718087`
- sha256: `779622d95b937dac777a8b93c4d38f298e6b3a7c64d7de4076b8a2dad4bb80b5`
- wrapper report: `reports/release_self_test/check_bird_mission_field_release_self_test.json`
- release-safe source report: `reports/release_self_test/latest_bird_mission_release_self_test.json`

## UI SLAM Mapping And Saved-Map Nav2

UI SLAM mapping smoke rerun:

```bash
bash scripts/waver_ui_slam_cleanup.sh || true
WAVER_SPAWN_TEST_OBSTACLE=true \
WAVER_USE_GUI=false \
WAVER_START_RVIZ=false \
WAVER_UI_SLAM_TIMEOUT=150 \
bash scripts/run_ui_slam_mapping_gazebo_smoke.sh
```

Observed result:

- `RESULT=PASS`
- Saved map: `maps/waver_latest_map.yaml`
- Map quality:
  - width `600`
  - height `600`
  - occupied `15333`
  - free `82379`
  - unknown `262288`
  - known ratio `0.2714`
  - obstacle occupied best count `309`

Saved-map Nav2 static obstacle check:

```bash
ROS_DOMAIN_ID=55 \
GAZEBO_MASTER_URI=http://127.0.0.1:11348 \
WAVER_USE_GUI=false \
WAVER_NAV_STARTUP_WAIT_SEC=30 \
WAVER_NAV_RESULT_TIMEOUT=100 \
WAVER_MIN_ODOM_DISTANCE=0.20 \
bash scripts/run_saved_map_static_obstacle_nav_check.sh
```

Observed result:

- `SAVED_MAP_UI_STARTUP_CHECK RESULT=PASS`
- `SAVED_MAP_NAV2_CHECK RESULT=PASS`
- `/map` publisher count: `1`
- `current_map_source=STATIC_MAP`
- odom distance: `1.711 m`
- obstacle clearance minimum: `0.687 m`
- required clearance: `0.420 m`
- safety state: `AUTO_PASS SCAN_CLEAR`

## Remote UI SLAM + Bird Detection Requirement

The user added an explicit integrated UI requirement after the latest smoke runs:

- Remote UI SLAM mapping must still show bird-detection/perception status when that stack is enabled.
- The panel must display bird class, confidence, bird-confirmed state, LiDAR target state, camera alignment, and sound state while the map display is in `SLAM_LIVE_MAP`.
- Mapping mode may display perception information, but patrol waypoint navigation, target approach, and sound deterrent must stay blocked unless a dedicated integrated inspection test mode explicitly enables them.

Source audit:

- `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py` already declares/subscribes/displays the SLAM/mapping topics and the bird/perception topics together.
- Added audit report: `reports/remote_ui_slam_bird_feature_audit.md`.
- Added static contract test: `src/waver_patrol/test/test_remote_ui_slam_bird_contract.py`.

Integrated smoke added and run:

```bash
ROS_DOMAIN_ID=62 \
GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
TIMEOUT_SEC=220 \
WAVER_USE_GUI=false \
WAVER_START_RVIZ=false \
bash scripts/run_ui_slam_bird_detection_gazebo_smoke.sh
```

Observed result:

- `UI_SLAM_BIRD_DETECTION_SMOKE=PASS`
- Report: `reports/ui_slam_bird_detection/latest.json`
- `current_map_source_slam_live=True`
- `bird_topics_visible=True`
- `bird_topics_fresh=True`
- `bird_detector_state_fresh=True`
- `bird_fusion_state_fresh=True`
- `mapping_path_visible=True`
- `no_patrol_conflict=True`
- `no_target_approach_without_arm=True`
- `no_sound_without_arm=True`
- `map_quality_pass=True`
- `/map` publisher count: `1`
- `/cmd_vel` publisher count: `1`
- `/waver/mode` publisher count: `1`
- map quality: occupied `5211`, free `86252`, known ratio `0.2541`
- obstacle mapped: best occupied count `102`

Interpretation:

- `UI_SLAM_AND_BIRD_DETECTION_SIM_READY=yes`, for deterministic Gazebo sim-only UI/perception display plumbing.
- This does not prove real detector/fusion/sound readiness.
- Real readiness still requires live Jetson, Livox, camera, calibration, base feedback, E-stop, blackbox, and safety evidence.

## Final Validation Rerun After UI SLAM + Bird Repair

Static and contract checks:

- `python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch src/ugv_main/ugv_gazebo/launch src/ugv_main/ugv_tools/ugv_tools`: PASS
- `find scripts -maxdepth 1 -type f -name '*.sh' -print0 | xargs -0 -r bash -n`: PASS
- `bash scripts/run_no_ros_unit_tests.sh`: `85 passed`
- targeted pytest excluding release-generation/source-archive tests: `165 passed, 10 skipped`
- focused UI SLAM/bird contract tests: `8 passed`
- `python3 scripts/waver_contract_check.py`: PASS, score `0`
- `python3 scripts/check_ui_slam_bird_detection_result.py --report reports/ui_slam_bird_detection/latest.json`: PASS

Field/real-profile dry-run checks:

- `python3 scripts/waver_launch_contract_check.py --profile config/real_profiles/bird_patrol_production.yaml`: PASS
- `python3 scripts/waver_field_bridge_regression_check.py --dry-run`: PASS
- `python3 scripts/waver_command_chain_check.py --dry-run --profile config/real_profiles/bird_patrol_production.yaml`: PASS
- `python3 scripts/waver_ros_network_check.py`: PASS
- `python3 scripts/waver_bird_mission_readiness_check.py --mode source --profile config/real_profiles/bird_patrol_production.yaml --strict --no-hardware`: PASS

Release:

- `python3 scripts/make_bird_mission_field_release.py --output /tmp/waver_bird_mission_field_release.tar.gz`: PASS
- `python3 scripts/check_bird_mission_field_release.py --path /tmp/waver_bird_mission_field_release.tar.gz --self-test`: PASS
- release bytes: `8732420`
- release sha256: `73923186dd989071a6faa55104b898fd9ef7e7b06a0e34207a0316baac3a528b`

## Gazebo Bird Patrol Mechanism

Final successful rerun:

```bash
ROS_DOMAIN_ID=61 \
GAZEBO_MASTER_URI=http://127.0.0.1:11354 \
TIMEOUT_SEC=300 \
RANDOM_SEED=706 \
bash scripts/run_gazebo_lidar_spatial_response_smoke.sh
```

Run directory:

`experiment_results/gazebo_spatial_response/spatial_lidar_20260705_213157_seed706_20260705_213157`

Verifier:

- `VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS`

Key metrics:

- `mechanism.removed_bird_count=2`
- `mechanism.return_resume_sequence_success=True`
- `mechanism.mid_patrol_preempt_success=True`
- `mechanism.has_odom_motion_after_target_goal=True`
- `safety.cmd_vel_publisher_count=1`
- `safety.cmd_vel_safety_mux_sole_publisher=True`
- `spatial.valid_lidar_target_sample_count=37`
- `spatial.valid_lidar_target_to_bird_xy_mean_m=0.5637178690287405`
- `navigation.odom_path_length_m=49.86622684647957`
- `lidar.filter_runtime_wall_mean_ms=52.773423483408415`

Mechanism verdict:

- Patrol is interrupted before waypoint completion by a LiDAR target.
- Target approach and body/camera alignment path executes in Gazebo.
- Sound task request and bird departure/removal path executes in Gazebo.
- Return/resume sequence executes after target removal.
- Final command authority remains single-publisher through the safety chain.

Important limitation:

- Gazebo uses simulator/fake-classification mechanisms and cannot be promoted to real detector/fusion readiness.

## Current Readiness Judgment

Current honest top-level judgment:

`BIRD_PATROL_SOURCE_READY`

Also proven in simulation:

- `UI_SLAM_MAPPING_SIM_READY`
- `SAVED_MAP_NAV2_SIM_READY`
- `GAZEBO_BIRD_PATROL_MECHANISM_SIM_READY`
- `UI_SLAM_AND_BIRD_DETECTION_SIM_READY` for deterministic Gazebo sim-only UI/perception display plumbing

Not proven on real hardware yet:

- `BIRD_PATROL_FIELD_BRIDGE_READY`
- `BIRD_PATROL_SENSOR_LIVE_READY`
- `BIRD_PATROL_LIDAR_TRACKING_READY`
- `BIRD_PATROL_DETECTOR_READY`
- `BIRD_PATROL_FUSION_READY`
- `BIRD_PATROL_INSPECTION_READY`
- `BIRD_PATROL_SUPERVISED_DETERRENCE_READY`
- `BIRD_PATROL_AUTONOMOUS_READY`

## Real Hardware Blockers

The package must not be called real autonomous ready until these have fresh evidence:

- Jetson Docker field bridge remote smoke on the actual target.
- Livox Mid-360 probe with `/livox/lidar`, `/scan_safety`, frame, point count, rate, timestamp, dropout, and TF evidence.
- RGB camera image and camera_info probe.
- Camera-LiDAR extrinsic check with live TF and live topics.
- Real detector model probe with class map and latency evidence.
- Base serial feedback probe, odom feedback, voltage scale, stop burst, and command timeout evidence.
- Wheel-off motor calibration and direction check.
- EKF and `odom -> base_link` freshness.
- Collision monitor active in final `/cmd_vel` chain.
- Hardware sound backend ACK, self-test, and cancel-test.
- Blackbox recording and emergency stop evidence.
- Battery return and target departure monitor evidence.

## Evidence Files

- `reports/worktree_inventory.md`
- `reports/livox_submodule_policy.md`
- `reports/launch_contract/latest.json`
- `reports/field_bridge_regression/latest.json`
- `reports/command_chain/latest.json`
- `reports/network/latest.json`
- `reports/release_self_test/latest_bird_mission_release_self_test.json`
- `reports/release_self_test/check_bird_mission_field_release_self_test.json`
- `reports/sim_regression/latest.json`
- `reports/bird_mission_readiness_audit.md`
- `/home/chotaehyun/0705waver.txt`

## 2026-07-06 Prompt Pass: Field Operator, Release Self-Test, and Profile Staging

This pass addressed the remaining field-product issues from the latest prompt:

- `scripts/run_bird_mission_release_self_tests.sh` now records per-step timeout, return code, log path, and output tail into JSON evidence.
- `scripts/check_bird_mission_field_release.py --self-test` now catches wrapper timeout explicitly and preserves stdout/stderr tail in the wrapper report.
- `src/waver_patrol/launch/remote_visualization.launch.py` now defaults to the field RViz config and `/scan_safety`.
- `src/waver_patrol/rviz/waver_field_operator.rviz` was added for local operator visualization of `/map`, `/scan_safety`, `/livox/lidar`, odom, paths, dynamic object markers, and bird target markers.
- `config/real_profiles/bird_patrol_production.yaml` now declares `schema_version: bird_patrol_profile_v1`, `source_of_truth: nested_sections`, and `compatibility_flat_aliases: true`.
- staged real profiles were added:
  - `sensor_live.yaml`
  - `inspection_dry_run.yaml`
  - `supervised_bird_patrol.yaml`
  - `autonomous_bird_patrol_locked.yaml`
- `docs/bird_patrol_field_profiles.md` documents the evidence-gated progression from wheel-off to autonomous locked profile.

Fresh validation:

- Python compileall: PASS
- shell syntax: PASS
- YAML parse for real profiles and waver configs: PASS
- `bash scripts/run_no_ros_unit_tests.sh`: `86 passed`
- focused pytest for operator station, release, profile, readiness, cleanup, and UI SLAM/bird contracts: PASS
- `bash scripts/run_bird_mission_release_self_tests.sh`: PASS, targeted pytest `42 passed`
- clean field release generation/check: PASS
- clean field release `--self-test`: PASS
- local operator dry-runs:
  - `waver_field_rviz_start.sh --dry-run`: PASS
  - `waver_field_local_ui_start.sh --dry-run`: PASS with password masked
  - `waver_field_operator_station_start.sh --dry-run --rviz --ui`: PASS

Fresh integrated Gazebo UI SLAM + bird smoke:

- command:
  `ROS_DOMAIN_ID=64 GAZEBO_MASTER_URI=http://127.0.0.1:11357 TIMEOUT_SEC=220 WAVER_USE_GUI=false WAVER_START_RVIZ=false bash scripts/run_ui_slam_bird_detection_gazebo_smoke.sh`
- result: `UI_SLAM_BIRD_DETECTION_SMOKE=PASS`
- report: `reports/ui_slam_bird_detection/latest.json`
- `/map` publisher count: 1, node `laser_scan_occupancy_mapper_node`
- `/cmd_vel` publisher count: 1, node `safety_cmd_mux_node`
- `/waver/mode` publisher count: 1, node `mission_patrol_manager_node`
- `current_map_source_slam_live=True`
- `bird_topics_visible=True`
- `bird_topics_fresh=True`
- `bird_detector_state_fresh=True`
- `bird_fusion_state_fresh=True`
- `mapping_path_visible=True`
- `map_quality_pass=True`
- `no_target_approach_without_arm=True`
- `no_sound_without_arm=True`

Observed limitation:

- During the smoke's post-mapping START_PATROL probe, the spawned static obstacle caused the Gazebo patrol helper to enter obstacle recovery/emergency-stop repeatedly before STOP. This did not violate the UI SLAM/bird smoke acceptance criteria, but it is useful evidence that obstacle placement can block patrol progression in this scenario.
- Real hardware readiness remains `BIRD_PATROL_SOURCE_READY`; live Jetson/Livox/camera/base/sound evidence is still required before any autonomous-ready claim.

## 2026-07-06 Final Prompt Pass: Staged Start Script and Gazebo Mechanism Recheck

Additional prompt requirements handled:

- `scripts/check_bird_mission_field_release.py` now exposes `--self-test-timeout-sec`.
- The release self-test wrapper report includes `timed_out`.
- `scripts/waver_bird_patrol_field_start.sh` now auto-selects staged profiles when `--profile` is omitted:
  - `sensor-live` -> `sensor_live.yaml`
  - `lidar-tracking` -> `lidar_nav_backend.yaml`
  - `detector-live`, `fusion-live`, `inspection-dry-run` -> `inspection_dry_run.yaml`
  - `supervised-deterrence` -> `supervised_bird_patrol.yaml`
  - `autonomous-patrol` -> `autonomous_bird_patrol_locked.yaml`
- Field readiness level defaults are mode-aware instead of always defaulting to L5.
- `README_BIRD_PATROL_FIELD.md` and `docs/bird_patrol_field_profiles.md` document the staged profile mapping.

Fresh validation after these changes:

- Static parse: PASS
- no-ROS unit tests: `86 passed`
- focused pytest: `30 passed`
- release self-test: PASS, targeted pytest `42 passed`
- local operator dependency and dry-run commands: PASS
- source readiness / launch contract / command chain dry-run: PASS

Fresh Gazebo bird patrol mechanism smoke:

- command:
  `ROS_DOMAIN_ID=65 GAZEBO_MASTER_URI=http://127.0.0.1:11358 TIMEOUT_SEC=320 RANDOM_SEED=707 WAVER_USE_GUI=false WAVER_START_RVIZ=false bash scripts/run_gazebo_lidar_spatial_response_smoke.sh`
- result: `VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS`
- run dir:
  `experiment_results/gazebo_spatial_response/spatial_lidar_20260706_005814_seed707_20260706_005814`
- notable PASS checks:
  - `detector_mode_lidar`
  - `lidar_only_decision_clean`
  - `has_dynamic_lock`
  - `has_object_mission_goal`
  - `has_active_target_nav_goal`
  - `patrol_preempt_to_target_goal`
  - `preempt_before_first_patrol_success`
  - `return_resume_sequence_success`
  - `removed_bird_count`
  - `cmd_vel_safety_mux_sole_publisher`

Fresh Gazebo remote UI SLAM + bird smoke:

- command:
  `ROS_DOMAIN_ID=66 GAZEBO_MASTER_URI=http://127.0.0.1:11359 TIMEOUT_SEC=220 WAVER_USE_GUI=false WAVER_START_RVIZ=false bash scripts/run_ui_slam_bird_detection_gazebo_smoke.sh`
- result: `UI_SLAM_BIRD_DETECTION_SMOKE=PASS`
- report: `reports/ui_slam_bird_detection/latest.json`
- `/map` publisher count: 1, node `laser_scan_occupancy_mapper_node`
- `/cmd_vel` publisher count: 1, node `safety_cmd_mux_node`
- `/waver/mode` publisher count: 1, node `mission_patrol_manager_node`
- `bird_topics_fresh=True`
- `bird_detector_state_fresh=True`
- `bird_fusion_state_fresh=True`
- `mapping_path_visible=True`
- `map_quality_pass=True`
- `no_target_approach_without_arm=True`
- `no_sound_without_arm=True`

Final real-hardware judgment remains unchanged:

- `BIRD_PATROL_SOURCE_READY`: YES
- `UI_SLAM_AND_BIRD_DETECTION_SIM_READY`: YES, simulation-only
- `GAZEBO_BIRD_PATROL_MECHANISM_SIM_READY`: YES, simulation-only
- `BIRD_PATROL_AUTONOMOUS_READY`: NOT_RUN until live Jetson/Livox/camera/base/sound field evidence exists.

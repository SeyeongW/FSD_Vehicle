# Codex Requirement Scoreboard

Status values: `TODO`, `PASS`, `FAIL`, `SKIP_WITH_REASON`.

| ID | Requirement | Status | Evidence | Files changed |
| --- | --- | --- | --- | --- |
| 0-01 | Create `AGENTS.md` before further source edits | PASS | File created first in this task turn before additional source edits. | `AGENTS.md` |
| 0-02 | Create readback report | PASS | Readback records goal, non-goals, scope, forbidden actions, validation, hardware limits. | `reports/codex_readback.md` |
| 0-03 | Create/update requirement scoreboard | PASS | This table updated after implementation and validation. | `reports/codex_requirement_scoreboard.md` |
| R-01 | Record repo state and pre-existing changes | PASS | Readback records dirty starting state and branch `jo`. | `reports/codex_readback.md` |
| R-02 | Write repository inventory | PASS | Inventory includes package/layout/runtime/sim/test/vendor/cleanup boundaries. | `src/waver_patrol/docs/repo_inventory.md` |
| R-03 | Write package role matrix | PASS | Role matrix includes required sections and future split recommendation. | `src/waver_patrol/docs/package_role_matrix.md` |
| S-01 | Fix OK_CLEAR scan safety ordering | PASS | `_scan_state()` no longer early-returns `SCAN_CLEAR` on `OK_CLEAR`; stale/finite/hard/slow order preserved. | `src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py` |
| S-02 | Add safety regression tests | PASS | Tests cover OK_CLEAR front/rear hard stop, low finite points, stale scan, slow zone, OK_OBSTACLE hard stop, clear scan. | `src/waver_patrol/test/test_waver_bird_autonomy_core.py` |
| S-03 | Add/static-check OK_CLEAR ordering | PASS | Static test checks no OK_CLEAR early return before hard-stop logic. | `src/waver_patrol/test/test_field_safety_contracts.py` |
| L-01 | Add indoor real patrol launch | PASS | Launch added with conservative indoor defaults and exposed required arguments. | `src/waver_patrol/launch/waver_indoor_patrol_real.launch.py` |
| L-02 | Make battery return pass-through controllable | PASS | Real launch exposes and forwards `enable_battery_return`; indoor default false. | `src/waver_patrol/launch/waver_real_bird_autonomy.launch.py`, `src/waver_patrol/launch/waver_indoor_patrol_real.launch.py` |
| L-03 | Add launch static contracts | PASS | Static tests assert indoor defaults and disabled non-indoor stacks. | `src/waver_patrol/test/test_indoor_real_profile_contracts.py` |
| P-01 | Adjust real preflight normal/fatal safety states | PASS | `STANDBY_STOP`/timeouts are not fatal; scan/emergency/fault strict states fail; camera/bird/battery/pointcloud are conditional. | `src/waver_patrol/scripts/waver_real_preflight_check.sh` |
| P-02 | Enforce safety mux as final `/cmd_vel` by default | PASS | `waver_cmd_chain_check.sh` requires `safety_cmd_mux_node`; collision monitor only via explicit env override. | `src/waver_patrol/scripts/waver_cmd_chain_check.sh` |
| P-03 | Improve scan quality check and strict mode | PASS | Script reports Hz, finite count, front/rear min, adapter state, strict adapter missing/degraded gates, requested env names. | `src/waver_patrol/scripts/waver_scan_quality_check.sh` |
| P-04 | Add indoor patrol status helper | PASS | Helper added and verified static contract; it reads topics only and does not publish commands. | `src/waver_patrol/scripts/waver_indoor_patrol_status.sh` |
| C-01 | Set Nav2 `min_y_velocity_threshold` to `0.001` | PASS | YAML parse passed; static test confirms diff-drive-safe threshold. | `src/waver_patrol/config/nav2_params_waver_real.yaml` |
| D-01 | Write indoor real topic contract | PASS | Topic contract added with command chain, owners, safety topics, rosbag list, forbidden defaults. | `src/waver_patrol/docs/indoor_real_topic_contract.md` |
| D-02 | Add TF/odom sanity items to docs | PASS | Runbook includes map->odom->base_link, odom direction, AMCL, waypoint safety, wheel-off direction checks. | `src/waver_patrol/docs/indoor_real_patrol_runbook.md` |
| D-03 | Update README real entry points | PASS | README and real vehicle README link the indoor profile/runbook. | `README.md`, `README_REAL_VEHICLE.md` |
| A-01 | Add cleanup audit script | PASS | Audit script supports text and JSON and required hygiene categories. | `scripts/waver_repo_cleanup_audit.py` |
| A-02 | Write stale candidates and cleanup report | PASS | Cleanup report and stale candidate list added. | `src/waver_patrol/docs/cleanup/cleanup_report.md`, `src/waver_patrol/docs/cleanup/stale_candidates.md` |
| CL-01 | Clean generated/cache artifacts safely | PASS | Source-side `__pycache__` and `.pytest_cache` removed after validation; build/install/log kept as generated artifacts. | cleanup action |
| CL-02 | Investigate backup/old/tmp/deprecated candidates | PASS | `cluster_node_backup.py` and legacy top-level scripts recorded as deferred candidates, not deleted. | `src/waver_patrol/docs/cleanup/stale_candidates.md` |
| CL-03 | Document top-level script cleanup policy | PASS | Top-level script candidates documented and kept pending reference consolidation. | `src/waver_patrol/docs/cleanup/stale_candidates.md` |
| CL-04 | Audit package metadata TODOs without license guessing | PASS | Existing metadata audit preserved; audit script reports TODO package XMLs. | `src/waver_patrol/docs/PACKAGE_METADATA_AUDIT.md`, `scripts/waver_repo_cleanup_audit.py` |
| AR-01 | Verify source archive excludes generated/private artifacts | PASS | `make_source_archive.py --dry-run --list` grep check passed. | `scripts/make_source_archive.py`, `src/waver_patrol/test/test_source_archive_contract.py` |
| RB-01 | Add indoor real patrol runbook | PASS | Runbook includes purpose, prohibitions, tests, sensors, launch, wheel-off/on, status, rosbag, patrol, emergency, final checklist. | `src/waver_patrol/docs/indoor_real_patrol_runbook.md` |
| V-01 | Python compile check | PASS | `python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch` passed. | validation |
| V-02 | no-ROS unit tests | PASS | `bash scripts/run_no_ros_unit_tests.sh` -> 56 passed. | validation |
| V-03 | contract check | PASS | `python3 scripts/waver_contract_check.py` -> PASS, score 0, only build/install/log INFO. | validation |
| V-04 | clone-to-run acceptance | PASS | `bash scripts/waver_clone_to_run_acceptance.sh` -> PASS. | validation |
| V-05 | waver_patrol tests | PASS | Exact `pytest` executable was unavailable; equivalent `python3 -m pytest -q test` -> 76 passed, 10 skipped. | validation |
| V-06 | YAML parse | PASS | `nav2_params_waver_real.yaml` OK; optional indoor-safe YAML absent and reported SKIP by command. | validation |
| V-07 | cleanup audit | PASS | `python3 scripts/waver_repo_cleanup_audit.py` ran and reported candidates. | validation |
| V-08 | source archive dry-run | PASS | Generated/private directory grep check returned OK. | validation |
| V-09 | static safety/profile checks | PASS | Focused pytest suite -> 38 passed; launch smoke also passed. | validation |
| V8-01 | Create red-team risk review | PASS | Risk review created before v8 source/script/YAML edits. | `reports/waver_redteam_risk_review.md` |
| V8-02 | Create execution plan | PASS | Execution plan created before v8 source/script/YAML edits. | `reports/waver_execution_plan.md` |
| V8-03 | Create baseline inventory | PASS | Baseline inventory and pre-existing git outputs recorded. | `reports/waver_baseline_inventory.md`, `reports/pre_existing_*.txt` |
| V8-04 | Open-source comparison matrix | PASS | Comparison matrix added with Nav2/TurtleBot/Clearpath/Autoware/Livox-style gaps and policy. | `src/waver_patrol/docs/open_source_comparison_matrix.md` |
| V8-05 | Remote UI feature inventory | PASS | UI command/display/bridge contracts documented; mock validation passed twice. | `src/waver_patrol/docs/remote_ui_feature_inventory.md`, `reports/remote_ui_validation/20260630_184352/` |
| V8-06 | Keyboard teleop validation doc/script | PASS | Doc added; Gazebo keyboard smoke passed 2 cycles. | `src/waver_patrol/docs/keyboard_teleop_validation.md`, `reports/gazebo_functional_validation/20260630_185044/` |
| V8-07 | Autonomous patrol validation doc/script | PASS | Doc added; Gazebo autonomous patrol smoke passed 2 cycles. | `src/waver_patrol/docs/autonomous_patrol_validation.md`, `reports/gazebo_functional_validation/20260630_185209/` |
| V8-08 | SLAM mapping validation doc/script | PASS | Doc added; Gazebo SLAM mapping with static obstacle passed 2 cycles via scan-mapper path. | `src/waver_patrol/docs/slam_mapping_validation.md`, `reports/gazebo_functional_validation/20260630_185338/` |
| V8-09 | Docker/SSH no-motion field check | PASS | No-motion local compose check passed twice; live Jetson check explicitly skipped unless env enabled. | `scripts/waver_field_docker_ssh_check.sh`, `reports/field_docker_ssh_check/20260630_184352/` |
| V8-10 | Gazebo functional validation loop | PASS | Wrapper added, fixed false PASS on Gazebo fatal logs, keyboard/patrol/SLAM/obstacle scenarios passed. | `src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh`, `src/waver_patrol/config/gazebo_functional_scenarios.yaml` |
| V8-11 | Full readiness loop with two cycles | PASS | `waver_full_readiness_loop.sh --cycles 2 --no-real-hardware` passed all 14 loop steps. | `scripts/waver_full_readiness_loop.sh`, `reports/full_readiness_loop/20260630_190110/` |
| V8-12 | Selected colcon build | PASS | `waver_patrol ugv_tools ugv_gazebo waver_experiment_logger waver_seo_tracking` built successfully. | validation |
| V8-13 | Final v8 validation report | PASS | Final report records tests, Gazebo evidence, skips, cleanup, and remaining manual checks. | `reports/waver_final_v8_validation_report.md` |

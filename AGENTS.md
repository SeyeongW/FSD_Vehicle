# Waver Agent Guide

This repository is a ROS 2 Humble workspace for Waver UGV field bringup,
Gazebo validation, operator UI, and real-vehicle low-speed patrol.

## Repo Layout

- `src/waver_patrol`: Waver-specific safety, mission, mapping, perception,
  launch, scripts, tests, docs.
- `src/ugv_main`: platform packages, Gazebo, Nav2 wrappers, UI tools, drivers.
- `src/ugv_else`: third-party or upstream ROS packages.
- `src/livox_ros_driver2`, `src/Livox-SDK2`: Livox vendor/upstream-like code.
- `scripts`: field startup, clone-to-run, archive, quality gates.
- `docs`, `reports`: handover, setup, quality and test notes.
- `build`, `install`, `log`: generated artifacts, not source.

## Required Checks

Run from repo root:

```bash
source /opt/ros/humble/setup.bash
python3 -m compileall src/waver_patrol/waver_patrol src/waver_patrol/launch
bash scripts/run_no_ros_unit_tests.sh
python3 scripts/waver_contract_check.py
bash scripts/waver_clone_to_run_acceptance.sh
cd src/waver_patrol && PYTHONPATH=. pytest -q test
```

For source archive hygiene:

```bash
python3 scripts/make_source_archive.py --dry-run --list
python3 scripts/waver_repo_cleanup_audit.py
```

For final readiness passes, also run or explicitly skip with reasons:

```bash
bash scripts/waver_field_docker_ssh_check.sh --local-compose --jetson-compose --no-motion --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario keyboard_teleop_smoke --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario autonomous_patrol_smoke --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario slam_mapping_smoke --cycles 2
bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 2
bash scripts/waver_full_readiness_loop.sh --cycles 2 --no-real-hardware
```

## Real-Vehicle Safety Rules

- Branch must be `jo` for field work.
- Final `/cmd_vel` publisher is `safety_cmd_mux_node` only.
- `/waver/mode` publisher is `mission_patrol_manager_node` only.
- Real/indoor default mode is `STANDBY`.
- Real/indoor `require_scan` default is `true`.
- First wheel-on limits stay at or below `0.05 m/s` and `0.20 rad/s`.
- Real profile must not default-enable fake, test, or Gazebo publishers.
- Do not auto-run motor commands during code validation.
- Do not open serial ports in automated tests.
- Wheel-on requires strict preflight, wheel-off direction checks, and E-stop.

## Do Not Do

- Do not delete `.git`.
- Do not reset, checkout, or overwrite uncommitted user changes.
- Do not guess or rewrite upstream/vendor licenses.
- Do not convert this stack to `ros2_control` in-place.
- Do not make Nav2 Collision Monitor the final `/cmd_vel` owner by default.
- Do not raise real speed limits to make tests pass.
- Do not remove backup/old files unless reference checks are documented.
- Do not declare Gazebo, SLAM, Nav2, Docker, SSH, or real hardware checks
  successful unless the command actually ran and produced evidence.

## Done Criteria

- Safety regressions are covered by tests.
- Indoor real launch has conservative defaults.
- Preflight separates normal standby from fatal safety states.
- Cleanup actions are documented.
- Generated/private artifacts are excluded from source archives.
- Failed or skipped checks are reported honestly.

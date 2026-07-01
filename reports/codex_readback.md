# Codex Readback

Date: 2026-06-30

## v8 Addendum

The current request is `waver_codex_final_v8`: perform a final pre-real-vehicle
readiness pass for `~/ros2_ws5/FSD_Vehicle` on branch `jo`, with explicit
readback, red-team review, baseline inventory, Gazebo/sim/mock validation
attempts, and honest PASS/FAIL/SKIP reporting.

## Goal

Improve `~/ros2_ws5/FSD_Vehicle` for indoor low-speed Waver real-vehicle patrol readiness without large architecture changes. The priority is P0 safety and field readiness: final command authority, scan safety ordering, conservative indoor launch defaults, preflight clarity, scan diagnostics, runbook, and tests.

## Not The Goal

- Do not redesign the whole stack.
- Do not convert to `ros2_control`.
- Do not make Collision Monitor the default final `/cmd_vel` owner.
- Do not raise speed limits.
- Do not change SSH/Docker field workflow security broadly.
- Do not delete vendor/upstream files or rewrite licenses by guesswork.
- Do not execute real motor commands or open serial ports during validation.

## Scope

P0:

- Fix `OK_CLEAR` scan safety bypass in `safety_cmd_mux_node`.
- Add safety regression tests.
- Add conservative indoor real launch.
- Improve preflight, command-chain, scan quality, and status helpers.
- Fix Nav2 `min_y_velocity_threshold`.
- Add indoor runbook and run validation commands.

P1:

- Add repo inventory, package role matrix, topic contract, cleanup audit, source archive checks, README links, static launch contracts.

P2:

- Clean only generated/cache files and document ambiguous stale candidates. Do not delete uncertain files.

v8 additional scope:

- Document open-source comparison, remote UI feature path, keyboard teleop,
  autonomous patrol, SLAM mapping, and Docker/SSH field workflow.
- Add or verify Gazebo functional validation scripts for keyboard, patrol,
  SLAM, remote UI, and safety obstacle scenarios.
- Run at least two validation cycles where the local environment permits it.
- If ROS/Gazebo/Docker/SSH is unavailable, record `SKIP_WITH_REASON` instead
  of pretending success.

## Forbidden Actions

- No `git reset`, `git checkout`, `git restore`, forced cleanup, or deleting `.git`.
- No hardware motor command publication.
- No serial-port opening tests.
- No broad topic/mission/package restructuring.
- No fake/test/Gazebo publishers in real profile defaults.

## Pre-Existing Changes Observed

At task start, git status was already dirty. Existing modified/untracked paths included field scripts, package metadata, Livox submodule changes, `waver_real_bird_autonomy.launch.py`, safety/preflight scripts, tests, docs, and untracked indoor/profile/audit files from previous work. These are treated as pre-existing and are not reverted.

Current branch observed: `jo`.

## Planned Validation Commands

```bash
python3 -m compileall src/waver_patrol/waver_patrol src/waver_patrol/launch
bash scripts/run_no_ros_unit_tests.sh
python3 scripts/waver_contract_check.py
bash scripts/waver_clone_to_run_acceptance.sh
cd src/waver_patrol && PYTHONPATH=. pytest -q test
python3 scripts/waver_repo_cleanup_audit.py
python3 scripts/make_source_archive.py --dry-run --list
```

Additional v8 validation commands:

```bash
bash scripts/waver_field_docker_ssh_check.sh --local-compose --jetson-compose --no-motion --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario keyboard_teleop_smoke --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario autonomous_patrol_smoke --cycles 2
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario slam_mapping_smoke --cycles 2
bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 2
bash scripts/waver_full_readiness_loop.sh --cycles 2 --no-real-hardware
```

## Hardware-Free Limitations

The following cannot be proven without the actual Waver, Livox, map/localization, serial device, and wheel-off/wheel-on setup:

- Real `/scan` quality and adapter state.
- Real odom direction and encoder/IMU consistency.
- Serial owner and base driver behavior on the robot.
- Wheel direction, rotation direction, braking, E-stop, and physical stop.
- AMCL pose quality on the real indoor map.
- Actual waypoint safety in the mapped environment.

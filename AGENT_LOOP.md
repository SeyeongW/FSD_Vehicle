# Waver Hardware-Free Convergence Loop

This workspace is intended to converge toward a real-vehicle-ready Waver UGV
stack without automatically moving hardware. The loop is deliberately
conservative: every iteration must preserve the real safety contracts before any
Gazebo, rosbag, or field checklist is considered useful.

## Scope

Allowed in this loop:

- Static analysis of launch, config, package metadata, and source files.
- Python compile checks, colcon build, colcon test, launch import checks.
- Gazebo simulation, mock tests, and offline rosbag replay.
- Report generation under `reports/`.
- Manual wheel-off / low-speed wheel-on checklists for a human operator.

Forbidden in this loop:

- Opening a real serial port.
- Publishing real motor commands.
- Starting a speaker or GPIO sound output.
- Changing the validated SSH/Jetson/Waver field-control workflow unless the
  operator explicitly requests that exact work.
- Running hardware scripts automatically.

## Iteration Stages

1. DISCOVER: scan package, launch, config, node, and safety-critical paths.
2. BASELINE: run the no-hardware quality gate and record the current score.
3. SELECT: choose a small number of P0/P1 issues, preferring safety contracts.
4. PATCH: make the smallest compatible change that improves the score.
5. BUILD: compile Python and selected ROS packages.
6. TEST: run contract checks, unit tests, launch import checks, and optional
   Gazebo/rosbag replay.
7. REPORT: write changed files, failures, score, next issue, and risks.
8. DECIDE: continue only when the score improves or a safety regression is
   being fixed.

Convergence candidate criteria:

- `safety_regressions = 0`
- `contract_violations = 0`
- `build_failures = 0`
- `critical_issues = 0`
- `high_issues = 0`
- `test_failures = 0`
- Three consecutive hardware-free quality gate passes.

## Scoring

`scripts/waver_contract_check.py` reports this score:

```text
critical_issues * 1000
+ high_issues * 300
+ medium_issues * 50
+ low_issues * 10
+ build_failures * 1000
+ test_failures * 200
+ contract_violations * 1000
+ launch_failures * 500
+ safety_regressions * 5000
```

The quality gate stores reports under `reports/quality_gate/<timestamp>/`.
The iteration loop stores summaries under `reports/agent_iterations/<timestamp>/`.

## Real-Vehicle Contract

The real profile must preserve these invariants:

- The final `/cmd_vel` publisher is `safety_cmd_mux_node`.
- UI, Nav2, and target tracking do not publish directly to `/cmd_vel`.
- The serial writer is a single owner, preferably `waver_base_driver_node`.
- `start_serial_bridge`, split feedback, and `enable_waver_base_driver` cannot
  create multiple serial owners.
- EKF mode gives `odom -> base_link` TF authority to `robot_localization`, not
  to the base driver.
- STANDBY, E-stop, external stop, scan stale, and battery fault produce zero
  command.
- Bird sound is blocked until bird class/confirmation/safety gates pass.
- Target mission interruption stores `departure_pose`; after target departure,
  the robot returns to that pose before resuming waypoint patrol.

## Commands

Run one no-hardware gate:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_quality_gate.sh --no-hardware --report-dir reports/quality_gate/latest
```

Run a bounded iteration loop:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_agent_iteration_loop.sh --max-iterations 3 --time-budget-hours 2 --no-hardware
```

Run only the contract checker:

```bash
cd ~/ros2_ws5/FSD_Vehicle
python3 scripts/waver_contract_check.py --root . --report-dir reports/quality_gate/manual_contract
```


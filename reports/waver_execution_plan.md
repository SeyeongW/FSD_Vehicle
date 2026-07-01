# Waver Execution Plan

Date: 2026-06-30
Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
Branch: `jo`
Prompt: `waver_codex_final_v8`

## Guardrails

- Work only on the `jo` branch.
- Preserve all pre-existing uncommitted changes.
- Do not run real motor commands.
- Do not open serial ports as part of automated validation.
- Keep indoor real defaults conservative: `STANDBY`, `require_scan=true`, `0.05 m/s`, `0.20 rad/s`.
- Keep final `/cmd_vel` owner as `safety_cmd_mux_node`.
- Use Gazebo/sim/mock validation for keyboard, patrol, SLAM, remote UI, and safety obstacle checks.

## Work Phases

1. Baseline capture
   - Record branch, git status, diff stat, changed file list.
   - Confirm required first-step documents exist.

2. Requirements gap audit
   - Compare prompt requirements with current docs, launch files, scripts, tests, and package boundaries.
   - Create missing inventory, comparison, validation, and runbook docs.

3. Implementation
   - Add missing no-motion/Docker/SSH validation helper.
   - Add Gazebo functional scenario config and validation loop scripts.
   - Add remote UI validation helper.
   - Keep changes script/doc focused unless source defects are found.

4. Validation cycle 1
   - Compile Python.
   - Run no-ROS tests.
   - Run contract checks and clone-to-run acceptance.
   - Parse YAML and run static docs/script checks.
   - Attempt Gazebo/sim/mock validations with timeouts.

5. Fix and validation cycle 2
   - Fix failures found in cycle 1.
   - Re-run the relevant checks.
   - Record PASS/FAIL/SKIP_WITH_REASON.

6. Final report
   - Update scoreboard.
   - Summarize tests, skipped items, remaining manual checks, and real-vehicle readiness.

## Validation Outputs

- `reports/pre_existing_git_status.txt`
- `reports/pre_existing_diff_stat.txt`
- `reports/pre_existing_changed_files.txt`
- `reports/full_readiness_loop/<timestamp>/`
- `reports/gazebo_functional_validation/<timestamp>/`

## Stop Conditions

- Any command that could move real hardware is not executed automatically.
- Any detected final `/cmd_vel` owner conflict is treated as a P0 failure.
- Any real profile fake/test/Gazebo publisher default is treated as a P0 failure.
- Missing ROS/Gazebo/Docker/SSH environments are recorded as `SKIP_WITH_REASON`, not hidden.

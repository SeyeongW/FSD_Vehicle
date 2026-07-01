# Waver Baseline Inventory

Date: 2026-06-30
Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
Branch: `jo`

## Baseline State

- Repository root: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- Branch: `jo`
- Worktree at start of this v8 pass: dirty, with prior modifications and untracked files.
- Policy: treat existing dirty changes as user/pre-existing work and do not revert them.

## Required First-Step Documents

- `AGENTS.md`: exists.
- `reports/codex_readback.md`: exists.
- `reports/codex_requirement_scoreboard.md`: exists.
- `reports/waver_redteam_risk_review.md`: created for v8.
- `reports/waver_execution_plan.md`: created for v8.
- `reports/waver_baseline_inventory.md`: this file.

## Key Packages

- `src/waver_patrol`: Waver mission, safety, mapping, perception, launch, checks, tests, docs.
- `src/ugv_main/ugv_tools`: remote panel, field keyboard/control helpers, waypoint utilities.
- `src/ugv_main/ugv_gazebo`: Gazebo worlds/models/visualization.
- `src/ugv_main/ugv_nav`: Nav2/cartographer launch wrappers and navigation configuration.
- `src/waver_experiment_logger`: experiment and paper-data logging.
- `src/waver_seo_tracking`: Gazebo/tracking reference logic, not real command authority.
- `src/livox_ros_driver2`, `src/Livox-SDK2`: Livox vendor/upstream-like code.

## Key Runtime Contracts

- Final real `/cmd_vel` publisher: `safety_cmd_mux_node`.
- Mode authority publisher: `mission_patrol_manager_node`.
- Field Docker container default: `fsd_dev_jetson`.
- Field scripts must support clone-to-run from `~/ros2_ws5/FSD_Vehicle`.
- Indoor real profile defaults must not start bird/camera/sound/test/Gazebo stacks.

## Baseline Files Recorded

Raw baseline command outputs are saved in:

- `reports/pre_existing_git_status.txt`
- `reports/pre_existing_diff_stat.txt`
- `reports/pre_existing_changed_files.txt`

## Immediate Gaps Found Before Source Edits

- Missing v8-specific validation docs for keyboard, autonomous patrol, SLAM, remote UI.
- Missing Gazebo scenario config and Gazebo functional validation loop scripts.
- Missing no-motion Docker/SSH field check helper.
- Missing full readiness loop wrapper.

These gaps are documentation/script-validation gaps. They do not imply that the previously implemented safety fixes should be reverted.

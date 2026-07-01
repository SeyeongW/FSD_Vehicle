# Stale Cleanup Candidates

Date: 2026-06-29

This file records possible cleanup targets. Items here are not removed until reference checks are complete.

## Candidate: `src/ugv_main/pcd_cluster_pkg/pcd_cluster_pkg/cluster_node_backup.py`

- Category: backup-like source file.
- Current action: kept.
- Reason kept: package is outside the Waver-owned safety path and is registered as a `console_scripts` entry point in `src/ugv_main/pcd_cluster_pkg/setup.py`.
- Required reference checks before removal:
  - `rg cluster_node_backup`
  - `rg Cluster` or relevant class/function names in package
  - inspect `setup.py` entry points
  - inspect launch files
  - inspect tests/docs
- Risk if deleted blindly: `colcon build`/entry-point installation or legacy cluster launch/tutorial may break.

## Candidate: top-level map save scripts

- Files:
  - `save_2d_cartographer_map.sh`
  - `save_2d_cartographer_map_gazebo.sh`
  - `save_2d_gmapping_map.sh`
  - `save_2d_gmapping_map_gazebo.sh`
- Current action: kept.
- Reason kept: mapping workflow still references map saving in README/runbooks.
- Future action: replace with one canonical wrapper after references are consolidated.

## Removed legacy PC/helper scripts

- Removed:
  - `run_pc.sh`
  - `build_pc.sh`
  - `remotessh.sh`
  - `ros2_humble.sh`
- Reason: no source, launch, setup, or current field script references remained. These scripts used old Docker service/container names or only started SSH inside a shell, which conflicts with the current documented Linux local PC -> Jetson Docker field workflow.
- Replacement:
  - `bash scripts/waver_field_docker_backend_start.sh`
  - `bash scripts/waver_field_local_ui_start.sh`
  - `bash scripts/waver_doctor.sh`
- Risk: none for ROS package build, Gazebo validation, or real-vehicle command chain.

### Removed legacy Windows helpers

- Removed:
  - `run_pc.bat`
  - `build_pc.bat`
- Reason: no source, launch, setup, or field script references remained after
  `docs/technical_report.md` was updated to the Linux local PC + Jetson Docker
  field workflow.
- Risk: no ROS package, console script, launch file, or real-vehicle command
  chain depends on these files.

## Generated Artifacts

Generated artifacts are not source and can be removed locally when not needed:

- `build/`
- `install/`
- `log/`
- `.pytest_cache/`
- `**/__pycache__/`
- `*.pyc`
- `bags/`, `rosbag*/`, `*.db3`, `*.mcap`
- `experiment_results/`

They are excluded from source archives and should not be edited as source.

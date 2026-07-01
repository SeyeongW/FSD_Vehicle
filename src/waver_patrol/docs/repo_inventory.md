# Waver Repo Inventory

Workspace root for this field-ready tree:

```bash
~/ros2_ws5/FSD_Vehicle
```

## Primary Packages

| Package | Role | Real-Vehicle Status |
| --- | --- | --- |
| `waver_patrol` | Safety mux, mission manager, mapping helpers, LiDAR/camera bird pipeline, real launch wrappers | Primary Waver autonomy package |
| `ugv_tools` | Operator panel, keyboard helpers, field micro-patrol helper | UI and local/field control package |
| `ugv_base_node` | Legacy C++ base odometry/control nodes | Use only when explicitly selected |
| `ugv_bringup` | Legacy bringup/feedback helpers | Do not run on the same serial port as `waver_base_driver_node` |
| `ugv_nav` | Nav2/Cartographer launch wrappers | Used by real and Gazebo navigation profiles |
| `ugv_gazebo` | Gazebo Classic worlds, models, RViz configs | Simulation-only |
| `livox_ros_driver2` | Livox Mid-360 ROS driver | Vendor/upstream-like; patch only when needed for Humble/field config |

## Runtime Generated Paths

These are local artifacts, not source:

```text
build/
install/
log/
.pytest_cache/
experiment_results/
bags/
rosbag*/
reports/quality_gate/*
reports/agent_iterations/*
```

The source archive script excludes these paths.

## Real Launch Entry Points

| Launch | Use |
| --- | --- |
| `waver_real_bird_autonomy.launch.py` | Full real bird-autonomy stack. Bird/camera/target/sound nodes are configurable and are not fake/test nodes. |
| `waver_indoor_patrol_real.launch.py` | Conservative indoor low-speed patrol profile. Bird/sound/target stacks default off, scan and safety mux remain on. |
| `waver_gazebo_mapping_debug.launch.py` | Gazebo mapping UI/SLAM debug only. |
| `waver_gazebo_bird_autonomy.launch.py` | Gazebo bird autonomy validation. Gazebo-only stubs are allowed here only. |

## Final Topic Authority

| Topic | Required Owner |
| --- | --- |
| `/cmd_vel` | `safety_cmd_mux_node` only |
| `/waver/mode` | `mission_patrol_manager_node` only |
| `/scan` or `/scan_safety` | One sensor/adapter publisher in the active profile |
| `/odom` | One EKF/base odometry source |
| `/map` during mapping | One live SLAM/map source |

## Audit Command

```bash
cd ~/ros2_ws5/FSD_Vehicle
python3 scripts/waver_repo_cleanup_audit.py
```

## Top-Level Directory Summary

| Path | Type | Notes |
| --- | --- | --- |
| `src/` | ROS packages | Source packages only. |
| `scripts/` | field/setup/test helpers | Keep canonical field scripts here. |
| `config/` | clone-to-field env defaults | Local secrets belong in ignored local files. |
| `docker/` | container build/runtime files | Used by Jetson Docker field workflow. |
| `docs/` | clone/run/handover docs | Top-level operator docs. |
| `reports/` | generated or template reports | Generated report outputs are ignored. |
| `maps/` | field maps | Source maps are kept; generated map archives are cleanup candidates. |
| `build/`, `install/`, `log/` | generated | Do not edit as source. |

## `src/waver_patrol` Layout

| Path | Role |
| --- | --- |
| `launch/` | real, indoor, Gazebo, mapping and mission launch entry points |
| `config/` | Nav2, mission, safety, mapping, localization YAML |
| `scripts/` | preflight, graph cleanup, scan quality, map quality, status helpers |
| `test/` | unit and static contract tests |
| `waver_patrol/safety/` | final command mux and safety helpers |
| `waver_patrol/mission/` | patrol manager, target goal manager, mission events |
| `waver_patrol/perception/` | Livox scan adapter, pointcloud objects, bird detector/fusion |
| `waver_patrol/mapping/` | mapping workflow, map status, path publisher |
| `waver_patrol/control/` | target tracking/aiming and behavior mux |
| `waver_patrol/bridges/` | serial/base driver, Gazebo bridges, sound stubs |
| `waver_patrol/test_nodes/` | Gazebo/test-only publishers |

## Runtime Core Files

- `launch/waver_indoor_patrol_real.launch.py`
- `launch/waver_real_bird_autonomy.launch.py`
- `waver_patrol/safety/safety_cmd_mux_node.py`
- `waver_patrol/mission/mission_patrol_manager_node.py`
- `waver_patrol/bridges/waver_base_driver_node.py`
- `scripts/waver_real_preflight_check.sh`
- `scripts/waver_cmd_chain_check.sh`
- `scripts/waver_scan_quality_check.sh`
- `scripts/waver_indoor_patrol_status.sh`

## Simulation/Gazebo Files

- `launch/waver_gazebo_mapping_debug.launch.py`
- `launch/waver_gazebo_bird_autonomy.launch.py`
- `launch/gazebo_moving_object_trial.launch.py`
- `src/ugv_main/ugv_gazebo`
- `waver_patrol/test_nodes/*`

## Test/Fake Publisher Files

The installed console scripts ending in `_test_publisher_node`, `simple_sim_odom_node`, `simple_nav2_cmd_sim_node`, and `gazebo_live_mapping_node` are test/Gazebo-only. They must not be enabled by default in real launches.

## Bird/Target/Deterrence Extension Files

- `perception/bird_detector_node.py`
- `perception/bird_3d_fusion_node.py`
- `mission/target_goal_manager_node.py`
- `mission/target_departure_monitor_node.py`
- `control/target_body_tracker_node.py`
- `bridges/sound_deterrent_node.py`

The indoor real launch disables these by default.

## Mapping/SLAM Files

- `launch/waver_gazebo_mapping_debug.launch.py`
- `launch/waver_mapping_backend.launch.py`
- `mapping/mapping_workflow_manager_node.py`
- `mapping/mapping_path_publisher_node.py`
- `scripts/waver_mapping_health_check.sh`
- `scripts/waver_map_quality_check.py`

## Evaluation/Reporting Files

- `waver_experiment_logger`
- `scripts/verify_gazebo_spatial_response_trial.py`
- `scripts/summarize_trial_evidence.py`
- `reports/`

## Vendor/Upstream-Like Packages

- `src/livox_ros_driver2`
- `src/Livox-SDK2`
- `src/livox_laser_simulation_RO2`
- `src/ugv_else`
- portions of `src/ugv_main`

Do not change licenses or delete files here without owner/upstream review.

## Cleanup And Deletion Policy

Generated artifacts can be removed locally. Ambiguous backup/old/tmp/deprecated files are recorded in `docs/cleanup/stale_candidates.md` and kept until references are checked.

# Waver Package Role Matrix

## 1. Real Robot Runtime Core

| Package | Real Runtime | Gazebo Runtime | Notes |
| --- | --- | --- | --- |
| `waver_patrol` | Yes | Yes | Owns safety-critical Waver-specific behavior. |
| `ugv_tools` | Yes | Yes | Operator panel must publish candidate topics, not final `/cmd_vel`. |
| `ugv_nav` | Yes | Yes | Nav2 and localization wrapper. Keep controller output remapped into safety mux. |
| `ugv_base_node` | Optional | No | Legacy base node. Do not run alongside `waver_base_driver_node` on the same serial port. |
| `ugv_bringup` | Optional | No | Feedback helpers only when explicitly selected. |
| `ugv_gazebo` | No | Yes | Simulation-only worlds/models/plugins. |
| `waver_experiment_logger` | Optional | Yes | Data collection; never owns motion command topics. |
| `waver_seo_tracking` | No | Yes/analysis | Reference tracking experiments. Do not use as real final command authority. |

## 2. Indoor Patrol Minimal Runtime

Required:

- `waver_patrol`
- `ugv_tools` operator UI if using local panel
- `ugv_nav` when Nav2/localization is enabled
- one base driver path only
- one scan source
- one odom source

Disabled by default:

- bird detector/fusion
- camera gimbal
- sound deterrent
- target departure monitor
- moving object map transform/filter
- fake/test/Gazebo publishers

## 3. Simulation/Gazebo Only

- `ugv_gazebo`
- `ros2_livox_simulation`
- `livox_laser_simulation_RO2`
- Gazebo launch files
- `waver_patrol.test_nodes`

These can be used for validation but must not run in real profile defaults.

## 4. Test/Fake Publishers

Installed test publishers are acceptable only under explicit Gazebo/test launch flags. They are forbidden in indoor real defaults.

## 5. Bird/Target/Deterrence Extension

The bird stack is an extension over the minimal indoor patrol runtime. It requires camera model/configuration, 3D fusion, dynamic association, target tracking, and safety gates. Camera-only detection must not create navigation goals.

## 6. Mapping/SLAM Utilities

Mapping launch and scripts are used to create/apply maps. Mapping mode must not run patrol goals at the same time.

## 7. Evaluation/Reporting Scripts

Evaluation scripts and loggers write experiment data and must not own motion command topics.

## 8. Vendor/Upstream Packages

Vendor/upstream-like packages are preserved. Do not guess licenses or delete large data files without a separate upstream audit.

## 9. Future Package Split Recommendation

Documentation-only recommendation for a future cleanup:

- `waver_platform_driver`
- `waver_safety`
- `waver_mission`
- `waver_bringup`
- `waver_perception`
- `waver_sim`
- `waver_eval`
- `waver_operator_ui`

No package split is performed in this task.

## Command Chain Contract

```text
Nav2 or patrol candidate
  -> /waver/cmd_vel_nav2 or /waver/cmd_vel_nav2_smooth
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node or the selected single base driver
```

Manual control:

```text
waver_remote_panel
  -> /waver/manual_cmd_vel
  -> safety_cmd_mux_node
  -> /cmd_vel
```

Forbidden in real profile:

- `waver_remote_panel` publishing `/cmd_vel` directly.
- Nav2 controller publishing `/cmd_vel` directly.
- `collision_monitor` owning final `/cmd_vel`.
- Fake Gazebo/test publishers in real launch.
- Camera-only bird detection creating a navigation goal.

## Indoor Patrol Profile

Use `waver_indoor_patrol_real.launch.py` for low-speed indoor checks:

- `default_mode=STANDBY`
- `require_scan=true`
- `safety_max_linear_speed=0.05`
- `safety_max_angular_speed=0.20`
- Bird detector, bird 3D fusion, sound, target departure, experiment logger, and battery-return manager default off.

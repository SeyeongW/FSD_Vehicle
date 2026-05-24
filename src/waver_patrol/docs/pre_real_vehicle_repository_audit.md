# Waver Pre-Real Repository Audit

Workspace: `~/ros2_ws`
Repository branch checked during this audit: `jo`

## Build-Visible Packages

| Package | Path | Role | Launch files | Main nodes | Input topics | Output topics | Related config | Build status | Risk |
|---|---|---|---|---|---|---|---|---|---|
| `ugv_bringup` | `src/ugv_main/ugv_bringup` | Real robot bringup and Waveshare UGV driver integration | bringup launch files | UGV driver / serial bridge candidates | `/cmd_vel`, sensor topics | serial motor commands, robot topics | package launch/config | Not rebuilt in focused Gazebo pass | Must not run serial driver together with Waver serial bridge |
| `ugv_slam` | `src/ugv_main/ugv_slam` | 2D/3D mapping wrappers | gmapping/cartographer/rtabmap launch files | SLAM processes | `/scan`, `/odom`, `/tf` | `/map`, `map->odom` | SLAM configs/scripts | Not rebuilt in focused Gazebo pass | Mapping quality depends on live TF and LiDAR |
| `ugv_nav` | `src/ugv_main/ugv_nav` | Navigation2/TEB/DWA wrappers | nav/slam-nav launch files | Nav2 stack | `/map`, `/scan`, `/odom`, goals | controller candidate cmd_vel | Nav2 configs | Not rebuilt in focused Gazebo pass | Nav2 output must be remapped into Waver safety mux |
| `ugv_gazebo` | `src/ugv_main/ugv_gazebo` | Gazebo world/models including `ugv_rover` | `bringup.launch.py` and model/world assets | Gazebo plugins, bird manager | Gazebo state/services | `/odom`, `/scan`, `/cmd_vel` subscriber in sim | `ugv_world.world` | Built in focused pass | Livox plugin warning may appear if plugin is unavailable |
| `ugv_tools` | `src/ugv_main/ugv_tools` | Keyboard tools, visual remote panel | `waver_operator_panel.launch.py` | `waver_remote_panel` | `/map`, `/plan`, `/odom`, mission state | `/waver/manual_cmd_vel`, `/waver/mode`, E-stop topics | launch params | Built in focused pass | Operator panel must not publish final `/cmd_vel` by default |
| `ugv_vision` | `src/ugv_main/ugv_vision` | Camera/detection integration placeholder | vision launch files if present | camera/vision nodes | camera images | detection/classification topics | package config | Not rebuilt in focused Gazebo pass | Real detector is still external/stubbed |
| `waver_patrol` | `src/waver_patrol` | Waver mission, safety, target, logging, Gazebo validation | mission/Gazebo/pre-real launch files | mission manager, safety mux, target transform/filter, loggers | `/waver/lidar_objects`, `/odom`, `/scan`, `/waver/mode` | `/cmd_vel`, `/waver/*` mission/log topics | `config/*.yaml`, `waypoints/*.yaml` | Built in focused pass | Real robot gate requires live scan, TF, localization |
| `pcd_cluster_pkg` | Not found in active tree; archive copy exists under `~/ros2_ws/FSD_Vehicle` | Legacy/alternate PointCloud cluster package | N/A in active build | archive `cluster_node.py` | PointCloud2 | cluster centers | package config | Not active | Active tree uses `pointcloud_lidar_objects_node.py`; do not let any cluster node publish `/cmd_vel` |

## Important File Locations

- Waver mission launch: `src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py`
- Gazebo moving-object validation launch: `src/waver_patrol/launch/gazebo_moving_object_trial.launch.py`
- Pre-real wrapper launch: `src/waver_patrol/launch/gazebo_pre_real_validation.launch.py`
- Operator panel launch: `src/ugv_main/ugv_tools/launch/waver_operator_panel.launch.py`
- Operator panel UI: `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py`
- Moving object map transform: `src/waver_patrol/waver_patrol/perception/moving_object_map_transform_node.py`
- Height-based elevated dynamic filter: `src/waver_patrol/waver_patrol/perception/moving_object_motion_filter_node.py`
- Gazebo CSV logger: `src/waver_patrol/waver_patrol/logging/gazebo_trial_logger_node.py`
- Gazebo repeat runner: `src/waver_patrol/scripts/run_pre_real_gazebo_trials.sh`
- RViz config: `src/waver_patrol/rviz/pre_real_gazebo_validation.rviz`

## Duplicate Package Risk

The current build should target specific packages first:

```bash
colcon build --packages-select ugv_gazebo ugv_tools waver_patrol --symlink-install
```

Full workspace builds may expose unrelated third-party package conflicts in `ugv_else`.

## Cluster Package Note

`pcd_cluster_pkg/cluster_node.py` was requested, but no `pcd_cluster_pkg` or `cluster_node.py` exists in the active local `jo` tree. An archive copy exists at `~/ros2_ws/FSD_Vehicle/src/ugv_main/pcd_cluster_pkg`. The active real-robot path is `waver_patrol/perception/pointcloud_lidar_objects_node.py`, which converts `sensor_msgs/msg/PointCloud2` into `/waver/lidar_objects` and never publishes `/cmd_vel`. If `pcd_cluster_pkg` is restored later, it must publish perception topics only and must not publish `/cmd_vel`.

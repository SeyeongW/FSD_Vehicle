# Gazebo Moving Object Mission Trials

This test path verifies the Waver mission chain before real-robot use:

1. waypoint patrol starts from a saved-map style route,
2. a simulated moving object publishes cluster-center coordinates,
3. object coordinates are transformed into `map`/`odom`,
4. the elevated dynamic filter confirms height >= 3.0 m and real map/odom motion,
5. the target mission interrupts patrol,
6. the robot navigates to a safety-offset target goal,
7. body yaw alignment is observed near the target,
8. camera/classification and sound-mission stubs run,
9. patrol resumes, and
10. CSV/optional rosbag data are written for paper analysis.

## Key Safety Rule

Only `safety_cmd_mux_node` may publish final `/cmd_vel`.

`cluster_node.py`, Gazebo target scripts, moving-object filters, target-goal
managers, camera stubs, sound stubs, and logger nodes must not publish final
robot velocity. In this workspace `pcd_cluster_pkg/cluster_node.py` was not
present, so the Gazebo trial uses a simulation-only adapter that publishes the
same perception interface:

- `/waver/lidar_objects` (`geometry_msgs/msg/PoseArray`)

If a real `pcd_cluster_pkg` is added later, keep it as perception only and
connect its cluster centers to `/waver/lidar_objects` or `/lidar/detections`.

## Elevated Dynamic Target Definition

The 3 m condition is **height**, not travel distance. Waver must not trigger a
mission merely because an object moved 3 m. `moving_object_motion_filter_node`
now checks:

- `object_height_m >= target_min_height_m` where the default is `3.0`,
- `z_valid=true`,
- compensated map/odom motion or velocity exceeds a small dynamic threshold,
- raw sensor-frame motion alone is never used for the target decision.

Default parameters:

```yaml
target_min_height_m: 3.0
min_dynamic_motion_m: 0.2
min_dynamic_velocity_mps: 0.05
min_tracking_duration_sec: 1.0
static_motion_tolerance_m: 0.25
```

2D LaserScan has no z channel; such data must be treated as `z_valid=false`.
Gazebo may use fake 3D PoseArray/model-state data for pre-real validation.

### Required Height Trials

- H1: z=3.2 m moving object. Expected valid target.
- H2: z=3.2 m static object. Expected rejection as static/unknown.
- H3: z=1.0 m moving object. Expected rejection as low altitude.
- H4/H5/H6 are recommended for ego-rotation and z-missing stress tests.

## Launch

Build first:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --packages-select waver_patrol ugv_tools ugv_gazebo --symlink-install
source install/setup.bash
```

Run one GUI trial:

```bash
ros2 launch waver_patrol gazebo_moving_object_trial.launch.py \
  trial_id:=1 \
  target_min_height_m:=3.0 \
  min_dynamic_motion_m:=0.2 \
  target_z:=3.2 \
  world_file:=~/ros2_ws5/FSD_Vehicle/install/ugv_gazebo/share/ugv_gazebo/worlds/ugv_world.world \
  use_gui:=true \
  enable_cluster_node:=true \
  enable_experiment_logger:=true \
  record_bag:=false
```

The trial launch sets `require_scan:=false` and
`ignore_scan_when_require_scan_false:=true` for simulation timing only. Do not
use that combination on the real robot.

The launch defaults to the existing repository world
`ugv_gazebo/worlds/ugv_world.world`. That airport-style world already includes
`libgazebo_ros_state.so`, so the trial target can be moved by `/set_entity_state`
while the perception adapter publishes the matching `/waver/lidar_objects`
cluster-center interface. A custom world can still be supplied with
`world_file:=/absolute/path/to/file.world` when debugging a narrower scenario.

Gazebo Classic's planar move model in this workspace under-reports motion
relative to the commanded Twist, so the trial launch exposes
`gazebo_sim_max_linear_speed` as a simulation-only correction. Real-robot speed
limits remain in `config/waver_nav2_radar_bird_mission.yaml` and should start at
0.1 m/s or lower during field tests.

Run the 3-trial headless sequence:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/FSD_Vehicle/src/waver_patrol/scripts/run_gazebo_mission_trials.sh
```

Optional rosbag recording can create large files:

```bash
RECORD_BAG=true bash src/FSD_Vehicle/src/waver_patrol/scripts/run_gazebo_mission_trials.sh
```

Do not commit `experiments_result/rosbag`, `build`, `install`, or `log`.

## Trial Scenarios

| Trial | Object condition | Purpose |
| --- | --- | --- |
| H1 | z=3.2 m and moving about 0.7 m | Valid elevated dynamic target |
| H2 | z=3.2 m and static | Static high object; should not trigger |
| H3 | z=1.0 m and moving about 0.7 m | Low dynamic object; should not trigger |
| H4-H6 | rotation/static/z-missing stress cases | Recommended extended validation |

## Important Topics

| Topic | Type | Role |
| --- | --- | --- |
| `/waver/lidar_objects` | `PoseArray` | Raw/simulated cluster centers |
| `/waver/lidar_objects_map` | `PoseArray` | map/odom transformed object centers |
| `/waver/elevated_dynamic_targets` | `PoseArray` | height>=3m and map/odom dynamic target |
| `/waver/moving_target_valid` | `Bool` | compatibility bool; true only for elevated dynamic target |
| `/waver/aerial_target` | `PointStamped` | target point for mission/AI stubs |
| `/waver/object_mission_goal` | `PoseStamped` | safety-offset target goal |
| `/waver/mission_state` | `String` | mission state machine |
| `/waver/sim_nav2_state` | `String` | Gazebo-only simple Nav2 stand-in |
| `/waver/sound_alert_state` | `String` | safe sound-mission stub state |
| `/cmd_vel` | `Twist` | final command, safety mux only |

## CSV Outputs

Each trial creates:

```text
~/ros2_ws5/FSD_Vehicle/experiments_result/gazebo_trial_<id>_<timestamp>/
  mission_events.csv
  lidar_clusters.csv
  moving_object_tracks.csv
  target_map_coordinates.csv
  nav_goal_results.csv
  yaw_alignment.csv
  camera_detections.csv
  sound_mission.csv
  experiment_summary.csv
  rosbag/
  rviz_screenshots/
  plots/
```

The key summary columns are:

- `dynamic_motion_m` (small map/odom compensated motion used only for dynamic/static separation)
- `target_detected`
- `cluster_published`
- `map_transform_success`
- `target_goal_success`
- `yaw_alignment_success`
- `camera_detection_success`
- `sound_mission_success`
- `patrol_resume_success`
- `overall_success`
- `failure_reason`

Analyze all trials:

```bash
python3 src/FSD_Vehicle/src/waver_patrol/scripts/analyze_gazebo_trials.py \
  --input_dir ~/ros2_ws5/FSD_Vehicle/experiments_result \
  --output_dir ~/ros2_ws5/FSD_Vehicle/experiments_result/results
```

Optional plots:

```bash
python3 src/FSD_Vehicle/src/waver_patrol/scripts/plot_gazebo_trial_results.py \
  --input_dir ~/ros2_ws5/FSD_Vehicle/experiments_result/results \
  --output_dir ~/ros2_ws5/FSD_Vehicle/experiments_result/results
```

## Real Robot Gate

Do not move the real Waver until:

1. at least three Gazebo trials succeed,
2. `/cmd_vel` has one publisher: `safety_cmd_mux_node`,
3. cluster/perception nodes do not publish `/cmd_vel`,
4. `/scan`, `/odom`, TF, map/localization are valid,
5. sound output is disabled or dry-run,
6. serial bridge is started only after wheel-off-ground tests, and
7. initial real speed is limited to 0.1 m/s or lower.

Check command ownership:

```bash
ros2 topic info -v /cmd_vel
```

Expected final publisher: `safety_cmd_mux_node` only.

# Gazebo Height-Based Target Trials

The final target gate is height-based, not distance-travel based:

```text
object_height_m >= 3.0
and z_valid == true
and dynamic_filter_pass == true
and ego_motion_compensated == true
```

`dynamic_filter_pass` uses map/odom compensated motion.  Raw LiDAR-frame
displacement is logged for debugging only and must not trigger missions.

## Required Trials

| Trial | Scenario | Expected Result |
|---|---|---|
| H1 | Object at z=3.2 m moves in map frame | `elevated_dynamic_target_valid=true`, mission starts |
| H2 | Object at z=3.2 m is static | `elevated_dynamic_target_valid=false`, no target mission |
| H3 | Object at z=1.0 m moves | `elevated_dynamic_target_valid=false`, no target mission |

## Commands

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

OUTPUT_ROOT=$HOME/ros2_ws/experiments_result \
ROS_DOMAIN_BASE=180 \
TRIALS="1 2 3" \
REQUIRED_SUCCESSES=3 \
TARGET_MIN_HEIGHT_M=3.0 \
MIN_DYNAMIC_MOTION_M=0.2 \
MIN_DYNAMIC_VELOCITY_MPS=0.05 \
bash src/FSD_Vehicle/src/waver_patrol/scripts/run_pre_real_gazebo_trials.sh
```

For a stronger stability check:

```bash
OUTPUT_ROOT=$HOME/ros2_ws/experiments_result \
ROS_DOMAIN_BASE=190 \
TRIALS="1 2 3 1 2" \
REQUIRED_SUCCESSES=5 \
SEQUENCE_DURATION_SEC=5.0 \
bash src/FSD_Vehicle/src/waver_patrol/scripts/run_pre_real_gazebo_trials.sh
```

## Success Criteria

- H1 publishes `/waver/elevated_dynamic_targets`.
- H2 and H3 do not publish a valid target mission.
- `/waver/lidar_objects_map` preserves z after map/odom transform.
- `experiment_summary.csv` includes height and dynamic filter columns.
- `/cmd_vel` has exactly one publisher: `safety_cmd_mux_node`.
- Results are written under `~/ros2_ws/experiments_result`.

## Remaining Recommended Trials

- H4: robot yaw rotation with a high static object. Expected `static_due_to_ego_motion`.
- H5: robot yaw rotation with a real high moving object. Expected valid elevated dynamic object.
- H6: z-unknown 2D input. Expected `height_unknown` rejection.

# Autonomous Patrol Validation

Goal: validate that Waver can patrol safely and can be preempted by target coordinates before it finishes the current waypoint leg.

## Contract

```text
START_PATROL
  -> mission_patrol_manager_node
  -> waypoint/Nav2 or supervised field patrol candidate command
  -> safety_cmd_mux_node
  -> /cmd_vel
```

Target preemption:

```text
LiDAR/dynamic target pose
  -> target_goal_manager / mission manager
  -> active target approach goal
  -> safety_cmd_mux_node
```

The target must interrupt the current patrol behavior when policy permits; it must not wait until a waypoint is fully completed.

## Gazebo/Mock Validation

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh \
  --scenario autonomous_patrol_smoke \
  --cycles 2
```

For bird/dynamic target experiments, use the spatial response smoke script:

```bash
bash scripts/run_gazebo_lidar_spatial_response_smoke.sh
```

Expected evidence:

- patrol command starts.
- final `/cmd_vel` remains safety mux owned.
- target preemption metrics are recorded by the experiment verifier when the bird target scenario is run.

## Real-Field Limitation

If odom/localization is unavailable, the Docker field backend can use a supervised open-loop micro-patrol fallback. That is acceptable only for early low-speed wheel-off/wheel-on checks and is not equivalent to localization-based autonomy.

## Manual Checks Before Wheel-On

- map is loaded and AMCL initial pose is set, if using Nav2.
- `/odom` direction matches physical motion.
- waypoints are inside the known safe map.
- `/scan` is valid and not stale/degraded.
- speed limits remain conservative.

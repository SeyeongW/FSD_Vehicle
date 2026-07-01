# SLAM Mapping Validation

Goal: verify mapping mode, live map display, static obstacle persistence, saved map quality, and localization handoff before real use.

## Contract

- Mapping mode must not run patrol goals at the same time.
- Mapping start clears old fixed map display in the remote UI.
- Mapping mode `/map` publisher must be a live SLAM/map source, not a stale fixed map publisher.
- Saved map must be quality-checked before Nav2/localization use.

## Gazebo Validation

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh \
  --scenario slam_mapping_smoke \
  --cycles 2
```

The wrapper calls the existing UI SLAM smoke path with a static obstacle enabled when available:

```bash
WAVER_SPAWN_TEST_OBSTACLE=true \
WAVER_USE_GUI=false \
bash scripts/run_ui_slam_mapping_gazebo_smoke.sh
```

## Manual RViz/UI Confirmation

Check:

- RViz fixed frame is `map`.
- `/map` updates while Waver moves.
- static obstacle stays fixed in the map when the robot rotates.
- UI shows `SLAM_LIVE` or equivalent, not stale `MAP_FIXED`, during mapping.
- map save creates `maps/waver_latest_map.yaml`.
- saved map passes:

```bash
python3 src/waver_patrol/scripts/waver_map_quality_check.py maps/waver_latest_map.yaml
```

## Localization Confirmation

After applying a saved map:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo --once /amcl_pose
```

Do not claim localization readiness if `map->odom` is missing or AMCL pose is stale.

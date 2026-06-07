# ROS2 WS3 Bird Patrol Merge Report

Workspace: `/home/chotaehyun/ros2_ws3/FSD_Vehicle`

Branch: `jo`

## Source Protection

- `ros2_ws2` was used as local autonomous-driving/UI reference.
- `/home/chotaehyun/ros2_ws_backups/ros2_ws2_FULL_20260602_073757.zip` exists as the pre-integration backup.
- SEO branch archive: `/home/chotaehyun/ros2_ws3/_sources/FSD_Vehicle_seo`.
- SEO archive check: `git status --short` is empty.
- All integration files are under `ros2_ws3/FSD_Vehicle`.

Checksum audit against `/home/chotaehyun/ros2_ws_backups/ros2_ws2_src_before_20260602_073757.sha256` was rerun for `/home/chotaehyun/ros2_ws2/FSD_Vehicle/src` after the Gazebo verification. Result: changed file hashes = 0, missing files = 15, new files = 5. The missing files are the `src/ugv_else/gmapping/openslam_gmapping/log*` source files already absent in the current workspace; the new files are `src/waver_patrol/.pytest_cache/*` artifacts. This Gazebo integration did not require modifying `ros2_ws2`, but the current `ros2_ws2` source tree is not byte-for-byte identical to the saved baseline because of those missing/artifact files.

## Selected World And Assets

- Selected world: `src/ugv_main/ugv_gazebo/worlds/ugv_world.world`
- Robot: `src/ugv_main/ugv_gazebo/models/ugv_rover/model.sdf`
- Bird model: `src/ugv_main/ugv_gazebo/models/bird/model.sdf`
- Bird manager: `src/ugv_main/ugv_gazebo/scripts/bird_manager.py`
- Livox/Mid360 Gazebo plugin source: `src/livox_laser_simulation_RO2`

## Implemented Gazebo Integration

Entrypoint:

```text
src/ugv_main/ugv_gazebo/launch/bird_patrol/ugv_gazebo_bird_patrol_seo.launch.py
```

New packages:

```text
src/waver_seo_tracking
src/waver_experiment_logger
```

Core flow:

```text
remote UI START_PATROL
-> waypoint patrol
-> dynamic target lock
-> target-relative inspection offset goal
-> approach target
-> camera alignment
-> Gazebo fake bird classification
-> simulated sound stub/log
-> return to interrupted waypoint
-> resume patrol
```

## Command Authority

- `/cmd_vel`: `safety_cmd_mux_node` only.
- `/waver/mode`: `mission_patrol_manager_node` only.
- Candidate patrol command: `/waver/cmd_vel_nav2`.
- Candidate body tracking command: `/waver/cmd_vel_target_track`.
- Selected candidate command: `/waver/cmd_vel_auto_selected`.

## Gazebo Verification Evidence

A direct Gazebo + remote UI run was performed.

Latest successful mechanism logger state:

```text
SEEN=patrol,dynamic_target,lock,inspection_goal,camera_align,classification,sound,return_or_resume MISSING=none
```

Latest verified successful logs:

```text
experiments_result/gazebo_bird_patrol/gazebo_seo_bird_patrol_20260603_072230/mechanism_events.csv
experiments_result/gazebo_bird_patrol/gazebo_trial_01_20260603_072230/
```

Observed mission-state sequence:

```text
APPROACH_TARGET_OFFSET
TARGET_REACHED
CAMERA_ALIGN_DONE
TARGET_CLASSIFICATION_WAIT
SOUND_TASK_REQUESTED
SOUND_TASK_RUNNING
SOUND_TASK_DONE
RETURN_TO_INTERRUPTED_WAYPOINT
RESUME_PATROL
PATROL_NAVIGATING
```


Automated log verification:

```bash
python3 scripts/verify_ros2_ws3_bird_patrol_log.py \
  experiments_result/gazebo_bird_patrol/gazebo_seo_bird_patrol_20260603_072230/mechanism_events.csv
```

Result:

```text
MECHANISM_LOG_VERIFY=PASS rows=4358 nonzero_cmd_rows=3994 odom_dx=5.948 odom_dy=0.127
```

## Real Prep Structure

Real-prep files are structural only. No real motor/serial launch was executed.

```text
src/ugv_main/ugv_gazebo/param/bird_patrol/profiles/real_prep.yaml
src/ugv_main/ugv_gazebo/param/bird_patrol/topic_remaps_real_prep.yaml
src/ugv_main/ugv_gazebo/param/bird_patrol/TF_CHECKLIST_REAL_PREP.md
```

The real-prep profile requires real detector, real Mid360 pointcloud, real camera info/image, and final command through `safety_cmd_mux_node`.

## Source Caveats

The final Gazebo launch uses `waver_remote_panel` only as an operator UI process and starts the verified demo path through the launch-level `remote_panel_demo_script:=start_patrol_once` argument. The START command observed in the latest run came from the UI node log line `operator command sent: START_PATROL`; no direct CLI mission-command injection was used for that latest verification.

The current `ros2_ws3` worktree still contains pre-existing field/Jetson dirty changes in `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py` and other real/field files. The Gazebo mechanism does not rely on the SSH remote bridge portion of those dirty UI changes, but the source-protection audit should treat that dirty state as a caveat until those unrelated field changes are either committed separately or restored by the owner.

## Limitations

- Gazebo classification is fake/demo classification, not measured bird mAP/precision/recall.
- Real field validation still requires camera-LiDAR calibration, Mid360 frame validation, real bird model, physical E-stop, battery validation, and serial command direction verification.

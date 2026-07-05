# Remote UI SLAM + Bird Detection Feature Audit

Date: 2026-07-05
Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
Branch: `jo`

## User Requirement Added

The remote panel must support SLAM mapping while bird-detection state remains visible and live.

Allowed during mapping:

- SLAM live map display and save/apply workflow.
- Manual WASD mapping movement through `/waver/manual_cmd_vel`.
- Bird/perception status display from detector/fusion topics.
- LiDAR target overlay/status if the perception stack is enabled.

Blocked during mapping unless a dedicated test mode explicitly enables it:

- Patrol waypoint navigation.
- Target-approach goal generation.
- Sound deterrent mission.
- Any extra `/map`, `/cmd_vel`, or `/waver/mode` publisher.

## Remote UI Feature Checklist

Source inspected: `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py`.

| UI area | Topic/action | Source status |
| --- | --- | --- |
| Manual WASD | publish `/waver/manual_cmd_vel` | present |
| Final command safety | does not publish `/cmd_vel` by default | present |
| Mode command | publish `/waver/mode_cmd` | present |
| Mission command | publish `/waver/mission_command` | present |
| SLAM mapping buttons | publish `/waver/mapping_command` | present |
| Mapping reset | subscribe `/waver/ui_map_reset` | present |
| Map source | subscribe/display `/waver/current_map_source` | present |
| Live map | subscribe `/map`, separate `/map_fixed` | present |
| Mapping path | can use `/waver/mapping_path` as global path | present in mapping launch |
| LiDAR target | subscribe/display `/waver/lidar_target_*` topics | present |
| Bird class | subscribe/display `/waver/target_class` | present |
| Bird confidence | subscribe/display `/waver/target_confidence` | present |
| Bird confirmed | subscribe/display `/waver/bird_confirmed` | present |
| Bird detector state | subscribe/display `/waver/bird_detector_state` with freshness age | present |
| Bird fusion state | subscribe/display `/waver/bird_fusion_state` with freshness age | present |
| Camera alignment | subscribe/display `/waver/camera_alignment_state`, `/waver/camera_target_centered` | present |
| Sound status | subscribe/display `/waver/sound_alert_state`, `/waver/sound_task_done` | present |
| Remote bridge | SSH/docker bridge for manual/mode/mission/mapping commands | present |
| E-stop | publish `/waver/emergency_stop` | present |
| Speed limits | publish `/waver/speed_limit`, `/waver/angular_speed_limit` | present |

## Current Simulation Evidence

Already proven separately:

- UI SLAM mapping smoke passed with obstacle map creation and map quality PASS.
- Saved-map Nav2 static obstacle check passed.
- Gazebo bird patrol mechanism smoke passed with LiDAR target, target approach, sound task, bird departure/removal, return/resume.

Now proven as one combined simulation regression:

- Remote UI SLAM mapping and bird detector/fusion running at the same time.
- UI showing `SLAM_LIVE_MAP` while `/waver/target_class`, `/waver/target_confidence`, `/waver/bird_confirmed`, `/waver/bird_detector_state`, `/waver/bird_fusion_state`, and LiDAR target state update live.

Latest command:

```bash
ROS_DOMAIN_ID=62 \
GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
TIMEOUT_SEC=220 \
WAVER_USE_GUI=false \
WAVER_START_RVIZ=false \
bash scripts/run_ui_slam_bird_detection_gazebo_smoke.sh
```

Latest result:

- `UI_SLAM_BIRD_DETECTION_SMOKE=PASS`
- `reports/ui_slam_bird_detection/latest.json`
- `current_map_source_slam_live=True`
- `bird_topics_visible=True`
- `bird_topics_fresh=True`
- `bird_detector_state_fresh=True`
- `bird_fusion_state_fresh=True`
- `mapping_path_visible=True`
- `no_patrol_conflict=True`
- `no_target_approach_without_arm=True`
- `no_sound_without_arm=True`
- `/map`, `/cmd_vel`, and `/waver/mode` publisher counts are each one.

## Why Pure Mapping Debug Still Remains Isolated

`waver_gazebo_mapping_debug.launch.py` intentionally disables bird/target pointcloud nodes to isolate `/scan`, `/odom`, TF, `/map`, and UI SLAM behavior. That pure debug mode is still useful and should remain isolated.

The combined UI regression is separate:

`scripts/run_ui_slam_bird_detection_gazebo_smoke.sh`

## Real Field Caveat

For real Waver hardware, bird detection during SLAM mapping requires live evidence from:

- Livox Mid-360 pointcloud or scan adapter.
- Camera image and camera info.
- Camera-LiDAR TF calibration.
- Detector model/probe report.
- Fusion readiness report.
- Safety chain with single final `/cmd_vel` publisher.

Until those probes pass, the correct status is:

- UI supports the combined display path.
- Gazebo has separate SLAM and bird mechanism evidence.
- Integrated SLAM+bird UI regression is still required before claiming complete readiness.

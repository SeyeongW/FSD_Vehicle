# Bird Detection Autonomy Validation

This project is a bird-detection autonomous patrol system. SLAM, map save/apply,
Nav2 patrol, and the operator UI are support gates; they must not be presented
as bird-detection performance metrics.

## Evaluation Priority

1. Bird detection:
   precision, recall, F1, mAP@50, mAP@50:95, false positive, false negative,
   confidence distribution, latency, and FPS.
2. Bird target localization and tracking:
   z-valid rate, height/range/bearing error, map/odom position error, track
   continuity, ID switch count, lost track count, reacquisition, and ego-motion
   compensation success.
3. Bird mission trigger:
   trigger precision/recall/F1, trigger latency, mapping-mode wrong trigger
   count, patrol-mode valid trigger count, and bird target to mission-state
   transition.
4. Navigation and safety:
   patrol completion, return home, STOP, E-STOP, single `/cmd_vel` publisher,
   stale sensor stops, and obstacle stops.
5. SLAM/map support:
   LiDAR-only `/scan` SLAM, scan/odom/TF health, map known ratio, map save/apply,
   and fixed-map UI reflection.

## Implemented Gazebo Pipeline

The current `jo` branch adds a Gazebo-only synthetic bird evaluation bridge:

- Node: `bird_detection_pipeline_node`
- Source file:
  `src/waver_patrol/waver_patrol/perception/bird_detection_pipeline_node.py`
- Launch:
  `src/waver_patrol/launch/gazebo_bird_detection_validation.launch.py`
- Repeated-run script:
  `src/waver_patrol/scripts/run_bird_detection_gazebo_ui_trials.sh`

The node uses Gazebo model-state provenance for a bird model already present in
`ugv_gazebo/worlds/ugv_world.world`. It publishes bird-specific topics:

- `/bird/ground_truth`
- `/bird/detections_2d`
- `/bird/detections_3d`
- `/bird/tracks`
- `/bird/metrics`
- `/bird/mission_debug`
- `/bird/mission_target`

Compatibility topics for the existing mission stack are also published:

- `/waver/bird_confirmed`
- `/waver/target_class`
- `/waver/target_confidence`
- `/waver/camera_detection_state`
- `/waver/elevated_dynamic_targets`

The node never publishes `/cmd_vel`.

## Target Hierarchy

The mission target is not a generic moving object.

```text
bird_candidate
  -> spatial_valid_bird
  -> elevated_bird_target
  -> dynamic_bird_target
  -> mission_trigger_bird
```

`mission_trigger_bird` requires all of the following:

- object class is bird
- z source provenance is valid
- height is at least 3.0 m
- map/odom-frame compensated motion or velocity is dynamic
- ego-motion compensation is applied
- current mode allows patrol mission trigger
- mapping mode is not active
- safety state does not block trigger

2D LaserScan alone is not a valid source for elevated bird height.

## Current 10-Run Result

Latest run:

```text
experiments_result/paper_ready/bird_detection_10runs
```

Summary:

- valid pass runs: 10/10
- bird precision mean: 1.000
- bird recall mean: 1.000
- bird F1 mean: 1.000
- average scan Hz: 16.9783
- average odom Hz: 83.978
- `/cmd_vel` publisher count: 1 in all final trials
- `/cmd_vel` publisher node: `safety_cmd_mux_node`
- direct `/cmd_vel` violation: 0

Important limitation:
this is Gazebo synthetic ground-truth evidence. It is useful for validating the
ROS2 topic pipeline, mission trigger gates, UI command path, and safety command
path. It is not a real-camera YOLO mAP claim.

## Run Commands

Build:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_tools waver_patrol ugv_slam ugv_gazebo --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=0
```

Single Gazebo bird validation launch:

```bash
ros2 launch waver_patrol gazebo_bird_detection_validation.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  scenario_id:=B3_DYNAMIC_BIRD_HIGH \
  expected_bird:=true \
  expected_mission_trigger:=true \
  move_target_model:=true \
  output_dir:=$HOME/ros2_ws5/FSD_Vehicle/experiments_result/paper_ready/manual_bird_trial/csv
```

Operator UI only:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  map_display_mode:=auto \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  publish_direct_cmd_vel:=false
```

Final 10-run Gazebo/UI bird pipeline validation:

```bash
RUNS=10 \
REQUIRED_SUCCESSES=10 \
USE_GUI=true \
TRIAL_DURATION_SEC=6 \
HZ_SAMPLE_SEC=5 \
OUTPUT_ROOT=$HOME/ros2_ws5/FSD_Vehicle/experiments_result/paper_ready/bird_detection_10runs \
bash src/waver_patrol/scripts/run_bird_detection_gazebo_ui_trials.sh
```

## External References

- ROS 2 Humble documentation:
  https://docs.ros.org/en/humble/
- Nav2 documentation:
  https://docs.nav2.org/
- Nav2 Collision Monitor tutorial:
  https://docs.nav2.org/tutorials/docs/using_collision_monitor.html
- Ultralytics ROS quickstart:
  https://docs.ultralytics.com/guides/ros-quickstart
- ByteTrack paper:
  https://arxiv.org/abs/2110.06864
- ByteTrack reference implementation:
  https://github.com/FoundationVision/ByteTrack

## Paper-Use Guidance

Allowed claim:

> Gazebo model-state ground truth was used to validate the Waver ROS2 bird
> target pipeline from bird candidate publication through 3D localization,
> tracking, patrol-mode mission trigger, operator UI command path, and safety
> command mux.

Not allowed:

> The real bird detector has mAP 1.0.

The current run does not use a real camera detector. Real bird detection claims
require a labeled image/video dataset, detector model card, inference hardware
description, threshold/NMS settings, and precision/recall/F1/mAP computed
against image-level ground truth.

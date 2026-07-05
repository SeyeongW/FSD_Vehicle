# Final Bird Patrol Architecture

The final product path is a real Waver hardware mission: waypoint patrol, 3D LiDAR dynamic target detection, safe standoff inspection, camera classification, camera-LiDAR fusion, non-contact deterrence, departure monitoring, and patrol resume.

Pipeline:

`Nav2 waypoint patrol -> MID-360 PointCloud2 -> elevated object candidate extraction -> dynamic target filtering -> map-frame target transform -> standoff inspection goal -> camera/body/gimbal alignment -> YOLO bird detector -> camera-LiDAR fusion -> bird_confirmed + fusion_valid + dynamic_valid + centered -> sound deterrent request -> target departure or timeout -> patrol resume`

| Stage | Node | Inputs | Outputs | ACTIVE Condition | DEGRADED/BLOCKED/FAIL_CLOSED |
| --- | --- | --- | --- | --- | --- |
| Base and odom | `waver_base_driver_node`, `robot_localization/ekf_node` | `/cmd_vel`, serial feedback, IMU | `/odom`, `/waver/base_driver_state`, TF | Single serial owner, feedback schema loaded, odom feedback OK | Missing/stale feedback blocks driving readiness. Timeout sends zero command. |
| Safety command chain | `safety_cmd_mux_node`, optional `nav2_collision_monitor` | manual/Nav2/mission cmd candidates, scan, mode, E-stop | `/cmd_vel` or `/waver/cmd_vel_safety` | Exactly one final publisher, speed caps applied | E-stop/external stop/bad scan forces zero. Missing collision monitor blocks autonomous-patrol claim. |
| Patrol | `mission_patrol_manager_node`, Nav2 | `/waver/mission_command`, route YAML, Nav2 action | `/waver/mission_state`, goals | AUTO/START_PATROL and map/localization/safety ready | STANDBY/MANUAL/MAPPING/EMERGENCY blocks bird interrupt. |
| LiDAR elevated candidate | `pointcloud_lidar_objects_node`, `lidar_aerial_motion_detector_node` | `/livox/lidar`, TF | `/waver/lidar_objects` | PointCloud2 live, TF to base/map valid, axis params valid | Bad TF/axis/no points publishes safe state and no target. |
| Dynamic filtering | `moving_object_motion_filter_node`, `moving_object_map_transform_node` | `/waver/lidar_objects`, odom/map TF | `/waver/elevated_dynamic_targets`, `/waver/moving_target_valid` | z/range/velocity/persistence gates pass | Static, ground, robot-body, stale, or jumpy target rejected. |
| Inspection goal | `target_goal_manager_node` | dynamic targets, robot pose, bird/fusion state, safety/battery state | `/waver/inspection_target_pose_map`, `/waver/object_mission_goal` | LiDAR dynamic target can create standoff inspection goal | LiDAR-only target cannot trigger sound. Direct collision goal forbidden. |
| Alignment | `camera_gimbal_controller_node` | `/waver/camera_aim_target_pose`, TF, optional bbox/gimbal feedback | `/waver/camera_alignment_state`, `/waver/camera_target_centered`, fallback yaw cmd | `robot_body` or `real_gimbal` backend has feedback and target is centered | `topic_only` is dry-run only; production centered remains false. |
| Detector | `bird_detector_node` | `/camera/image_raw`, `/camera/camera_info`, model | `/waver/bird_detections_2d`, `/waver/bird_confirmed`, `/waver/target_class`, `/waver/target_confidence` | Model/import/camera ready and latency OK | Missing model/import/camera fail-closed with `bird_confirmed=false`. |
| Fusion | `bird_3d_fusion_node` | detections, camera_info, PointCloud2, dynamic targets, TF | `/waver/bird_fusion_state`, `/waver/bird_target_valid`, target poses | Synchronized packet, calibrated extrinsic, bbox points, height/range/spread, dynamic association | Invalid reasons are explicit and `bird_target_valid=false`. |
| Deterrence | `sound_deterrent_node` | sound request, class/confidence, bird confirmed, fusion valid, centered, safety | `/waver/sound_alert_state`, active/done/event_id | Only with bird + fusion + dynamic + centered + safety + ack | Disabled backend or missing ack blocks real output. E-stop cancels. |
| Departure/resume | `target_departure_monitor_node`, `mission_patrol_manager_node` | target pose/validity, sound done, Nav2 state | `/waver/target_departed`, mission state | Target departure or timeout observed | Target lost, non-bird, camera failure, approach failure returns to patrol/fails safe. |
| Blackbox | `waver_blackbox_recorder.sh` | mission topic set | rosbag + metadata | Recorder active for field evidence | Heavy camera/lidar topics are opt-in but metadata is always recorded. |

The current implementation is intended to keep all bird-mission functions in the production pipeline. Functions without field evidence are implemented but not yet field-validated, and must report `BLOCKED`/`DEGRADED` rather than being advertised as autonomous-ready.

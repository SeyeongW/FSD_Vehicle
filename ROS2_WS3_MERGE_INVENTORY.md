# ROS2 WS3 Gazebo Bird Patrol Merge Inventory

Workspace: `/home/chotaehyun/ros2_ws3/FSD_Vehicle`

Branch: `jo`

Reference-only sources:

- Local autonomous driving/UI reference: `/home/chotaehyun/ros2_ws2/FSD_Vehicle`
- SEO branch archive: `/home/chotaehyun/ros2_ws3/_sources/FSD_Vehicle_seo`

Protection result:

- `ros2_ws2` was backed up before creating this sandbox.
- The SEO branch archive is read-only and is not modified.
- Gazebo integration is added inside `ros2_ws3` only.
- Final launch entrypoint is in `ugv_gazebo`, not `waver_patrol`.

Selected SEO/Gazebo assets:

- World: `src/ugv_main/ugv_gazebo/worlds/ugv_world.world`
- Robot model: `src/ugv_main/ugv_gazebo/models/ugv_rover/model.sdf`
- Bird model: `src/ugv_main/ugv_gazebo/models/bird/model.sdf`
- Bird runtime manager: `src/ugv_main/ugv_gazebo/scripts/bird_manager.py`

New integration package:

- `src/waver_seo_tracking`

Gazebo-only adapters:

- `seo_cluster_tracker_node`
- `seo_bird_yolo_node`
- `seo_camera_tilt_joint_node`
- `seo_observation_body_tracker_node`
- `mission_state_cmd_selector_node`
- `remote_ui_patrol_adapter_node`
- `scan_alias_node`

New logger package:

- `src/waver_experiment_logger`

Gazebo bird patrol entrypoint:

- `src/ugv_main/ugv_gazebo/launch/bird_patrol/ugv_gazebo_bird_patrol_seo.launch.py`

Config files:

- `src/ugv_main/ugv_gazebo/param/bird_patrol/ugv_gazebo_bird_patrol_seo.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/seo_tracking_gazebo.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/experiment_logging.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/patrol_waypoints_square_4m_7m.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/profiles/gazebo.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/profiles/real_prep.yaml`
- `src/ugv_main/ugv_gazebo/param/bird_patrol/topic_remaps_real_prep.yaml`

Command authority:

- `/waver/mode`: `mission_patrol_manager_node`
- `/waver/cmd_vel_nav2`: `simple_nav2_cmd_sim_node`
- `/waver/cmd_vel_target_track`: `seo_observation_body_tracker_node`
- `/waver/cmd_vel_auto_selected`: `mission_state_cmd_selector_node`
- `/cmd_vel`: `safety_cmd_mux_node`

Gazebo classification note:

- `seo_bird_yolo_node` is a Gazebo-only fake/YOLO placeholder.
- Real profile must use a real detector and must not enable fake classification.

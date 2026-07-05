# Waver Worktree Inventory

- workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- branch: `jo`
- head: `170c38b`
- total changed/untracked rows: 155

## Policy

- `build/`, `install/`, and `log/` are generated artifacts and must never be edited as source.
- `src/livox_ros_driver2` is treated as `VENDOR_OR_SUBMODULE` until the Livox overlay policy is finalized.
- `scripts/waver_field_docker_backend_start.sh`, `scripts/waver_field_local_ui_start.sh`, `scripts/waver_field_lidar_nav_backend_start.sh`, and `scripts/waver_start_field_backend.sh` are field bridge critical and require regression checks.
- Local operator PC scripts are `LOCAL_RUNTIME_REQUIRED`; they must not start real robot backend nodes locally.
- Probe JSON files and source manifests under `reports/` are generated evidence, not hand-edited source.

## Critical Field Bridge Scripts

- `scripts/waver_field_docker_backend_start.sh`: present
- `scripts/waver_field_lidar_nav_backend_start.sh`: present
- `scripts/waver_field_local_ui_start.sh`: present
- `scripts/waver_start_field_backend.sh`: present

## Categories

### FIELD_BRIDGE_CRITICAL

- `M` `scripts/waver_field_docker_backend_start.sh`
- `M` `scripts/waver_field_lidar_nav_backend_start.sh`
- `M` `scripts/waver_field_local_ui_start.sh`
- `M` `scripts/waver_start_field_backend.sh`

### FIELD_RELEASE_REQUIRED

- `??` `README_BIRD_PATROL_FIELD.md`
- `??` `config/perception/`
- `??` `config/real_profiles/bird_patrol_production.yaml`
- `??` `config/sensors/`
- `??` `scripts/check_bird_mission_field_release.py`
- `??` `scripts/make_bird_mission_field_release.py`
- `??` `scripts/run_bird_mission_release_self_tests.sh`
- `??` `scripts/waver_bird_detector_probe.py`
- `??` `scripts/waver_bird_mission_readiness_check.py`
- `??` `scripts/waver_bird_mission_supervisor.py`
- `??` `scripts/waver_bird_patrol_field_start.sh`
- `??` `scripts/waver_camera_lidar_calibration_check.py`
- `??` `scripts/waver_camera_probe.py`
- `??` `scripts/waver_command_chain_check.py`
- `??` `scripts/waver_field_bridge_regression_check.py`
- `??` `scripts/waver_launch_contract_check.py`
- `??` `scripts/waver_livox_mid360_probe.py`
- `??` `src/waver_patrol/launch/bird_patrol_production.launch.py`

### GENERATED_REPORT

- `M` `reports/source_manifest.json`
- `M` `reports/source_sha256_manifest.csv`

### LOCAL_RUNTIME_REQUIRED

- `??` `docs/LOCAL_OPERATOR_STATION.md`
- `??` `scripts/waver_check_local_operator_deps.py`
- `M` `scripts/waver_create_home_field_env.sh`
- `M` `scripts/waver_field_env_load.sh`
- `??` `scripts/waver_field_operator_station_start.sh`
- `??` `scripts/waver_field_rviz_start.sh`
- `M` `scripts/waver_setup_local_pc.sh`

### SIM_ONLY

- `M` `src/ugv_main/ugv_gazebo/launch/bird_patrol/ugv_gazebo_bird_patrol_seo.launch.py`
- `M` `src/ugv_main/ugv_gazebo/launch/ui_slam/ugv_gazebo_ui_slam_mapping.launch.py`
- `M` `src/ugv_main/ugv_gazebo/param/bird_patrol/seo_tracking_gazebo.yaml`
- `M` `src/ugv_main/ugv_gazebo/param/bird_patrol/ugv_gazebo_bird_patrol_seo.yaml`

### SOURCE_OPTIONAL

- `M` `gitignore`
- `??` `reports/bird_mission_readiness_audit.md`
- `??` `reports/bird_mission_regression_audit.md`
- `??` `reports/cleanup_plan.md`
- `??` `reports/command_chain/`
- `??` `reports/field_bridge_regression/`
- `M` `reports/field_hardware_readiness_audit.md`
- `??` `reports/final_repair_validation_summary.md`
- `??` `reports/launch_contract/`
- `??` `reports/livox_submodule_policy.md`
- `??` `reports/network/`
- `??` `reports/release_self_test/`
- `??` `reports/remote_ui_slam_bird_feature_audit.md`
- `??` `reports/sim_regression/`
- `??` `reports/ui_slam_bird_detection/`
- `??` `reports/worktree_inventory.md`
- `??` `requirements-jetson-perception.txt`

### SOURCE_REQUIRED

- `M` `README.md`
- `M` `README_REAL_VEHICLE.md`
- `M` `README_ROS2_WS5_BIRD_PATROL_GAZEBO.md`
- `??` `config/hardware_acceptance_matrix.yaml`
- `M` `docs/CLONE_TO_FIELD.md`
- `??` `docs/RVIZ_FIELD_RUNBOOK.md`
- `M` `docs/SETUP_LOCAL_PC.md`
- `??` `docs/bird_mission_readiness_levels.md`
- `??` `docs/final_bird_patrol_architecture.md`
- `M` `docs/hardware_readiness_levels.md`
- `??` `docs/repository_cleanup_policy.md`
- `M` `docs/technical_report.md`
- `M` `scripts/check_field_release.py`
- `M` `scripts/check_remote_ui_slam_mapping_result.py`
- `??` `scripts/check_ui_slam_bird_detection_result.py`
- `??` `scripts/generate_bird_mission_readiness_audit.py`
- `M` `scripts/make_field_release.py`
- `M` `scripts/make_source_archive.py`
- `M` `scripts/run_gazebo_lidar_spatial_response_smoke.sh`
- `??` `scripts/run_ui_slam_bird_detection_gazebo_smoke.sh`
- `M` `scripts/waver_blackbox_recorder.sh`
- `M` `scripts/waver_clone_to_run_acceptance.sh`
- `??` `scripts/waver_conservative_cleanup_plan.py`
- `M` `scripts/waver_doctor.sh`
- `M` `scripts/waver_field_bootstrap_jetson.sh`
- `M` `scripts/waver_field_readiness_check.py`
- `M` `scripts/waver_field_stop_all.sh`
- `??` `scripts/waver_hardware_acceptance_update.py`
- `??` `scripts/waver_health_supervisor.py`
- `M` `scripts/waver_quality_gate.sh`
- `M` `scripts/waver_quickstart_field.sh`
- `??` `scripts/waver_ros_network_check.py`
- `M` `scripts/waver_setup_livox_mid360_docker.sh`
- `??` `scripts/waver_worktree_inventory.py`
- `M` `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py`
- `M` `src/waver_experiment_logger/waver_experiment_logger/waver_spatial_response_logger_node.py`
- `M` `src/waver_patrol/config/collision_monitor_waver.yaml`
- `M` `src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml`
- `M` `src/waver_patrol/launch/remote_visualization.launch.py`
- `??` `src/waver_patrol/launch/sensor_static_transforms.launch.py`
- `??` `src/waver_patrol/launch/waver_gazebo_mapping_bird_detection.launch.py`
- `M` `src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py`
- `M` `src/waver_patrol/launch/waver_real_bird_autonomy.launch.py`
- `M` `src/waver_patrol/setup.py`
- `??` `src/waver_patrol/test/test_archive_root_normalization.py`
- `??` `src/waver_patrol/test/test_base_driver_launch_uses_feedback_schema.py`
- `??` `src/waver_patrol/test/test_bird_detector_deployment_contract.py`
- `??` `src/waver_patrol/test/test_bird_detector_probe_warmup_contract.py`
- `??` `src/waver_patrol/test/test_bird_entrypoint_uses_production_launch.py`
- `??` `src/waver_patrol/test/test_bird_mission_field_release.py`
- `??` `src/waver_patrol/test/test_bird_mission_not_removed.py`
- `??` `src/waver_patrol/test/test_bird_mission_policy.py`
- `??` `src/waver_patrol/test/test_bird_mission_readiness_checker.py`
- `??` `src/waver_patrol/test/test_bird_model_registry_contract.py`
- `??` `src/waver_patrol/test/test_bird_patrol_production_profile.py`
- `??` `src/waver_patrol/test/test_bird_readiness_v2_contract.py`
- `??` `src/waver_patrol/test/test_bird_ready_degraded_policy.py`
- `??` `src/waver_patrol/test/test_bird_release_hygiene_contract.py`
- `??` `src/waver_patrol/test/test_blackbox_stop_field_contract.py`
- `??` `src/waver_patrol/test/test_calibration_checker_live_contract.py`
- `??` `src/waver_patrol/test/test_camera_alignment_bbox_error_pipeline.py`
- `??` `src/waver_patrol/test/test_camera_alignment_production_contract.py`
- `??` `src/waver_patrol/test/test_camera_lidar_fusion_gate.py`
- `??` `src/waver_patrol/test/test_camera_probe_numeric_contract.py`
- `??` `src/waver_patrol/test/test_collision_monitor_production_chain.py`
- `M` `src/waver_patrol/test/test_collision_monitor_topology.py`
- `??` `src/waver_patrol/test/test_command_chain_collision_monitor_contract.py`
- `??` `src/waver_patrol/test/test_default_backend_uses_strict_readiness.py`
- `??` `src/waver_patrol/test/test_detector_alignment_deadlock_contract.py`
- `??` `src/waver_patrol/test/test_field_bridge_regression_contract.py`
- `??` `src/waver_patrol/test/test_field_release_clone_to_run.py`
- `??` `src/waver_patrol/test/test_field_sound_ack_pass_through.py`
- `??` `src/waver_patrol/test/test_fusion_readiness_uses_calibration_probe.py`
- `??` `src/waver_patrol/test/test_hardware_acceptance_matrix_gate.py`
- `??` `src/waver_patrol/test/test_health_supervisor_contract.py`
- `??` `src/waver_patrol/test/test_launch_contract_required_args.py`
- `??` `src/waver_patrol/test/test_legacy_open_loop_requires_explicit_gate.py`
- `??` `src/waver_patrol/test/test_livox_field_config_policy.py`
- `??` `src/waver_patrol/test/test_livox_probe_json_contract.py`
- `??` `src/waver_patrol/test/test_local_operator_station_contract.py`
- `??` `src/waver_patrol/test/test_readiness_truthfulness_contract.py`
- `M` `src/waver_patrol/test/test_real_profile_source_of_truth.py`
- `M` `src/waver_patrol/test/test_real_profiles_contract.py`
- `??` `src/waver_patrol/test/test_release_generation_no_source_mutation.py`
- `??` `src/waver_patrol/test/test_remote_ui_slam_bird_contract.py`
- `??` `src/waver_patrol/test/test_repository_cleanup_policy.py`
- `??` `src/waver_patrol/test/test_ros_domain_policy.py`
- `??` `src/waver_patrol/test/test_seo_camera_alignment_tf_contract.py`
- `??` `src/waver_patrol/test/test_sound_backend_interface.py`
- `??` `src/waver_patrol/test/test_sound_deterrent_real_backend_gate.py`
- `??` `src/waver_patrol/test/test_ui_slam_bird_detection_smoke_contract.py`
- `M` `src/waver_patrol/test/test_ui_slam_mapping_configs.py`
- `??` `src/waver_patrol/test/test_worktree_inventory_policy.py`
- `M` `src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py`
- `M` `src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py`
- `M` `src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py`
- `??` `src/waver_patrol/waver_patrol/mission/bird_mission_supervisor_node.py`
- `M` `src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py`
- `M` `src/waver_patrol/waver_patrol/perception/bird_3d_fusion_node.py`
- `M` `src/waver_patrol/waver_patrol/perception/bird_detector_node.py`
- `M` `src/waver_seo_tracking/waver_seo_tracking/seo_camera_tilt_joint_node.py`
- `M` `src/waver_seo_tracking/waver_seo_tracking/seo_observation_body_tracker_node.py`

### VENDOR_OR_SUBMODULE

- `m` `src/livox_ros_driver2`

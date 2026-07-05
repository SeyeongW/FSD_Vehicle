# Bird Mission Regression Audit

Generated as a source-level audit for the production bird patrol hardening pass.

| Item | Status | Notes |
| --- | --- | --- |
| `pointcloud_lidar_objects_node.py` | OK | LiDAR elevated candidate node is present. |
| `lidar_aerial_motion_detector_node.py` | OK | LiDAR aerial detector node is present. |
| `moving_object_motion_filter_node.py` | OK | Dynamic target filtering node is present. |
| `moving_object_map_transform_node.py` | OK | Map/odom target transform node is present. |
| `target_goal_manager_node.py` | OK | Standoff inspection policy exists. |
| `mission_patrol_manager_node.py` | OK | Patrol interrupt, classification, sound request, return/resume states exist. |
| `bird_detector_node.py` | OK | Fail-closed detector wrapper exists and was hardened for machine-readable state. |
| `bird_3d_fusion_node.py` | OK | Fusion node exists and is being hardened for sync/invalid reason contracts. |
| `camera_gimbal_controller_node.py` | OK | Alignment node exists and topic-only production centered is now blocked. |
| `sound_deterrent_node.py` | OK | Existing node lives under `bridges/` and remains the production deterrence interface. |
| `safety_cmd_mux_node.py` | OK | Final safety command mux exists. |
| `waver_base_driver_node.py` | OK | Single serial owner driver exists with feedback state. |
| `waver_real_bird_autonomy.launch.py` | OK | Real launch exists. |
| `bird_patrol_production.launch.py` | OK | Added product-oriented wrapper launch. |

No required bird mission function was removed. Capabilities without field evidence are kept in the production pipeline and reported as `BLOCKED`, `DEGRADED`, or `FAIL_CLOSED`.

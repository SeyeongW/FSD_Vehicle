# External Research Summary

Date checked: 2026-05-26
Scope: ROS 2 Humble, Nav2, SLAM/map visualization, tf2, rosbag2, Gazebo, 3D LiDAR clustering, ego-motion compensation, and pre-real validation for Waver.

## Official Documentation

| Source | Link | Project Rule Applied |
|---|---|---|
| ROS 2 Humble tf2 concepts | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html | Object coordinates must be transformed through the TF tree before target decisions. Raw LiDAR-frame motion is debug-only. |
| ROS 2 Humble tf2 listener Python tutorial | https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html | Perception transform nodes must catch transform exceptions and continue running. |
| ROS 2 Humble rosbag2 record/replay | https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html | Gazebo and real-robot trials should preserve `/tf`, `/odom`, `/map`, `/scan`, target, mission, safety, and camera/sound topics. |
| Nav2 concepts and NavigateToPose action flow | https://docs.nav2.org/concepts/index.html | Mission manager should use Nav2 goal/action interfaces and let planner/controller/recovery handle path generation and replanning. |
| Nav2 Waypoint Follower | https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html | Patrol routes are waypoint-driven; target missions pause patrol and resume the saved waypoint index. |
| Nav2 Collision Monitor | https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html | Final velocity must pass through an independent safety layer. In this project that role is `safety_cmd_mux_node`. |
| Nav2 Velocity Smoother | https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html | Candidate velocity commands should be rate-limited or smoothed before real wheels receive them. |
| Nav2 Map Saver | https://docs.nav2.org/configuration/packages/map_server/configuring-map-saver.html | Mapping workflows should persist YAML/PGM maps and explicitly separate save from apply/localization. |
| slam_toolbox Humble docs | https://docs.ros.org/en/humble/p/slam_toolbox/ | SLAM mode and localization/saved-map mode must be separated in UI and launch files. |
| RViz Marker display tutorial | https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html | Target, waypoint, cluster, yaw-alignment, and trial states should be visible as markers or operator-panel overlays. |
| RViz fixed frame behavior | https://docs.ros.org/en/ros2_packages/jazzy/api/rviz2/doc/index.html | Map visualization must use `map` as the fixed frame in Patrol Mode so only robot/path/target overlays move. |
| Gazebo Classic tutorials | https://classic.gazebosim.org/tutorials | Pre-real tests should launch the existing `ugv_world.world` and `ugv_rover` before hardware tests. |
| Gazebo SDF world/spawn model docs | https://gazebosim.org/docs/latest/sdf_worlds/ | The airport world and robot SDF should be treated as simulation fixtures, not recreated in a new project. |

## GitHub / Open Source References

| Repository | Link | Use In Waver |
|---|---|---|
| `ros2/rosbag2` | https://github.com/ros2/rosbag2 | Reference for topic recording/replay and bag artifact handling. Bags are not committed. |
| `SteveMacenski/slam_toolbox` | https://github.com/SteveMacenski/slam_toolbox | Reference SLAM/localization package for ROS 2 mapping workflows. Existing `ugv_slam` launch files remain first choice. |
| `jkk-research/lidar_cluster_ros2` | https://github.com/jkk-research/lidar_cluster_ros2 | Reference for ROS 2 Humble PointCloud2 clustering patterns. |
| `klintan/ros2_pcl_object_detection` | https://github.com/klintan/ros2_pcl_object_detection | Reference for Euclidean cluster extraction and pointcloud object topics. |
| `mgonzs13/yolo_ros` | https://github.com/mgonzs13/yolo_ros | Candidate future camera detector bridge. Current project keeps AI model integration as a stub. |
| Ultralytics ROS quickstart | https://docs.ultralytics.com/guides/ros-quickstart | Reference interface for future real image detector integration using ROS image topics. |
| `FoundationVision/ByteTrack` | https://github.com/FoundationVision/ByteTrack | Reference tracking-by-detection baseline for associating bird detections into tracklets. |

GitHub topic/trend check on 2026-05-26 favored ROS 2 Nav2, slam_toolbox, rosbag2, YOLO ROS wrappers, and PointCloud2 clustering packages. None of those replaces the active FSD_Vehicle structure; they only guide interfaces and safety defaults.

## Papers / Literature Direction

| Topic | Source / Search Result | Design Consequence |
|---|---|---|
| SLAM Toolbox | Macenski & Jambrecic, "SLAM Toolbox: SLAM for the dynamic world", JOSS 2021, referenced in slam_toolbox docs | Supports saved-map localization and mapping/localization separation. |
| Unsupervised LiDAR object detection | "Towards Unsupervised Object Detection From LiDAR Point Clouds", arXiv:2311.02007 | Temporal consistency and clustering are useful, but Waver only needs conservative cluster centroids before real learning models. |
| Ego-motion compensated dynamic detection | Dynamic-object detection literature consistently separates sensor-frame apparent motion from world-frame motion | Waver must classify static objects during yaw rotation as `static_due_to_ego_motion` instead of target. |
| LiDAR-camera fusion surveys | Recent fusion work emphasizes calibrated frames, timestamp alignment, and explicit 3D measurements | Waver height filtering requires real z from PointCloud2/depth/custom 3D messages; 2D LaserScan is insufficient. |
| ByteTrack | "ByteTrack: Multi-Object Tracking by Associating Every Detection Box", arXiv:2110.06864 | Bird tracking should be evaluated separately from detection using track continuity, ID switches, and trigger latency. |

## Bird Detection Autonomy Update

The project objective is bird-detection autonomous patrol, not SLAM-only
coverage improvement. SLAM/map metrics are support gates. Bird-system claims
must be based on bird precision, recall, F1, mAP, false positive/negative,
tracking continuity, z-valid localization, and mission-trigger correctness.

Current `jo` implements a Gazebo synthetic ground-truth bridge
(`bird_detection_pipeline_node`) to validate the ROS2 topic/mission/safety
pipeline before a real detector is installed. This bridge publishes
`/bird/detections_2d`, `/bird/detections_3d`, `/bird/tracks`,
`/bird/metrics`, and `/bird/mission_target`. It is intentionally labeled
`gazebo_model_state_synthetic` and must not be described as real-camera YOLO
performance.

Future real-detector integration should use an image model such as YOLO/RT-DETR
with documented license, class list, threshold/NMS settings, inference
hardware, and labeled bird dataset. ByteTrack/SORT-style tracking can then be
used for track continuity and ID-switch metrics. Gazebo 10-run results are
simulation-based pipeline evidence only.

## Applied Design Principles

1. Use `map` as the fixed UI/RViz frame in Patrol Mode; use live `/map` updates only in Mapping Mode.
2. Convert LiDAR/base/camera candidate coordinates into `map` or `odom` before motion classification.
3. Treat `target_min_height_m=3.0` as a height threshold, not a travel distance.
4. Reject `z_valid=false` targets; 2D LaserScan can support obstacle stopping but not elevated-object classification.
5. Keep Nav2 output remapped to `/waver/cmd_vel_nav2`; final `/cmd_vel` belongs only to `safety_cmd_mux_node`.
6. Keep `enable_sound_output=false`, `start_serial_bridge=false`, and test publishers off for real preflight.
7. Use rosbag2 plus CSV summaries; do not commit bag files or `experiments_result*` outputs.

## Gap Against Current Workspace

- Active tree does not contain `pcd_cluster_pkg`; the archive copy has it under `~/ros2_ws/FSD_Vehicle`. Active real-robot path is therefore `waver_patrol/perception/pointcloud_lidar_objects_node.py`.
- Gazebo validation can prove the height/dynamic logic with PoseArray data, but real deployment still needs a live 3D PointCloud2/depth/custom z source.
- H4/H5/H6 ego-rotation and z-unknown trials are recommended stress tests; H1/H2/H3 are the current hard gate.
- The latest 10-run Gazebo/UI mapping validation is simulation-based evidence. It supports pre-deployment readiness, but wheel-on PASS still requires real_vehicle_precheck, rosbag replay, wheel-off HIL, hardware E-STOP, and supervised low-speed closed-area trials.

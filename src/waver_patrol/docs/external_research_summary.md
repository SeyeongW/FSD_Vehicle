# External Research Summary

Date checked: 2026-05-25
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
| slam_toolbox Humble docs | https://docs.ros.org/en/humble/p/slam_toolbox/ | SLAM mode and localization/saved-map mode must be separated in UI and launch files. |
| RViz Marker display tutorial | https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html | Target, waypoint, cluster, yaw-alignment, and trial states should be visible as markers or operator-panel overlays. |
| Gazebo Classic tutorials | https://classic.gazebosim.org/tutorials | Pre-real tests should launch the existing `ugv_world.world` and `ugv_rover` before hardware tests. |

Additional 2026-05-25 re-check:

- ROS 2 Humble tf2 documentation confirms each node keeps a buffered frame tree and tooling such as `tf2_echo`/`view_frames` is the expected debugging path. This supports the Waver rule that raw LiDAR-frame apparent motion is not a target trigger.
- ROS 2 Humble rosbag2 documentation describes recording selected topic sets for replay and examination, matching the `experiments_result` CSV/plot/report plus optional rosbag workflow.
- Nav2 package documentation for Waypoint Follower, Collision Monitor, and Velocity Smoother supports keeping patrol in Nav2 while final robot velocity is guarded by a safety layer.
- slam_toolbox/SLAM references reinforce separating live mapping from fixed-map localization. The current code keeps `backend:=gazebo_live` simulation-only and uses `cartographer`/`gmapping` launch wrappers for real mapping.

## GitHub / Open Source References

| Repository | Link | Use In Waver |
|---|---|---|
| `ros2/rosbag2` | https://github.com/ros2/rosbag2 | Reference for topic recording/replay and bag artifact handling. Bags are not committed. |
| `SteveMacenski/slam_toolbox` | https://github.com/SteveMacenski/slam_toolbox | Reference SLAM/localization package for ROS 2 mapping workflows. Existing `ugv_slam` launch files remain first choice. |
| `jkk-research/lidar_cluster_ros2` | https://github.com/jkk-research/lidar_cluster_ros2 | Reference for ROS 2 Humble PointCloud2 clustering patterns. |
| `klintan/ros2_pcl_object_detection` | https://github.com/klintan/ros2_pcl_object_detection | Reference for Euclidean cluster extraction and pointcloud object topics. |
| `mgonzs13/yolo_ros` | https://github.com/mgonzs13/yolo_ros | Candidate future camera detector bridge. Current project keeps AI model integration as a stub. |

GitHub topic/trend check on 2026-05-24 favored ROS 2 Nav2, slam_toolbox, rosbag2, YOLO ROS wrappers, and PointCloud2 clustering packages. None of those replaces the active FSD_Vehicle structure; they only guide interfaces and safety defaults.

## Papers / Literature Direction

| Topic | Source / Search Result | Design Consequence |
|---|---|---|
| SLAM Toolbox | Macenski & Jambrecic, "SLAM Toolbox: SLAM for the dynamic world", JOSS 2021, referenced in slam_toolbox docs | Supports saved-map localization and mapping/localization separation. |
| Unsupervised LiDAR object detection | "Towards Unsupervised Object Detection From LiDAR Point Clouds", arXiv:2311.02007 | Temporal consistency and clustering are useful, but Waver only needs conservative cluster centroids before real learning models. |
| Ego-motion compensated dynamic detection | Dynamic-object detection literature consistently separates sensor-frame apparent motion from world-frame motion | Waver must classify static objects during yaw rotation as `static_due_to_ego_motion` instead of target. |
| LiDAR-camera fusion surveys | Recent fusion work emphasizes calibrated frames, timestamp alignment, and explicit 3D measurements | Waver height filtering requires real z from PointCloud2/depth/custom 3D messages; 2D LaserScan is insufficient. |

## Applied Design Principles

1. Use `map` as the fixed UI/RViz frame in Patrol Mode; use live `/map` updates only in Mapping Mode.
2. Convert LiDAR/base/camera candidate coordinates into `map` or `odom` before motion classification.
3. Treat `target_min_height_m=3.0` as a height threshold, not a travel distance.
4. Reject `z_valid=false` targets; 2D LaserScan can support obstacle stopping but not elevated-object classification.
5. Keep Nav2 output remapped to `/waver/cmd_vel_nav2`; final `/cmd_vel` belongs only to `safety_cmd_mux_node`.
6. Keep `enable_sound_output=false`, `start_serial_bridge=false`, `enable_deep_learning_stub=false`, and test publishers off for real preflight.
7. Use rosbag2 plus CSV summaries; do not commit bag files or `experiments_result*` outputs.
8. Keep UI WASD as `/waver/manual_cmd_vel` only; direct `/cmd_vel` publishing is blocked in `profile:=real`.

## Gap Against Current Workspace

- Active tree uses `waver_patrol/perception/pointcloud_lidar_objects_node.py` as the real PointCloud2 object-candidate path. If archive `pcd_cluster_pkg` is later restored, it must remain perception-only and publish `/waver/lidar_objects` without `/cmd_vel`.
- Gazebo validation can prove the height/dynamic logic with PoseArray data, but real deployment still needs a live 3D PointCloud2/depth/custom z source.
- H4/H5/H6 ego-rotation and z-unknown trials are recommended stress tests; H1/H2/H3 are the current hard gate.

# Current Waver Algorithm Analysis

| Name | Purpose | Input topic | Output topic | Parameter | Formula / Rule | Success criteria | Failure condition | Gazebo validation | Real-robot limitation |
|---|---|---|---|---|---|---|---|---|---|
| SLAM mapping | Build 2D map | `/scan`, `/odom`, `/tf` | `/map`, `map->odom` | gmapping/cartographer configs | SLAM backend dependent | map appears and saves | no scan/TF/map | run `ugv_slam` wrappers | outdoor drift and sensor noise |
| Localization | Pose on saved map | `/map`, `/scan`, `/odom` | `/amcl_pose`, TF | AMCL/localization params | particle filter / odom fallback | stable pose covariance | stale TF/pose | simulated odom fallback | needs real map alignment |
| Waypoint patrol | Patrol route | waypoint yaml, odom/Nav2 | active goal, mission state | `waypoint_file`, dwell, loop | sequential state machine | index loops and resumes | no waypoint/goal timeout | Gazebo waypoint loop | real Nav2 needs tuned costmaps |
| Safety mux | Single final command | auto/manual cmd, `/scan`, stop topics | `/cmd_vel`, safety state | speed, stale, scan thresholds | priority and clamp/rate limit | one `/cmd_vel` publisher | duplicate publisher/stale scan | `ros2 topic info -v /cmd_vel` | live scan required on robot |
| Cluster candidate | Object center interface | PointCloud or fake PoseArray | `/waver/lidar_objects` | cluster topic/frame params | centroid candidate | candidate published | no PointCloud2/z source | fake Gazebo publisher and `pointcloud_lidar_objects_node` | active `pcd_cluster_pkg` absent; Waver PointCloud2 adapter is the current real path |
| Map transform | Common coordinate | `/waver/lidar_objects` | `/waver/lidar_objects_map` | target/fallback frame, timeout | `p_map=T_map_lidar*p_lidar` | output in map/odom | TF missing | map-frame fake and TF fallback | source frame must be correct |
| elevated dynamic filter | Reject low/static/ego-motion artifacts | `/waver/lidar_objects_map`, `/waver/lidar_objects`, `/odom` | `/waver/elevated_dynamic_targets`, `/waver/moving_target_valid`, debug topics | `target_min_height_m`, `min_dynamic_motion_m`, duration, yaw-rate guard | height >= 3 m and map/odom compensated motion | valid only when z is real, high enough, and dynamic | low height, static, z missing, ego-motion artifact | H1/H2/H3 height trials | needs real 3D z source on robot |
| Target goal | Approach moving object | object goal sources | `/waver/object_mission_goal` | offset, min/max range | stop before target by offset | target goal sent | invalid target | Gazebo mission goal | real obstacle avoidance needs Nav2 |
| Yaw alignment | Face target | robot pose and object pose | cmd candidate/state | `kp_yaw`, max angular | `yaw_error=atan2(dy,dx)-yaw` | yaw error below tolerance | timeout/target lost | CSV yaw_alignment | camera frame calibration needed |
| Camera detection | Confirm class | external/camera stub | class/confidence/bird bool | confidence threshold | class-level decision | fake/real detection true | low confidence/unknown | fake camera publisher | real model not included |
| Sound mission | Non-contact warning stub | sound request, class | sound state/done | duration/cooldown/output false | dry-run state machine | sound done topic | timeout | CSV sound_mission | legal/safety approval required |
| Patrol resume | Return to route | mission result | mission state/current waypoint | resume policy | saved waypoint index | patrol navigating again | resume timeout | trial summary flag | real localization must be stable |

## False Moving Target Guard

The elevated dynamic filter does not use raw sensor-frame displacement for target decisions. It first consumes map/odom-transformed object coordinates, then checks object height and compensated motion. If a static elevated object appears to move only in the raw frame while Waver rotates, it is classified as `static_due_to_ego_motion` or `unknown_or_static` and does not trigger a mission.

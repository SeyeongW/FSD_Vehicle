# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Coding Guidelines

### 1. Think Before Coding

Before implementing, state assumptions explicitly. If multiple interpretations exist, present them — don't pick silently. If a simpler approach exists, say so. If something is unclear, stop and ask.

### 2. Simplicity First

Minimum code that solves the problem. No features beyond what was asked, no abstractions for single-use code, no "flexibility" that wasn't requested, no error handling for impossible scenarios. If it could be 50 lines, don't write 200.

### 3. Surgical Changes

Touch only what you must. Don't improve adjacent code, comments, or formatting. Match existing style. If unrelated dead code is noticed, mention it — don't delete it. Remove only the imports/variables/functions that *your* changes made unused.

### 4. Goal-Driven Execution

Transform tasks into verifiable goals. For multi-step tasks, state a brief plan with a verify step for each. Clarifying questions come before implementation, not after mistakes.

---

## Project Overview

**FSD_Vehicle** is a ROS2 Humble workspace for UGV (Unmanned Ground Vehicle) autonomous driving research. Supports Gazebo simulation and physical deployment on x86_64 PC and ARM64 Jetson. Current focus: LiDAR-based aerial bird detection and tracking integrated with Gazebo.

## Build Commands

```bash
# First-time full build (clones external SDKs, builds all packages)
bash build_first.sh

# Rebuild specific packages after code changes
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_gazebo
source install/setup.bash

# Rebuild external/dependency packages (ugv_else layer)
colcon build --packages-select <pkg> --cmake-args -DHUMBLE_ROS=humble
```

Always run `source install/setup.bash` after any build. Do not use `--symlink-install`.

## Docker Environment

```bash
# Linux/WSL
make build_pc   # build image
make run_pc     # start and attach to container

# Windows
build_pc / run_pc

# Jetson deployment
bash docker/run.sh build-jetson
bash docker/run.sh jetson
bash docker/run.sh stop
```

The `.env` file at the project root controls `UGV_MODEL` (ugv_rover / ugv_beast / rasp_rover), serial ports, and `ROS_DOMAIN_ID`.

## Running the Simulation

```bash
# Set model (required before any launch)
export UGV_MODEL=ugv_rover

# Terminal 1: Main Gazebo simulation (spawns UGV + bird managers)
ros2 launch ugv_gazebo bringup.launch.py

# Terminal 2: LiDAR PointCloud → LaserScan converter
ros2 run pcd_to_scan_pkg pointcloud_to_laserscan_node

# Teleop
ros2 run ugv_tools keyboard_ctrl
ros2 launch ugv_tools teleop_twist_joy.launch.py
```

## SLAM and Navigation (Gazebo)

```bash
# Cartographer (current workflow)
ros2 launch ugv_gazebo slam/cartographer.launch.py

# Save map (run while cartographer is active)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/seo/ros2_ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream'}"
# Or use the convenience script (adjust paths if inside Docker):
bash save_2d_cartographer_map_gazebo.sh

# Gmapping
ros2 launch ugv_gazebo slam/gmapping.launch.py
bash save_2d_gmapping_map_gazebo.sh

# Navigation2 (AMCL + TEB is the primary configuration)
ros2 launch ugv_gazebo nav/nav.launch.py

# SLAM + Nav simultaneously
ros2 launch ugv_gazebo slam_nav/slam_nav.launch.py
```

Nav2 parameter files: `src/ugv_main/ugv_nav/param/` and `src/ugv_main/ugv_gazebo/param/`, named `<localization>_<local_planner>.yaml` (e.g., `amcl_teb.yaml`).

## Physical Hardware

```bash
# LiDAR + chassis bringup (auto-selects LiDAR model from env)
ros2 launch ugv_bringup bringup_lidar.launch.py

# With IMU/EKF
ros2 launch ugv_bringup bringup_imu_ekf.launch.py

# Navigation on hardware
ros2 launch ugv_nav nav.launch.py use_localization:=amcl use_localplan:=teb
```

## Architecture

### Package Layers

**`src/ugv_main/`** — project-owned packages:
- `ugv_base_node` — differential drive kinematics and odometry
- `ugv_bringup` — hardware interface: motor control + sensor driver launch
- `ugv_description` — URDF/Xacro models (`ugv_rover.urdf`, `ugv_beast.urdf`, `rasp_rover.urdf`, `mid360.xacro`)
- `ugv_gazebo` — simulation: Gazebo worlds, models, launch files, manager scripts
- `ugv_nav` — Navigation2 integration (DWA/TEB planners, AMCL/EMCL/RTAB-Map localization)
- `ugv_slam` — SLAM launch configs (Gmapping, Cartographer, RTAB-Map)
- `ugv_vision` — AprilTag tracking, YOLO, camera launches
- `ugv_chat_ai` — LLM-based interaction via Ollama
- `ugv_tools` — teleop utilities
- `ugv_lidar_detection` — DBSCAN-based 3D object detection from PointCloud2 → `/lidar/detected_objects` (MarkerArray)
- `pcd_to_scan_pkg` — 3D PointCloud2 → LaserScan `/scan` + projected OccupancyGrid `/projected_map` (height filter + TF)
- `pcd_cluster_pkg` — DBSCAN clustering on `/mid360_PointCloud2` with ROI filter and auto-avoidance via `/cmd_vel`
- `plane_fit_pkg` — ground plane fitting

**`src/ugv_else/`** — vendored dependencies: `cartographer`, `teb_local_planner`, `vizanti`, `ldlidar`, `emcl2`, `apriltag_ros`, `rf2o_laser_odometry`, `costmap_converter`, `explore_lite`

**`src/livox_ros_driver2/`, `src/Livox-SDK2/`, `src/livox_laser_simulation_RO2/`** — Livox Mid-360 LiDAR driver and Gazebo simulation plugin

### Gazebo Simulation Architecture

`bringup.launch.py` orchestrates the full simulation:
1. Starts `gzserver` with `ugv_world.world`
2. Launches `robot_state_publisher` and spawns UGV at (0, 0)
3. After 8.5s: `bird_manager.py` — animates bird entities via `SetEntityState`/`GetEntityState`; publishes to `/bird_manager/bird_positions` and `/bird_manager/bird_detected`
4. After 10s: `ugv_manager.py` — UGV behavior manager

Scripts are in `src/ugv_main/ugv_gazebo/scripts/`. Models (bird, ugv_rover, ugv_beast, rasp_rover, world) in `src/ugv_main/ugv_gazebo/models/`.

### LiDAR Perception Pipeline

- Raw point cloud (hardware): `/mid360_PointCloud2`
- Raw point cloud (simulation): `/unilidar/cloud`
- `pcd_to_scan_pkg` → height-filtered `/scan` + `/projected_map`
- `pcd_cluster_pkg` → DBSCAN clusters in `odom` frame → `/cluster_markers`, `/filtered_points`
- `ugv_lidar_detection` → `/lidar/detected_objects` (MarkerArray)

## Key Environment Variables

| Variable | Values | Purpose |
|----------|--------|---------|
| `UGV_MODEL` | `ugv_rover`, `ugv_beast`, `rasp_rover` | Selects URDF and Gazebo model |
| `ROS_DOMAIN_ID` | `0` (default) | ROS2 DDS domain isolation |
| `SERIAL_PORT_PC` | `/dev/ttyUSB0` | Motor controller on PC |
| `SERIAL_PORT_JETSON` | `/dev/ttyTHS1` | Motor controller on Jetson |

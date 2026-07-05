# Local Operator Station

This document defines the local PC side of the Waver field stack.

## Boundary

The local PC may run:

- SSH/SCP/rsync clients
- RViz visualization
- `waver_remote_panel`
- ROS CLI tools for inspection
- release generation and transfer scripts
- Jetson Docker control scripts

The local PC must not run:

- Waver base drivers
- Livox or camera drivers
- Nav2, SLAM, robot localization, mission backend nodes for the real robot
- any final `/cmd_vel` publisher

The real field backend runs on the Jetson, inside Docker, and owns the robot
hardware path:

```text
Local PC UI/RViz
  -> SSH
  -> Jetson host
  -> Docker container fsd_dev_jetson
  -> ROS 2 backend nodes
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node
  -> Waver USB serial
```

## First-Time Local PC Setup

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_setup_local_pc.sh --check
bash scripts/waver_setup_local_pc.sh --install-minimal-ui
bash scripts/waver_setup_local_pc.sh --check --require-ros
bash scripts/waver_setup_local_pc.sh --install-rviz
```

Developer/test machines can install extra tooling:

```bash
bash scripts/waver_setup_local_pc.sh --install-dev
```

## Normal Field Operation

Start the Jetson Docker backend first:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_start_field_backend.sh
```

Then open the local operator station:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_operator_station_start.sh --rviz --ui
```

UI-only operation is still available:

```bash
bash scripts/waver_field_local_ui_start.sh --dry-run
bash scripts/waver_field_local_ui_start.sh
```

RViz-only operation is available:

```bash
bash scripts/waver_field_rviz_start.sh --dry-run --fixed-frame map --map-topic /map --scan-topic /scan_safety --pointcloud-topic /livox/lidar
bash scripts/waver_field_rviz_start.sh --fixed-frame map --map-topic /map --scan-topic /scan_safety --pointcloud-topic /livox/lidar
```

Combined dry-run examples:

```bash
bash scripts/waver_field_operator_station_start.sh --dry-run --rviz --ui
bash scripts/waver_field_operator_station_start.sh --rviz-only
bash scripts/waver_field_operator_station_start.sh --ui-only
```

## Safety Checks

The operator station checks Jetson SSH and the Docker container before opening
the UI/RViz by default. If this check fails, do not treat local UI values as
robot commands. Fix hotspot, IP address, SSH authentication, Docker, or backend
startup first.

For Gazebo-only or offline UI replay, explicitly opt out:

```bash
WAVER_SKIP_JETSON_CHECK=true bash scripts/waver_field_local_ui_start.sh
```

Do not use this bypass for real Waver movement.

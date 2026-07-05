# RViz Field Runbook

RViz is a local operator visualization tool only. It must not start robot
backend nodes, base drivers, Livox drivers, Nav2, SLAM, or any final
`/cmd_vel` publisher on the local PC.

## Default Field Topics

- fixed frame: `map`
- map: `/map`
- safety scan: `/scan_safety`
- Livox pointcloud: `/livox/lidar`

## Dry-Run

Dry-run must succeed without sourcing ROS, requiring a Jetson password, or
checking a live Jetson:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_rviz_start.sh --dry-run \
  --fixed-frame map \
  --map-topic /map \
  --scan-topic /scan_safety \
  --pointcloud-topic /livox/lidar
```

## Real Field Visualization

Start the Jetson Docker backend first, then launch RViz locally:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_start_field_backend.sh

bash scripts/waver_field_rviz_start.sh \
  --fixed-frame map \
  --map-topic /map \
  --scan-topic /scan_safety \
  --pointcloud-topic /livox/lidar
```

## Operator Station With UI

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_operator_station_start.sh --rviz --ui
```

The UI process must use the SSH bridge path, and manual control must publish
`/waver/manual_cmd_vel` only. Final `/cmd_vel` must remain owned by the Jetson
Docker backend command chain.

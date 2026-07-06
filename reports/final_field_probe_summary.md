# Waver Final Field Probe Summary

- generated_at: `2026-07-06T12:12:12+09:00`
- workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- branch: `jo`
- package_git_head: `6f07a15`
- run_mode: `field_probe_collection`
- hardware_connected: `false`
- probe_execution: `SKIPPED_BY_USER_CONTEXT_NO_HARDWARE_CONNECTED`

## Preserved Topology

Local PC -> SSH -> Jetson host -> docker exec -> Docker container `fsd_dev_jetson` -> ROS 2 nodes -> Waver USB serial -> Waver base.

## Current Honest Readiness

- `BIRD_PATROL_SOURCE_READY`: yes
- `BIRD_PATROL_FIELD_BRIDGE_READY`: not collected in this session
- `BIRD_PATROL_SENSOR_LIVE_READY`: not collected in this session
- `BIRD_PATROL_LIDAR_TRACKING_READY`: not collected in this session
- `BIRD_PATROL_DETECTOR_READY`: not collected in this session
- `BIRD_PATROL_FUSION_READY`: not collected in this session
- `BIRD_PATROL_INSPECTION_READY`: not collected in this session
- `BIRD_PATROL_AUTONOMOUS_READY`: no

Gazebo/UI simulation evidence remains simulation-only and must not promote real hardware readiness.

## Expected Field Environment

- JETSON_HOST: `10.139.225.150`
- JETSON_USER: `sw`
- JETSON_WS: `/home/sw/ros2_ws5/FSD_Vehicle`
- CONTAINER: `fsd_dev_jetson`
- ROS_DOMAIN_ID: `0`
- RMW_IMPLEMENTATION: `rmw_cyclonedds_cpp`
- serial_port: `auto`
- pointcloud_topic: `/livox/lidar`
- scan_topic: `/scan_safety`
- camera_image_topic: `/camera/image_raw`
- camera_info_topic: `/camera/camera_info`
- bird_model_path: `NOT_SET`
- camera_lidar_extrinsic_path: `config/sensors/camera_lidar_extrinsic.yaml`

## When Hardware Is Connected, Run In This Order

```bash
cd ~/ros2_ws5/FSD_Vehicle
export JETSON_HOST=10.139.225.150
export JETSON_USER=sw
export JETSON_WS=/home/sw/ros2_ws5/FSD_Vehicle
export CONTAINER=fsd_dev_jetson
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1. SSH and Docker bridge smoke
python3 scripts/waver_field_bridge_regression_check.py --remote-smoke --output reports/field_bridge_regression/latest.json

# 2. Start sensor-live backend, no movement/sound
bash scripts/waver_bird_patrol_field_start.sh --mode sensor-live

# 3. Livox live probe
python3 scripts/waver_livox_mid360_probe.py --pointcloud-topic /livox/lidar --scan-topic /scan_safety --duration-sec 15 --output reports/livox_mid360/latest.json

# 4. Camera live probe
python3 scripts/waver_camera_probe.py --image-topic /camera/image_raw --camera-info-topic /camera/camera_info --duration-sec 10 --output reports/camera/latest.json

# 5. Camera-LiDAR calibration check
python3 scripts/waver_camera_lidar_calibration_check.py --config config/sensors/camera_lidar_extrinsic.yaml --output reports/hardware_calibration/camera_lidar_latest.json

# 6. Bird detector probe, set real model path first
python3 scripts/waver_bird_detector_probe.py --model-path /models/bird_detector.pt --duration-sec 15 --output reports/bird_detector/latest.json

# 7. Wheel-off base feedback only, wheels physically off ground
python3 scripts/waver_base_feedback_probe.py --wheel-off-confirm --duration-sec 10 --output reports/base_feedback/latest.json

# 8. Command chain and readiness refresh
python3 scripts/waver_command_chain_check.py --output reports/command_chain/latest.json
python3 scripts/waver_bird_mission_readiness_check.py --mode sensor-live --strict --use-probe-reports --output reports/bird_mission_readiness/sensor_live_latest.json
python3 scripts/generate_bird_mission_readiness_audit.py
```

## Do Not Run Yet

- `inspection-dry-run` until wheel-off base feedback is PASS.
- `supervised-bird-patrol` until Livox, camera, calibration, detector, command chain, E-stop, and operator supervision are PASS.
- `autonomous-patrol` until full live evidence exists.
- real sound output unless all sound ACK environment variables are explicitly set.

## Expected Failure Classes

- SSH/Docker: Jetson IP changed, container not running, wrong workspace path.
- ROS network: ROS_DOMAIN_ID/RMW mismatch.
- Serial/base: no by-id serial path, multiple serial owners, stale base feedback, battery fault.
- Livox: sensor IP/host IP mismatch, no `/livox/lidar`, stale timestamp, low point count, missing TF.
- Camera: no image, missing camera_info frame, low FPS, wrong exposure.
- Fusion: uncalibrated extrinsic, camera-only target rejection, dynamic association failure.
- Safety: `/cmd_vel` publisher count not 1, `/waver/mode` publisher count not 1, external stop active.

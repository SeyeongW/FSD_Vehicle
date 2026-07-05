# Waver Bird Patrol Field Runbook

This is the field entrypoint for the Waver bird patrol product path. Bird mission functions are part of the current pipeline. Capabilities that are not ready must report `BLOCKED`, `DEGRADED`, or `FAIL_CLOSED`; they are not silently removed.

## Safety

Use this stack only in a supervised closed area. Real sound output is non-contact and non-injurious only, and is forbidden unless hardware, local-rule, and operator-supervision acknowledgments are present. Topic-only alignment and Gazebo/fake detection are not production bird evidence.

## Hardware Assumptions

- Waver base with one serial owner.
- MID-360 or equivalent 3D LiDAR publishing `/livox/lidar`.
- Camera publishing `/camera/image_raw` and `/camera/camera_info`.
- Calibrated camera-LiDAR extrinsic before fusion-live or higher.
- Optional gimbal or robot-body yaw alignment.
- Optional sound deterrent hardware, disabled by default.

## Build And Configure

Do not deploy the raw workspace zip. It is a backup artifact and may contain
`.git`, `build/`, `install/`, `log/`, `.pytest_cache/`, and runtime experiment
data. The deploy artifact is the clean field release tarball generated below.

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Set robot-local values in `config/waver_field_env.local` or `~/.waver_field_env`. Do not commit private field secrets.

## Field Procedure And Markers

1. Build bird mission release:
   `python3 scripts/make_bird_mission_field_release.py --output /tmp/waver_bird_mission_field_release.tar.gz`
   Success marker: `BIRD_MISSION_FIELD_RELEASE_WRITTEN=...`
2. Check release:
   `python3 scripts/check_bird_mission_field_release.py --path /tmp/waver_bird_mission_field_release.tar.gz --self-test`
   Success marker: `BIRD_MISSION_FIELD_RELEASE_CHECK=PASS`
3. On the local operator PC, check dependencies:
   `bash scripts/waver_setup_local_pc.sh --check`
   Optional RViz check:
   `bash scripts/waver_setup_local_pc.sh --check --require-ros`
4. Copy release to Jetson and configure local env:
   `bash scripts/waver_create_home_field_env.sh`
   Success marker: field env file exists and SSH probe passes.
5. Verify base driver wheel-off:
   `python3 scripts/waver_field_readiness_check.py --level L3 --strict --serial-port /dev/serial/by-id/<WAVER_SERIAL_ID>`
   Success marker: `FIELD_READINESS=PASS`.
6. Verify MID-360:
   `python3 scripts/waver_livox_mid360_probe.py --pointcloud-topic /livox/lidar --scan-topic /scan_safety --duration-sec 15 --output reports/livox_mid360/latest.json`
   Success marker: `LIDAR_READY`.
7. Verify camera:
   `python3 scripts/waver_camera_probe.py --image-topic /camera/image_raw --camera-info-topic /camera/camera_info --duration-sec 10 --output reports/camera/latest.json`
   Success marker: `CAMERA_READY`.
8. Calibrate camera-LiDAR:
   `python3 scripts/waver_camera_lidar_calibration_check.py --config config/sensors/camera_lidar_extrinsic.yaml --output reports/hardware_calibration/camera_lidar_latest.json`
   Success marker: `CALIBRATION_READY`.
9. Verify detector:
   `python3 scripts/waver_bird_detector_probe.py --model-path /models/bird_detector.pt --output reports/bird_detector/latest.json`
   Success marker: `DETECTOR_READY`.
10. Run `sensor-live`, `lidar-tracking`, `detector-live`, `fusion-live`, `inspection-dry-run`, `supervised-deterrence`, then `autonomous-patrol` in order with `waver_bird_mission_readiness_check.py --strict --use-probe-reports`.
   Success marker: `BIRD_MISSION_READINESS=PASS` for the selected mode.

## Checks

```bash
python3 scripts/waver_bird_mission_readiness_check.py --mode source --profile config/real_profiles/bird_patrol_production.yaml --strict --no-hardware
python3 scripts/waver_livox_mid360_probe.py --pointcloud-topic /livox/lidar --scan-topic /scan_safety --duration-sec 15 --output reports/livox_mid360/latest.json
python3 scripts/waver_camera_probe.py --image-topic /camera/image_raw --camera-info-topic /camera/camera_info --duration-sec 10 --output reports/camera/latest.json
python3 scripts/waver_camera_lidar_calibration_check.py --config config/sensors/camera_lidar_extrinsic.yaml --output reports/hardware_calibration/camera_lidar_latest.json
python3 scripts/waver_bird_detector_probe.py --model-path /models/bird_detector.pt --duration-sec 15 --output reports/bird_detector/latest.json
```

## Start Modes

If `--profile` is omitted, the entrypoint selects the staged profile matching
the selected mode:

- `sensor-live` -> `sensor_live.yaml`
- `lidar-tracking` -> `lidar_nav_backend.yaml`
- `detector-live`, `fusion-live`, `inspection-dry-run` -> `inspection_dry_run.yaml`
- `supervised-deterrence` or `supervised-bird-patrol` -> `supervised_bird_patrol.yaml`
- `autonomous-patrol` -> `autonomous_bird_patrol_locked.yaml`

```bash
bash scripts/waver_bird_patrol_field_start.sh --mode source
bash scripts/waver_bird_patrol_field_start.sh --mode sensor-live
bash scripts/waver_bird_patrol_field_start.sh --mode lidar-tracking
bash scripts/waver_bird_patrol_field_start.sh --mode detector-live --bird-model /models/bird_detector.pt
bash scripts/waver_bird_patrol_field_start.sh --mode fusion-live --bird-model /models/bird_detector.pt
bash scripts/waver_bird_patrol_field_start.sh --mode inspection-dry-run --speed-tier first-wheel-on --bird-model /models/bird_detector.pt
bash scripts/waver_bird_patrol_field_start.sh --mode supervised-bird-patrol --speed-tier supervised-low-speed --bird-model /models/bird_detector.pt
```

`inspection-dry-run` is a real first-wheel-on inspection movement dry-run, not a
monitoring-only mode. It stays capped at 0.05 m/s and 0.20 rad/s. The supervised
tier defaults to 0.08 m/s and 0.25 rad/s and requires operator, collision
monitor, and blackbox evidence before field use.

## Local Operator Station

The operator PC runs SSH, RViz, `waver_remote_panel`, and ROS CLI inspection
only. The Jetson Docker container owns Nav2, SLAM, Livox/camera/base drivers,
mission backend, safety mux, and final `/cmd_vel`.

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_operator_station_start.sh --dry-run --rviz --ui
bash scripts/waver_field_operator_station_start.sh --rviz --ui
```

RViz-only:

```bash
bash scripts/waver_field_rviz_start.sh --fixed-frame map --map-topic /map --scan-topic /scan_safety --pointcloud-topic /livox/lidar
```

UI-only:

```bash
bash scripts/waver_field_local_ui_start.sh
```

Real deterrence requires explicit acknowledgments:

```bash
WAVER_ACK_SOUND_HARDWARE=1 WAVER_ACK_LOCAL_SOUND_LAW=1 WAVER_ACK_OPERATOR_SUPERVISION=1 \
bash scripts/waver_bird_patrol_field_start.sh --mode supervised-deterrence --enable-sound-output --bird-model /models/bird_detector.pt
```

## Blackbox Logs

```bash
bash scripts/waver_blackbox_recorder.sh --profile bird_mission --output-dir reports/field_runs/manual_test
```

Heavy camera/LiDAR recording is opt-in with `--include-heavy-sensors`.

## Known Limitations

- `config/sensors/camera_lidar_extrinsic.yaml` ships as `calibrated: false`; fusion-live is blocked until field calibration.
- The default sound backend is `disabled`; supervised deterrence and autonomous deterrence are blocked until real hardware is verified.
- If collision monitor is not connected in the final command chain, autonomous-patrol readiness must fail even if safety mux is active.

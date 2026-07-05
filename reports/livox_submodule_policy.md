# Livox Mid-360 Submodule Policy

Generated: 2026-07-05

## Current State

- Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- Vendor path: `src/livox_ros_driver2`
- Vendor HEAD: `6b9356c`
- Dirty files:
  - `CMakeLists.txt`
  - `config/MID360_config.json`

## Observed Diff Summary

`CMakeLists.txt` contains local ROS 2 Humble build compatibility changes:

- default `HUMBLE_ROS` from `ROS_DISTRO=humble`
- set `LIVOX_INTERFACE_TARGET`
- set `LIVOX_INTERFACES_INCLUDE_DIRECTORIES`

`config/MID360_config.json` contains local field network values:

- host IP changed from `192.168.1.5` to `192.168.1.50`
- lidar IP changed from `192.168.1.12` to `192.168.1.102`

## Policy Decision

Use policy B:

1. Treat `src/livox_ros_driver2` as vendor/submodule code.
2. Do not silently include local field IP edits as source truth.
3. Keep reusable Livox settings in:
   - `config/sensors/livox_mid360_field.example.yaml`
   - ignored local override: `config/sensors/livox_mid360_field.local.yaml`
4. Runtime launch/backend scripts may generate or patch a Docker/runtime copy of `MID360_config.json`.
5. Source release should document this dirty vendor state and avoid claiming the vendor tree is clean until the policy is resolved.

## Required Follow-Up

- Move durable Humble build fixes into a deliberate vendor patch, fork, or documented overlay.
- Keep field IP addresses out of tracked vendor `MID360_config.json`.
- Validate actual topics on Jetson:
  - `/livox/lidar`
  - `/livox/imu`
  - `/scan_safety`
- Run:

```bash
python3 scripts/waver_livox_mid360_probe.py \
  --pointcloud-topic /livox/lidar \
  --scan-topic /scan_safety \
  --duration-sec 15 \
  --output reports/livox_mid360/latest.json
```

## Current Readiness Impact

This does not block `BIRD_PATROL_SOURCE_READY`, but it blocks any honest claim of fully clean field release provenance until the vendor/submodule policy is finalized.

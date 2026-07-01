# Field Docker SSH Runbook

This is the preserved field workflow. It must not be replaced casually because it is how the local PC drives the Jetson Docker backend and the Waver USB serial path.

## Physical Setup

1. Turn on hotspot.
2. Connect local PC and Jetson to the same hotspot.
3. Connect to Jetson by NoMachine if needed.
4. Power Waver.
5. Connect Waver USB serial to Jetson only after power is stable.
6. Confirm Jetson SSH:

```bash
ssh sw@10.139.225.150
```

## Local PC Backend Terminal

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_docker_backend_start.sh
```

This script:

- loads `config/waver_field_env` and optional local overrides.
- checks or bootstraps the Jetson workspace.
- starts/uses Docker container `fsd_dev_jetson`.
- starts mission manager, safety mux, Waver base driver, and the supervised field patrol helper.
- verifies `/cmd_vel` chain and base driver status.

## Local PC UI Terminal

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_local_ui_start.sh
```

This script:

- opens the local remote panel.
- bridges manual/mode/mission commands to the Jetson Docker ROS graph over SSH.
- keeps local UI away from direct final `/cmd_vel` publishing.

## Change Jetson IP

Edit:

```bash
config/waver_field_env
```

or create ignored local override:

```bash
config/waver_field_env.local
```

Recommended override:

```bash
JETSON_HOST=<current_jetson_ip>
JETSON_USER=sw
JETSON_WS=/home/sw/ros2_ws5/FSD_Vehicle
CONTAINER=fsd_dev_jetson
ROS_DOMAIN_ID=0
```

## No-Motion Validation

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_docker_ssh_check.sh \
  --local-compose \
  --jetson-compose \
  --no-motion \
  --cycles 2
```

Live Jetson SSH status is skipped unless:

```bash
export WAVER_ENABLE_SSH_VALIDATION=1
```

## Safety Notes

- Backend startup may open the Waver serial driver. Use only when the robot is physically safe.
- Validation scripts added for v8 do not publish motion commands.
- Wheel-on requires strict preflight and wheel-off direction checks.

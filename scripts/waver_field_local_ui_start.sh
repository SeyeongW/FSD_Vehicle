#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
unset RMW_IMPLEMENTATION

JETSON_HOST="${JETSON_HOST:-10.63.240.150}"
JETSON_USER="${JETSON_USER:-sw}"
JETSON_PASS="${JETSON_PASS:-12341234}"
JETSON_WS="${JETSON_WS:-/home/sw/ros2_ws2/FSD_Vehicle}"

echo "[LOCAL] starting remote panel bridge to ${JETSON_USER}@${JETSON_HOST} ws=${JETSON_WS}"

exec ros2 run ugv_tools waver_remote_panel --ros-args \
  --params-file src/ugv_main/ugv_tools/config/waver_4wd_control.yaml \
  -p publish_direct_cmd_vel:=false \
  -p profile:=operator_bridge \
  -p remote_bridge_enabled:=true \
  -p remote_bridge_host:="${JETSON_HOST}" \
  -p remote_bridge_user:="${JETSON_USER}" \
  -p remote_bridge_password:="${JETSON_PASS}" \
  -p remote_bridge_workspace:="${JETSON_WS}" \
  -p remote_bridge_ros_domain_id:=30 \
  -p remote_bridge_rmw:=rmw_cyclonedds_cpp \
  -p remote_bridge_command_timeout_s:=0.30 \
  -p lidar_required:=false \
  -p enable_scan_assist:=false \
  -p allow_start_without_map:=true \
  -p allow_start_without_localization:=true \
  -p publish_mode_heartbeat:=false \
  -p manual_override_returns_to_auto:=false \
  -p command_rate_hz:=60.0 \
  -p default_speed:=0.06 \
  -p default_angular:=0.08 \
  -p max_linear_speed:=0.08 \
  -p max_angular_speed:=0.08 \
  -p max_linear_accel:=1.20 \
  -p max_angular_accel:=1.20

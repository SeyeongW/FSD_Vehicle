# Rosbag Regression Guide

Use rosbags to turn real field observations into hardware-free regression
checks. Never replay a bag while real serial or sound output is enabled.

## Recommended Topics

```text
/cmd_vel
/waver/manual_cmd_vel
/waver/cmd_vel_nav2_raw
/waver/cmd_vel_nav2_smooth
/waver/cmd_vel_target_track
/waver/safety_state
/waver/mode
/waver/mission_state
/waver/mission_event
/odom
/tf
/tf_static
/scan
/scan_safety
/mid360_PointCloud2
/camera/image_raw
/camera/camera_info
/waver/bird_confirmed
/waver/bird_target_valid
/waver/target_class
/waver/target_confidence
/voltage
```

## Record

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag record -o ~/waver_bags/<test_id> \
  /cmd_vel /waver/manual_cmd_vel /waver/safety_state /waver/mode \
  /waver/mission_state /odom /tf /tf_static /scan /scan_safety /voltage
```

## Metadata Check

```bash
bash scripts/waver_rosbag_replay_check.sh --bag ~/waver_bags/<test_id>
```

## Replay Boundary

Do not replay into an active real robot graph. Use a separate terminal with:

```bash
export WAVER_ALLOW_HARDWARE=0
export WAVER_NO_HARDWARE=1
export WAVER_BLOCK_SERIAL=1
export WAVER_DISABLE_SOUND_OUTPUT=1
```

Replay should target offline analysis nodes or tests only. If a field bag shows
a bug, add a focused test that asserts the corrected behavior without opening
serial or sound hardware.

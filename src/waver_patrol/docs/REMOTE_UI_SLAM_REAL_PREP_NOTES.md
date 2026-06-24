# Remote UI SLAM Real Prep Notes

This note is for porting the `ros2_ws5` UI-SLAM workflow to the real Waver
backend later. It does not change the current field-success backend.

## Do Not Mix Profiles

- Gazebo `/scan` alias nodes are simulation-only.
- Gazebo live/fake map publishers are simulation-only.
- Real profile must keep `start_serial_bridge` and serial owner gates intact.

## Real Topic Contract

- `/waver/manual_cmd_vel`: UI or keyboard candidate only.
- `/waver/cmd_vel_nav2`: Nav2 or patrol candidate only.
- `/cmd_vel`: final command from `safety_cmd_mux_node` only.
- `/waver/mode`: state from `mission_patrol_manager_node` only.
- `/scan_slam`: dense scan suitable for SLAM.
- `/scan_safety`: scan suitable for safety stopping.
- `/odom`: filtered odometry or base driver odometry.

## First Wheel-On Limits

- Linear speed: `<= 0.05 m/s`
- Angular speed: `<= 0.20 rad/s`
- Physical E-stop in hand.
- Wheels and serial direction verified wheel-off first.

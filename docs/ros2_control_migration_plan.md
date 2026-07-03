# ros2_control Migration Plan

## Current Structure

This release keeps `waver_base_driver_node.py` as the custom single-owner serial
driver. It reads Waver feedback packets, publishes odometry/IMU/voltage/state,
and writes final safety-filtered commands to the Waver serial protocol.

## Why It Remains Accepted For This Release

- It matches the existing Waver JSON serial protocol.
- It supports stop burst on startup/shutdown.
- It enforces one serial owner.
- It has command timeout behavior.
- It is practical for L3 wheel-off and L4 low-speed bringup.

## Limitations

- No `controller_manager` lifecycle.
- No standard `hardware_interface::SystemInterface`.
- No standard `diff_drive_controller` state/command interface.
- Hardware calibration and feedback schema must be validated by Waver-specific
  scripts before wheel-on operation.

## Target Architecture

```text
waver_base_hardware_interface
  read(): serial feedback -> wheel state, IMU, battery
  write(): wheel command -> Waver serial JSON
  exported interfaces -> diff_drive_controller or custom controller
```

## Acceptance Tests Required Before Migration

- single serial owner
- command timeout stop
- stop burst on startup and shutdown
- odom rate and covariance sanity
- IMU rate sanity
- battery stale and low-voltage detection
- final `/cmd_vel` owner topology
- wheel direction and scale calibration

Until that migration is complete, the custom Python driver is accepted only for
guarded L3/L4 bringup and must remain behind readiness checks.

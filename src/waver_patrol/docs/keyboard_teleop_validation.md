# Keyboard Teleop Validation

Goal: verify that keyboard/manual commands behave like a game-style control path in simulation/mock first, then wheel-off in the field.

## Contract

```text
waver_remote_panel or keyboard helper
  -> /waver/manual_cmd_vel
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> selected base driver
```

The keyboard/UI must not publish final `/cmd_vel` directly in real profiles.

## Gazebo/Mock Validation

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh \
  --scenario keyboard_teleop_smoke \
  --cycles 2
```

Expected evidence:

- launch or Gazebo failure is captured under `reports/gazebo_functional_validation/...`
- if graph is visible, final `/cmd_vel` owner remains `safety_cmd_mux_node`
- missing Gazebo/ROS environment is recorded as `SKIP_WITH_REASON`

## Field Wheel-Off Validation

Only after strict preflight:

```bash
bash src/waver_patrol/scripts/waver_cmd_chain_check.sh
bash src/waver_patrol/scripts/waver_indoor_patrol_status.sh --strict
```

Then use the remote UI with the robot safely lifted:

- `W`: all drive wheels forward as expected.
- `S`: all drive wheels reverse as expected.
- `A`: turn direction matches operator expectation.
- `D`: turn direction matches operator expectation.
- key release sends zero promptly.
- E-stop forces zero.

## Do Not Claim PASS If

- UI-only values change but Jetson `/waver/manual_cmd_vel` does not.
- `/cmd_vel` is absent or has multiple publishers.
- final `/cmd_vel` is not from `safety_cmd_mux_node`.
- wheel directions are unverified.

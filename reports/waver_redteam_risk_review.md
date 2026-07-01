# Waver Red-Team Risk Review

Date: 2026-06-30
Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
Branch: `jo`
Prompt: `waver_codex_final_v8`

## Safety-Critical Risks

1. Final command authority drift
   - Risk: Nav2, teleop, a bridge, or a test node publishes final `/cmd_vel` directly.
   - Required guard: field and indoor real profiles must keep `safety_cmd_mux_node` as the only `/cmd_vel` publisher.
   - Validation: `waver_cmd_chain_check.sh`, launch contract tests, runtime topic info before wheel-off.

2. Scan clear state masking hazards
   - Risk: `OK_CLEAR` from Livox/scan adapter bypasses stale scan, low finite point count, or hard stop checks.
   - Required guard: `OK_CLEAR` means no obstacle after scan validity is checked; it must not short-circuit safety logic.
   - Validation: regression tests for stale scan, low finite points, front/rear hard stop, slow zone, and adapter degraded states.

3. Real profile accidentally starts sim/test publishers
   - Risk: fake odom, Gazebo publishers, fake camera, test sound, or demo obstacles affect real robot behavior.
   - Required guard: real/indoor launch defaults must disable fake/test/Gazebo publishers.
   - Validation: launch static contracts and preflight forbidden-node checks.

4. Docker/SSH field workflow hidden coupling
   - Risk: new users clone repo and run scripts without local env, wrong Jetson IP, wrong Docker container, or missing SSH helper.
   - Required guard: env loader with defaults, dry-run checks, clear runbook, no-motion status script.
   - Validation: `waver_field_docker_ssh_check.sh` in no-motion mode.

5. SLAM/Nav validation overclaims
   - Risk: declaring SLAM/localization/nav ready without Gazebo or hardware evidence.
   - Required guard: Gazebo validation scripts must record PASS/FAIL/SKIP_WITH_REASON and never silently pass.
   - Validation: isolated ROS domain, timeout-bound Gazebo/sim scripts, evidence files under `reports/`.

6. Real motor command during automated validation
   - Risk: an automated check publishes motion to a connected vehicle.
   - Required guard: no real serial open or motion publish from validation scripts unless explicit operator field script is launched.
   - Validation: scripts use mock/sim or no-motion status checks; final report states no motor command was executed.

## First-Principles Checks

- A mobile robot is safe only if command authority, sensing validity, and stop authority are deterministic.
- A green UI state is not enough; final ROS graph ownership must match the safety contract.
- Simulation evidence is useful only when isolation prevents accidental hardware actuation.
- Real readiness requires conservative defaults, explicit preflight failure, and honest skip reporting.

## Steelman Counterarguments

- Using open-loop field patrol is useful for early wheel-off/wheel-on tests when odom is missing.
  - Response: keep it explicitly labeled as supervised fallback, speed-limited, and not equivalent to localization-based autonomy.
- Collision Monitor could be a stronger final safety layer.
  - Response: not in this pass; introducing it as final `/cmd_vel` owner changes the command-chain contract and needs a separate integration plan.
- Real bird detector/fusion should stay enabled for final project behavior.
  - Response: full real bird autonomy can exist, but the indoor low-speed readiness profile must default to the minimal safe patrol stack.

## Deferred Risks

- Actual Livox Mid-360 frame orientation and scan density are hardware-dependent.
- Camera-LiDAR extrinsic calibration cannot be proven from local static checks.
- AMCL/map quality and waypoint safety require a real map or Gazebo run.
- Wheel direction, braking, and E-stop require wheel-off physical checks.

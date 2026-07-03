# Waver Field Hardware Readiness Audit

Final judgement: **FIELD_READY_DRY_RUN**

Generated: 2026-07-04 01:23 KST

## Scope

This audit covers local source/static validation for
`~/ros2_ws5/FSD_Vehicle` on branch `jo`. It does not claim live LiDAR, live
serial motor feedback, wheel-off, wheel-on, or autonomous patrol readiness.

## Validation Results

| Check | Result |
|---|---|
| Python compileall for scripts, launch, and Waver Python modules | PASS |
| Shell syntax for `scripts/*.sh` and `src/waver_patrol/scripts/*.sh` | PASS |
| `bash scripts/run_no_ros_unit_tests.sh` | PASS, 85 tests |
| `cd src/waver_patrol && PYTHONPATH=. python3 -m pytest -q test` | PASS, 105 passed, 10 skipped |
| `python3 scripts/waver_contract_check.py --require-git-branch jo` | PASS |
| `python3 scripts/make_field_release.py --output /tmp/waver_field_release.tar.gz` | PASS |
| `python3 scripts/check_field_release.py --path /tmp/waver_field_release.tar.gz` | PASS |
| `python3 scripts/waver_field_readiness_check.py --level L0 --strict --no-hardware` | PASS |
| `python3 scripts/waver_field_readiness_check.py --level L1 --strict --no-hardware` | PASS |
| `python3 scripts/waver_field_readiness_check.py --level L2 --strict --no-hardware` | FAIL as expected; L2 requires live sensor evidence |

## Why This Is Not L2 Or Higher

L2 requires live sensor evidence. L3 requires wheel-off serial/base feedback.
L4 requires closed-area low-speed wheel-on evidence, battery evidence, scan,
odom, TF, safety state, and acceptance-matrix PASS entries. Those were not
collected in this local source-only run.

## Safety Policy Confirmed

- Final `/cmd_vel` authority is `safety_cmd_mux_node`.
- `nav2_collision_monitor` is documented as future optional, not claimed as
  implemented safety authority for this hardware release.
- Real bird launch defaults keep bird detector, 3D fusion, gimbal, sound
  deterrent, target approach, target departure, radar bridge, and experiment
  logger disabled.
- Sound output requires explicit hardware/legal acknowledgement.
- Readiness scripts no longer print unconditional `READY=YES`.
- L2+ cannot pass with `--no-hardware`.
- Field release archive excludes `.git`, `build`, `install`, `log`, private
  env files, rosbags, and logs.

## Remaining Hardware Evidence Required

- Mid360 `/livox/lidar` and `/livox/imu` live rate.
- Scan adapter output `/scan_safety` or `/scan` rate and frame.
- TF: `base_link -> livox_frame`, `odom -> base_link`, and in localization
  mode `map -> odom`.
- Waver serial by-id path, single serial owner, and base feedback packet schema.
- `/odom_raw`, `/imu/data_raw`, `/voltage`, `/waver/base_driver_state`,
  `/waver/serial_owner_state` freshness.
- Wheel direction, wheel scale, encoder direction, battery voltage scaling,
  E-stop physical check.
- Camera-LiDAR extrinsic and bird model validation before enabling bird
  experimental profile.

## Practical Readiness Assessment

The package is suitable for controlled L0/L1 dry-run and for preparing L2/L3
hardware tests. It is not yet proven wheel-on ready from source alone. The
mechanism is more complete and safer than previous local workspace copies
because strict field readiness, feedback schema, real profiles, source release
checks, blackbox/stop helpers, and fail-closed tests are now present.

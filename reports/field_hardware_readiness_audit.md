# Waver Field Hardware Readiness Audit

Final judgement: **FIELD_READY_DRY_RUN**

Generated: 2026-07-04 07:24 KST

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
| `PYTHONPATH=src/waver_patrol:src/ugv_main/ugv_tools python3 -m pytest -q src/waver_patrol/test` | PASS, 114 passed, 10 skipped |
| `python3 scripts/waver_contract_check.py --require-git-branch jo` | PASS |
| `python3 scripts/make_field_release.py --output /tmp/waver_field_release.tar.gz` | PASS |
| `python3 scripts/check_field_release.py --path /tmp/waver_field_release.tar.gz` | PASS; extracted release quickstart dry-run and clone-to-run acceptance PASS |
| default field release archive size | PASS, about 4.3 MB |
| `python3 scripts/waver_field_readiness_check.py --level L0 --strict --no-hardware --profile config/real_profiles/lidar_nav_backend.yaml` | PASS |
| `python3 scripts/waver_field_readiness_check.py --level L1 --strict --no-hardware --profile config/real_profiles/lidar_nav_backend.yaml` | PASS |
| `python3 scripts/waver_field_readiness_check.py --level L2 --strict --no-hardware --profile config/real_profiles/lidar_nav_backend.yaml` | FAIL as expected; L2 requires live sensor evidence |

## Field Release Fixes Confirmed

- `config/waver_field_env` is no longer required for clean release startup.
- `scripts/waver_quickstart_field.sh --dry-run` can create ignored
  `config/waver_field_env.local` from CLI values.
- `scripts/check_field_release.py` now extracts the archive and runs both
  quickstart dry-run and release-mode clone-to-run acceptance.
- Default backend entrypoint is now strict:
  `scripts/waver_start_field_backend.sh` -> `scripts/waver_field_lidar_nav_backend_start.sh`.
- Legacy supervised open-loop backend is gated by
  `WAVER_ALLOW_LEGACY_OPEN_LOOP_MICRO_PATROL=1` and cannot print
  `BACKEND_READY=YES`.
- Real profile YAML files now drive field backend defaults and readiness
  checker profile checks.

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
  env files, rosbags, logs, default Gazebo simulation assets, and vendor-heavy
  source trees unless explicitly included.
- Base driver launch passes `feedback_schema_path`, and base driver state now
  reports feedback schema/calibration, stop burst, timeout, odom rate, IMU rate,
  and voltage status fields.

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

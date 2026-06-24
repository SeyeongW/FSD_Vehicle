# Waver Quality Gates

This repository uses a hardware-free quality gate before any field test. The
gate is intentionally conservative: it may build, import launch files, run
static contract checks, and inspect rosbag metadata, but it must not open a real
serial port, publish motor commands to hardware, or start sound/GPIO outputs.

## Primary Gate

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_quality_gate.sh --no-hardware \
  --report-dir reports/quality_gate/latest
```

The gate sets these safety variables:

```bash
WAVER_ALLOW_HARDWARE=0
WAVER_NO_HARDWARE=1
WAVER_BLOCK_SERIAL=1
WAVER_DISABLE_SOUND_OUTPUT=1
```

## Required Pass Criteria

- Current git branch is `jo`.
- Python compile checks pass for Waver source packages.
- Static contract checker reports no CRITICAL or HIGH violations.
- Launch files import without executing hardware.
- Rosbag replay harness is present and either checks a provided bag or records
  an explicit optional skip.
- No command in the gate enables real serial, speaker, GPIO, or live motor
  output.
- `contract_report.json` contains `safety_regression: false`.
- Release archives are created only by `scripts/make_source_archive.py`, which
  excludes `.git`, `.env`, build outputs, logs, rosbags, and experiment outputs.
- ROS-dependent tests are marked `ros_required`; no-ROS tests can be run without
  importing `rclpy` or ROS message packages.

## Release Hygiene

Preview the source archive contents without writing a file:

```bash
python3 scripts/make_source_archive.py --root . --dry-run --list
```

Create a clean source archive:

```bash
python3 scripts/make_source_archive.py --root .
```

The archive is for source handoff only. It intentionally excludes generated
maps, bags, experiment outputs, logs, local `.env` files, and `.git`.

## Test Commands

Hardware-free pure Python tests:

```bash
bash scripts/run_no_ros_unit_tests.sh
```

Full ROS environment checks, when ROS 2 Humble is available:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  waver_patrol waver_experiment_logger waver_seo_tracking ugv_tools ugv_bringup
source install/setup.bash
colcon test --packages-select \
  waver_patrol waver_experiment_logger waver_seo_tracking ugv_tools ugv_bringup
colcon test-result --verbose
```

## Report Outputs

Each run writes a timestamped report directory under `reports/quality_gate/`.
The latest summary is also written to:

```text
reports/quality_gate/latest.md
reports/quality_gate/latest.json
```

Interpretation:

- `WAVER_CONTRACT_CHECK=PASS` and `safety_regression=false` mean the static
  safety contract found no CRITICAL/HIGH issue.
- INFO entries often describe local generated directories such as `build/` or
  `install/`; they are reminders, not field approval.
- A Gazebo PASS is evidence level L2 only. It does not prove real detector
  accuracy or outdoor deterrence performance.
- Hardware readiness still requires wheel-off and wheel-on checklists.

## Field Test Boundary

Hardware tests are not part of this automated gate. Wheel-off and wheel-on
tests must be run manually with the checklists in `reports/hardware_feedback/`
and the results should be converted into rosbags or written reports before
being added as regression inputs.

BIRD_PATROL_SOURCE_READY

# Bird Mission Readiness Audit

This audit separates source/static evidence, Gazebo simulation evidence, and real hardware evidence.
Gazebo/UI simulation PASS must never promote real autonomous readiness.
PASS_DEGRADED must never promote `BIRD_PATROL_AUTONOMOUS_READY`.

## Top-Level Judgment

- `BIRD_PATROL_SOURCE_READY`: YES
- live hardware evidence present: NO
- honest top-level judgment: `BIRD_PATROL_SOURCE_READY`

## Simulation Evidence

- `UI_SLAM_MAPPING_SIM_READY`: YES
- `SAVED_MAP_NAV2_SIM_READY`: YES
- `GAZEBO_BIRD_PATROL_MECHANISM_SIM_READY`: YES
- `UI_SLAM_AND_BIRD_DETECTION_SIM_READY`: YES

## Real Hardware Readiness

- `BIRD_PATROL_FIELD_BRIDGE_READY`: NO_LIVE_HARDWARE_EVIDENCE
- `BIRD_PATROL_SENSOR_LIVE_READY`: NOT_RUN
- `BIRD_PATROL_LIDAR_TRACKING_READY`: NOT_RUN
- `BIRD_PATROL_DETECTOR_READY`: NOT_RUN
- `BIRD_PATROL_FUSION_READY`: NOT_RUN
- `BIRD_PATROL_INSPECTION_READY`: NOT_RUN
- `BIRD_PATROL_SUPERVISED_DETERRENCE_READY`: NOT_RUN
- `BIRD_PATROL_AUTONOMOUS_READY`: NOT_RUN

## What Is Available Now

- Clean field release generation and validation.
- Local operator station scripts and SSH/Jetson Docker bridge contracts.
- Source/static/dry-run checks for launch, command chain, network, and release hygiene.
- Deterministic Gazebo UI SLAM + bird display regression evidence.
- Hardware probe scripts and checklists ready for live Jetson/Livox/camera/base tests.

## What Is Not Yet Proven

- Autonomous wheel-on field patrol.
- Live Livox pointcloud/scan/TF quality on the target Jetson.
- Live camera detector, camera-LiDAR calibration, and 3D fusion readiness.
- Real sound deterrent backend and operator/legal/hardware ACK.
- Real base feedback, odometry, braking, E-stop, and blackbox evidence under motion.

## Recent Readiness Reports

| Report | Mode | Status | Failed | Blocked |
| --- | --- | --- | --- | --- |
| `1783239228_source.json` | source | PASS | 0 | 0 |
| `1783243243_source.json` | source | PASS | 0 | 0 |
| `1783243995_source.json` | source | PASS | 0 | 0 |
| `1783245712_source.json` | source | PASS | 0 | 0 |
| `1783245714_source.json` | source | PASS | 0 | 0 |
| `1783248394_source.json` | source | PASS | 0 | 0 |
| `1783250901_source.json` | source | PASS | 0 | 0 |
| `1783254166_source.json` | source | PASS | 0 | 0 |
| `1783255050_source.json` | source | PASS | 0 | 0 |
| `1783258975_source.json` | source | PASS | 0 | 0 |
| `1783260401_source.json` | source | PASS | 0 | 0 |
| `1783260404_source.json` | source | PASS | 0 | 0 |
| `1783262671_source.json` | source | PASS | 0 | 0 |
| `1783265106_source.json` | source | PASS | 0 | 0 |
| `1783265525_source.json` | source | PASS | 0 | 0 |
| `1783267082_source.json` | source | PASS | 0 | 0 |
| `1783267085_source.json` | source | PASS | 0 | 0 |
| `source_latest.json` | source | PASS | 0 | 0 |

# Bird Mission Readiness Levels

This project treats bird patrol as a real hardware mission, not a Gazebo-only demo. A lower level may pass while higher levels remain blocked. Missing hardware evidence must never be upgraded into an autonomous-ready claim.

| Level | Name | Required Evidence | Allowed Judgment |
| --- | --- | --- | --- |
| B0 | SOURCE_AND_NAV_BASE | Compile/static/unit/contract checks pass, clean field release can be built, no hardware access required. | BIRD_MISSION_READY_SOURCE |
| B1 | SENSOR_LIVE | MID-360 or equivalent PointCloud2 live, camera image and camera_info live, scan/scan_safety live, odom/tf live, no fake/Gazebo/test publishers. | BIRD_MISSION_SENSOR_LIVE |
| B2 | LIDAR_OBJECT_TRACKING | PointCloud2 creates elevated object candidates, dynamic target tracking publishes z/dynamic/range/velocity-gated state, robot-body reflections are rejected. | BIRD_MISSION_LIDAR_TRACKING_READY |
| B3 | CAMERA_BIRD_DETECTOR | YOLO or external detector backend ready, model exists, import succeeds, camera is valid, bird/non-bird/person/vehicle/drone/unknown class map exists, latency below threshold. | BIRD_MISSION_CAMERA_DETECTOR_READY |
| B4 | CAMERA_LIDAR_FUSION | Calibrated camera-LiDAR extrinsic, static TF verified, Detection2DArray/CameraInfo/PointCloud2 synchronized, bbox point count/spread/range/height checked. | BIRD_MISSION_FUSION_READY |
| B5 | INSPECTION_MISSION_DRY_RUN | Nav2 patrol running, LiDAR target creates a standoff inspection goal, camera/body/gimbal command is produced, classification controls sound request, real sound output disabled. | BIRD_MISSION_INSPECTION_DRY_RUN_READY |
| B6 | DETERRENCE_HARDWARE_READY | Real sound backend verified, volume/duration/cooldown/retry limits tested, E-stop/external stop cancels output, legal/hardware/operator ack present. | BIRD_MISSION_DETERRENCE_HARDWARE_READY |
| B7 | AUTONOMOUS_BIRD_PATROL_READY | B0-B6 pass, blackbox recorder enabled, collision/obstacle stop enabled, battery return enabled, target departure/timeout handled, interrupted patrol resumes, supervised closed-area evidence exists. | BIRD_MISSION_AUTONOMOUS_PATROL_READY |

`NOT_BIRD_MISSION_READY` is used when the requested level fails.

Hardware-less validation can only claim `BIRD_MISSION_READY_SOURCE`. Simulation, fake detector output, topic-only gimbal alignment, and Gazebo ground truth are not B1+ field evidence.

Production policy:

- The bird mission pipeline remains part of the product stack.
- Capabilities that are not ready must publish `DEGRADED`, `BLOCKED`, or `FAIL_CLOSED`, not silently disappear.
- Camera-LiDAR extrinsic with `calibrated: false` blocks B4+ and forces fusion invalid.
- Topic-only alignment is dry-run only and cannot publish production `centered=true`.
- Real sound output is blocked unless hardware, local law, and operator supervision acknowledgments are present.

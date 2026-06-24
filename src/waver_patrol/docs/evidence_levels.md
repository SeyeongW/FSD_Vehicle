# Evidence Levels

Use these levels in reports and experiment metadata so simulation and field
evidence are not mixed.

| level | name | meaning |
| --- | --- | --- |
| L0 | static/contract check | Source, launch, config, and safety contract checks only. |
| L1 | pure unit test | Hardware-free Python logic tests. |
| L2 | Gazebo synthetic pipeline validation | Simulated sensors, fake labels, or fake sound states may be used. |
| L3 | rosbag replay with real sensors | Real sensor data replayed without live motion. |
| L4 | wheel-off hardware test | Real serial/base driver with wheels raised or disconnected. |
| L5 | low-speed wheel-on field test | Real robot motion at initial capped speed. |
| L6 | repeated outdoor mission trials | Repeated real missions with documented conditions and operator safety record. |

## Required Log Fields

Experiment CSV/summary files should include these fields when practical:

- `evidence_level`
- `sim_or_real`
- `detector_backend`
- `detector_model_path_hash` or `model_id`
- `fake_detector_used`
- `fake_sound_used`
- `serial_enabled`
- `safety_ack`
- `commit_hash`

Gazebo fake camera or fake sound trials are L2 pipeline evidence. They can show
that the mission state machine and logger are wired, but they are not real
detector accuracy or real deterrence performance.

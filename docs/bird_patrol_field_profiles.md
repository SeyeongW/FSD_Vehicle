# Bird Patrol Field Profiles

These profiles separate real Waver bringup into staged, evidence-gated steps.
Do not jump from source validation directly to autonomous bird patrol.

| Profile | Purpose | Motion | Bird stack | Sound output |
| --- | --- | --- | --- | --- |
| `wheel_off_driver_check.yaml` | Base serial, stop burst, feedback, and wheel direction check with wheels off ground. | Tiny supervised command only | Off | Off |
| `wheel_on_low_speed.yaml` | First closed-area wheel-on check. | <= 0.05 m/s, <= 0.20 rad/s | LiDAR obstacle perception only | Off |
| `sensor_live.yaml` | Livox/camera topic, rate, frame, and timestamp probe. | None | Detector/fusion off | Off |
| `inspection_dry_run.yaml` | First-wheel-on supervised inspection movement dry-run. It may move only at the first-wheel-on tier while sound remains locked off. | <= 0.05 m/s, <= 0.20 rad/s | On | Off |
| `supervised_bird_patrol.yaml` | Human-supervised low-speed patrol and inspection after first-wheel-on, collision-monitor, blackbox, and operator-confirmation evidence. | 0.08-0.12 m/s, 0.25-0.35 rad/s | On | Locked off unless external ACK is supplied |
| `bird_patrol_production.yaml` | Nested source-of-truth production profile. | Evidence-gated | On | Locked off unless external ACK is supplied |
| `autonomous_bird_patrol_locked.yaml` | Autonomous target profile. It remains blocked until fresh hardware, collision-monitor, E-stop, detector, fusion, sound, and blackbox evidence exists. | Evidence-gated | On | Locked off unless external ACK is supplied |

The production schema source of truth is the nested sections in
`bird_patrol_production.yaml`. Flat keys are retained only as compatibility
aliases for existing field scripts.

`scripts/waver_bird_patrol_field_start.sh` automatically selects these staged
profiles when `--profile` is omitted. This prevents a first sensor probe from
accidentally starting the full production bird patrol stack.

Speed tiers are explicit:

- `monitoring-only`: no motion.
- `first-wheel-on`: <= 0.05 m/s, <= 0.20 rad/s.
- `supervised-low-speed`: 0.08-0.12 m/s, 0.25-0.35 rad/s.
- `production`: rejected unless `WAVER_ACK_PRODUCTION_SPEED_EVIDENCE=1`.

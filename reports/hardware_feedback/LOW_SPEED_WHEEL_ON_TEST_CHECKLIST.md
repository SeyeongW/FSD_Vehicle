# Low-Speed Wheel-On Test Checklist

This checklist is for a human operator. Do not automate it.

Prerequisites:

- Closed test area.
- No people, vehicles, or fragile objects nearby.
- Physical E-stop ready.
- Wheel-off checklist passed.
- Speed clamp: linear `0.03-0.05 m/s`, angular `<= 0.20 rad/s`.
- `waver_real_preflight_check` or equivalent graph checks pass.

Sequence:

1. Start real launch in STANDBY.
2. Confirm `/cmd_vel` is zero.
3. Confirm serial owner count is one.
4. Manual forward/backward 0.1 m.
5. Manual left/right pivot at low angular command.
6. E-stop immediate stop.
7. Resume only after safety reset.
8. Nav2 single goal under 0.5 m.
9. Two-waypoint patrol at low speed.
10. Static obstacle stop check.
11. Save rosbag and operator report.


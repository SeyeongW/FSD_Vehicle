# Patrol Operation

Waver patrol starts only when the map, `/tf`, `/scan`, localization covariance, serial ownership,
and manual E-Stop operator are ready.

Set initial pose in RViz/Nav2 before patrol. Waypoints are YAML files in `waypoints/` with `home`,
`loop_count`, `mode`, and waypoint entries. Use `loop_count: -1` for continuous supervised patrol,
`mode: pingpong` to reverse at the ends, or `mode: reverse` to traverse in reverse order.

When a person or dynamic obstacle appears, Waver stops or slows based on hard-stop and TTC limits.
A manual key command immediately enters `MANUAL_OVERRIDE`, cancels patrol intent, and patrol must not
resume until the operator resets/resumes. After E-Stop, run `stop_robot`, inspect the cause, then reset.

Logs are written under `logs/` when CLI tools can create the directory. During tests, watch:

```bash
ros2 topic hz /scan
ros2 topic echo /amcl_pose
ros2 node list
ros2 run waver_patrol inspect_stack
```

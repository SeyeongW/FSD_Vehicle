# Waver Reports

Generated reports live here. They are intentionally separated from source code
and should not be treated as launch/config input unless explicitly documented.

```text
reports/
  agent_iterations/   bounded no-hardware convergence loop summaries
  quality_gate/       hardware-free quality gate logs and latest summaries
  hardware_feedback/  manual wheel-off/wheel-on checklists and reports
  rosbag_replay/      replay notes and bag-derived regression summaries
```

Large rosbags and heavy Gazebo data should be archived outside git and copied
back only as small summaries or focused regression fixtures.

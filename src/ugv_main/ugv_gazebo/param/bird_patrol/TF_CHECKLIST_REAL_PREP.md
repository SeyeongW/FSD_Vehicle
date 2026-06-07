# Real Prep TF Checklist

This checklist is for future real-vehicle adaptation only. Do not treat it as field validation.

Required frames:

- `map`
- `odom`
- `base_link` or `base_footprint` with a documented static transform
- Mid360/Livox frame, expected candidate: `livox` or `livox_frame`
- Camera optical frame, expected candidate: `camera_optical_frame` or Gazebo-compatible camera frame remapped to the real optical frame

Required TF chains:

```text
map -> odom
odom -> base_link
base_link -> livox_frame
base_link -> camera_optical_frame
```

Before real wheel-on:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

Rules:

- Camera-only detection must not create a navigation goal.
- Dynamic LiDAR target or 3D fusion validity is required before approach/sound flow.
- Final `/cmd_vel` must still be published only by `safety_cmd_mux_node`.
- `/waver/mode` must still be published only by `mission_patrol_manager_node`.

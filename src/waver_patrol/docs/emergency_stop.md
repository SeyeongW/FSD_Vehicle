# Emergency Stop

Immediate stop command:

```bash
ros2 run waver_patrol stop_robot
```

Keyboard teleop stop keys are Space, `K`, ESC, and Ctrl+C. Stop sends `{"T":1,"L":0,"R":0}` and
is never inverted or swapped by motor configuration.

After E-Stop, do not resume patrol until the operator inspects serial ownership, LiDAR, localization,
battery, thermal status, and obstacle conditions.

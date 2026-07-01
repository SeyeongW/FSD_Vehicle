# Open-Source Comparison Matrix

Purpose: compare Waver's current ROS 2 stack against common field-robot patterns before real-vehicle use. This is a design audit, not a license or code import decision.

## Comparison Targets

| Reference Pattern | Typical Strength | Waver Current Alignment | Gap / Action |
| --- | --- | --- | --- |
| Nav2 TurtleBot-style bringup | Clear separation between Nav2 controller output and final robot command | Waver remaps Nav2 output into `/waver/cmd_vel_nav2` or smoothed candidate topics | Keep final `/cmd_vel` owned by `safety_cmd_mux_node`; do not let Nav2 publish `/cmd_vel` directly. |
| Clearpath-style field bringup | Conservative launch defaults, explicit sensors, operator checklist | Indoor real launch defaults to `STANDBY`, low speed, scan required | Keep separate runbooks for Gazebo, indoor wheel-off, and wheel-on. |
| TurtleBot3 SLAM demos | SLAM isolated from patrol and old static map source | Waver has mapping workflow and UI map reset behavior | Validate with Gazebo obstacle smoke and map quality check before claiming SLAM readiness. |
| Autoware safety layering | Candidate command generators feed a safety/arbitration layer | Waver has manual/Nav2/target candidate topics and a safety mux | Collision Monitor is not made final owner in this pass; that needs a separate integration plan. |
| Livox ROS driver deployments | Vendor driver publishes point cloud/IMU; downstream adapter handles safety scan | Waver expects `/livox/lidar` and `/livox/imu`, with scan adapter diagnostics | Real readiness still requires live Mid-360 rate, frame, TF, and coverage checks. |
| Dataset/experiment stacks | Metrics and evidence files are separated from runtime command authority | Waver experiment logger and reports are separated from command topics | Keep generated experiment data out of source archives unless intentionally curated. |

## Good Existing Waver Patterns

- Candidate command topics are separated from final `/cmd_vel`.
- Remote UI defaults away from direct `/cmd_vel` publishing.
- Real field scripts route through Jetson Docker and the selected base driver instead of local serial.
- Indoor real profile is conservative and avoids camera/bird/sound/test stacks by default.
- Preflight and scan quality helpers expose concrete operator checks.

## Main Weaknesses To Keep Watching

- Many historical Gazebo/test scripts remain; operators must use documented entry points.
- Gazebo success does not prove real Livox frame orientation or scan coverage.
- Open-loop fallback patrol is useful for early Waver movement checks but is not localization-based autonomy.
- SSH/Docker field workflow depends on correct `JETSON_HOST`, container name, and `install_docker` state.
- SLAM mapping can appear visually plausible while map quality or static obstacle persistence is poor; use map quality checks.

## Applied Policy For This Pass

- Improve validation wrappers and documentation without changing the core architecture.
- Preserve the existing SSH -> Jetson -> Docker -> Waver USB serial workflow.
- Do not import external code or restructure packages.
- Record skipped Gazebo/Docker/SSH checks honestly when the local environment is missing.

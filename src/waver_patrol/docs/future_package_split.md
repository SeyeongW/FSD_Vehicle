# Future Package Split Recommendation

Do not split the repository until current Gazebo and real-prep tests are stable.
This note records a low-risk migration path.

| future package | candidate contents | why it helps | migration risk |
| --- | --- | --- | --- |
| `waver_platform_driver` | `waver_base_driver_node`, serial protocol helpers, base feedback contracts. | Keeps hardware I/O thin and auditable. | Serial launch remaps must remain identical. |
| `waver_safety` | `safety_cmd_mux_node`, speed limits, stale command guards, preflight scripts. | Makes final `/cmd_vel` ownership explicit. | Any topic rename can break field scripts. |
| `waver_mission` | patrol manager, target goal manager, departure monitor, return-to-patrol logic. | Separates behavior from perception and platform. | Nav2 action state tests required before/after. |
| `waver_perception` | bird detector, 3D fusion, LiDAR object extraction, motion filtering. | Makes real detector contracts easier to evaluate. | Message dependencies and TF tests must move cleanly. |
| `waver_sim` | Gazebo-only nodes, fake camera/sound states, scenario launch files. | Prevents fake/test nodes from leaking into real profile. | Launch compatibility needs static checks. |
| `waver_eval` | experiment loggers, rosbag replay checks, metrics utilities. | Keeps paper/data tooling out of runtime stack. | Existing experiment paths need aliases. |
| `waver_operator_ui` | remote panel and operator map UI helpers. | UI can evolve without touching base driver. | Field SSH/Docker bridge scripts must be preserved. |

Recommended order:

1. Extract pure safety and mission helpers with tests.
2. Split Gazebo-only simulation package from real profile launch paths.
3. Split evaluation/logging after schemas are stable.
4. Split platform driver only after wheel-off and wheel-on contracts are stable.

Every move needs static contract checks, no-ROS tests, ROS tests, and at least
one Gazebo smoke test before field use.

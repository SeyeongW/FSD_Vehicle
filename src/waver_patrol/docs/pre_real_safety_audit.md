# Pre-Real Safety Audit

## Mandatory Gates

| Gate | Expected state | Verification |
|---|---|---|
| Final command publisher | only `safety_cmd_mux_node` publishes `/cmd_vel` | `ros2 topic info -v /cmd_vel` |
| Operator panel | publishes `/waver/manual_cmd_vel`, not final `/cmd_vel` by default | `publish_direct_cmd_vel:=false` |
| Cluster/perception | publishes object candidates only | no `/cmd_vel` publisher in cluster path |
| Serial bridge | off during Gazebo validation | `start_serial_bridge:=false` |
| Sound output | disabled by default | `enable_sound_output:=false` |
| Scan safety | required on real robot | `require_scan:=true` for field tests |
| Gazebo validation | H1/H2/H3 height-based trials pass; H4/H5/H6 recommended | `run_pre_real_gazebo_trials.sh` |

## Known Local Finding

`pcd_cluster_pkg/cluster_node.py` is not present in this local `jo` workspace. The Gazebo validation uses a Waver fake cluster publisher with the same `/waver/lidar_objects` interface. When `pcd_cluster_pkg` is restored, it must keep `direct_cmd_vel_enabled=false`.

## Real Robot Blockers Before Field Autonomy

- Verify live `/scan`, `/odom`, `/tf`, and saved `/map`.
- Verify `map->odom->base_link->laser/camera` TF tree.
- Verify `/cmd_vel` has exactly one publisher.
- Keep physical E-stop and operator present.
- Start at `max_patrol_speed <= 0.1 m/s`.
- Keep sound output disabled until legal and hardware safety approval.

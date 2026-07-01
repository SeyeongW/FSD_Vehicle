# Remote UI Feature Inventory

Runtime file: `src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py`

## Command Path

| UI Feature | Topic / Bridge Packet | Real Safety Rule |
| --- | --- | --- |
| Manual WASD | `/waver/manual_cmd_vel` | Candidate command only; safety mux publishes final `/cmd_vel`. |
| Mode buttons | `/waver/mode_cmd` | UI does not publish `/waver/mode`; mission manager owns mode state. |
| Start/stop patrol | `/waver/mission_command` | `START_PATROL` must be blocked by mission/safety gates when unsafe. |
| SLAM mapping | `/waver/mapping_command` | Mapping must not start patrol goals or old fixed map source. |
| Emergency stop | `/waver/emergency_stop` | Must force zero command through safety path. |
| External stop | `/waver/external_stop` | Must behave as a stop gate, not a direct motor publisher. |

## Display Path

- `/waver/mode`
- `/waver/mission_state`
- `/waver/mapping_state`
- `/waver/current_map_source`
- `/waver/safety_state`
- `/waver/mapping_path`
- `/waver/target_class`
- `/waver/target_confidence`
- dynamic obstacle / target marker topics when enabled

## Field Bridge

The local panel can bridge commands to the Jetson Docker ROS graph:

```text
local PC waver_remote_panel
  -> SSH
  -> Jetson host
  -> docker exec fsd_dev_jetson
  -> ROS 2 topics inside /ros2_ws/ros2_ws5
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node
  -> Waver USB serial
```

Required defaults:

- `publish_direct_cmd_vel=false`
- `remote_bridge_enabled=true` only for field bridge mode
- `remote_bridge_use_docker=true`
- `remote_bridge_container=fsd_dev_jetson`
- `remote_bridge_container_workspace=/ros2_ws/ros2_ws5`
- `remote_bridge_ros_domain_id=0`

## Validation

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 2
```

This static/mock validation checks the UI contract and does not publish real motor commands.

## Manual Field Confirmation

Before wheel-on:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/manual_cmd_vel
ros2 topic echo --once /waver/safety_state
```

Expected:

- `/cmd_vel` publisher count is 1.
- Publisher node is `safety_cmd_mux_node`.
- UI is not a direct `/cmd_vel` publisher.

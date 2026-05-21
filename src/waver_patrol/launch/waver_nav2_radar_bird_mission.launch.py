from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_config = os.path.join(share, "config", "waver_nav2_radar_bird_mission.yaml")
    default_waypoints = os.path.join(share, "waypoints", "waver_nav2_patrol_mission.yaml")
    default_nav2_params = os.path.join(share, "config", "nav2_params_waver.yaml")
    try:
        default_map = os.path.join(get_package_share_directory("ugv_nav"), "maps", "map.yaml")
    except PackageNotFoundError:
        default_map = os.path.join(share, "maps", "map.yaml")

    config_file = LaunchConfiguration("config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    require_scan = LaunchConfiguration("require_scan")
    default_mode = LaunchConfiguration("default_mode")
    use_nav2 = LaunchConfiguration("use_nav2")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    common = [config_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    serial_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_serial_bridge"),
                "' == 'true' and '",
                LaunchConfiguration("include_existing_ugv_driver"),
                "' == 'false'",
            ]
        )
    )

    ugv_nav_include = GroupAction(
        condition=IfCondition(use_nav2),
        actions=[
            LogInfo(msg="Nav2 enabled: remapping Nav2 /cmd_vel output to /waver/cmd_vel_nav2 before safety mux."),
            SetRemap(src="/cmd_vel", dst="/waver/cmd_vel_nav2", condition=IfCondition(LaunchConfiguration("remap_nav2_cmd_vel"))),
            SetRemap(src="cmd_vel", dst="/waver/cmd_vel_nav2", condition=IfCondition(LaunchConfiguration("remap_nav2_cmd_vel"))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ugv_nav"), "/launch/nav.launch.py"]
                ),
                launch_arguments={
                    "use_localization": LaunchConfiguration("use_localization"),
                    "use_localplan": LaunchConfiguration("use_localplan"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_localization", default_value="amcl"),
            DeclareLaunchArgument("use_amcl", default_value="true"),
            DeclareLaunchArgument("use_slam", default_value="false"),
            DeclareLaunchArgument("use_localplan", default_value="dwa"),
            DeclareLaunchArgument("enable_mission_patrol_manager", default_value="true"),
            DeclareLaunchArgument("enable_radar_command_bridge", default_value="true"),
            DeclareLaunchArgument("enable_target_goal_manager", default_value="true"),
            DeclareLaunchArgument("enable_safety_cmd_mux", default_value="true"),
            DeclareLaunchArgument("enable_battery_return", default_value="true"),
            DeclareLaunchArgument("enable_deep_learning_stub", default_value="true"),
            DeclareLaunchArgument("enable_sound_stub", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("enable_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("enable_test_publishers", default_value="false"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("default_mode", default_value="AUTO"),
            DeclareLaunchArgument("enable_sim_nav_goal_arrival", default_value="false"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="false"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="false"),
            DeclareLaunchArgument("enable_moving_object_map_transform", default_value="false"),
            DeclareLaunchArgument("enable_moving_object_goal_relay", default_value="false"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/mid360_PointCloud2"),
            DeclareLaunchArgument("moving_object_input_topic", default_value="/waver/lidar_objects"),
            DeclareLaunchArgument("moving_object_input_type", default_value="pose_array"),
            DeclareLaunchArgument("moving_object_output_topic", default_value="/waver/lidar_objects_map"),
            DeclareLaunchArgument("moving_object_marker_topic", default_value="/waver/moving_objects_map_marker"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_forward_axis", default_value="x"),
            DeclareLaunchArgument("scan_lateral_axis", default_value="y"),
            DeclareLaunchArgument("scan_height_axis", default_value="z"),
            DeclareLaunchArgument("scan_positive_lateral_is_left", default_value="true"),
            DeclareLaunchArgument("remap_nav2_cmd_vel", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(
                msg=(
                    "Safety rule: final /cmd_vel must have exactly one publisher: safety_cmd_mux_node. "
                    "Check with `ros2 topic info -v /cmd_vel` before enabling serial."
                )
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("start_serial_bridge")),
                msg="start_serial_bridge=true: stop ugv_driver/app.py/other serial bridges first.",
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_livox_scan_adapter")),
                msg=(
                    "3D LiDAR safety adapter enabled: PointCloud2 is projected to /scan for ground-obstacle "
                    "stop/slowdown only. 2D LaserScan has no z/height information."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_lidar_objects")),
                msg=(
                    "3D LiDAR object candidate adapter enabled: PointCloud2 centers are published to "
                    "/waver/lidar_objects and never directly to /cmd_vel."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_moving_object_map_transform")),
                msg=(
                    "Moving object map transform enabled: LiDAR object coordinates are transformed to map/odom "
                    "data topics. Radar mission command topics remain separate."
                ),
            ),
            ugv_nav_include,
            Node(
                package="waver_patrol",
                executable="livox_pointcloud_to_scan_node",
                name="livox_pointcloud_to_scan_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_livox_scan_adapter")),
                parameters=[
                    *common,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "scan_topic": scan_topic,
                        "forward_axis": ParameterValue(LaunchConfiguration("scan_forward_axis"), value_type=str),
                        "lateral_axis": ParameterValue(LaunchConfiguration("scan_lateral_axis"), value_type=str),
                        "height_axis": ParameterValue(LaunchConfiguration("scan_height_axis"), value_type=str),
                        "positive_lateral_is_left": ParameterValue(
                            LaunchConfiguration("scan_positive_lateral_is_left"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="pointcloud_lidar_objects_node",
                name="pointcloud_lidar_objects_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_lidar_objects")),
                parameters=[
                    *common,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "output_pose_array_topic": "/waver/lidar_objects",
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_map_transform_node",
                name="moving_object_map_transform_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_map_transform")),
                parameters=[
                    *common,
                    {
                        "input_topic": ParameterValue(LaunchConfiguration("moving_object_input_topic"), value_type=str),
                        "input_type": ParameterValue(LaunchConfiguration("moving_object_input_type"), value_type=str),
                        "output_pose_array_topic": ParameterValue(LaunchConfiguration("moving_object_output_topic"), value_type=str),
                        "debug_marker_topic": ParameterValue(LaunchConfiguration("moving_object_marker_topic"), value_type=str),
                        "target_frame": "map",
                        "fallback_frame": "odom",
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_goal_relay_node",
                name="moving_object_goal_relay_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_goal_relay")),
                parameters=[
                    *common,
                    {
                        "input_pose_array_topic": ParameterValue(LaunchConfiguration("moving_object_output_topic"), value_type=str),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="radar_command_bridge_node",
                name="radar_command_bridge_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_radar_command_bridge")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="target_goal_manager_node",
                name="target_goal_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_target_goal_manager")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="mission_patrol_manager_node",
                name="mission_patrol_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_mission_patrol_manager")),
                parameters=[
                    *common,
                    {
                        "waypoint_file": waypoint_file,
                        "use_nav2": ParameterValue(use_nav2, value_type=bool),
                        "default_mode": default_mode,
                        "enable_sim_nav_goal_arrival": ParameterValue(
                            LaunchConfiguration("enable_sim_nav_goal_arrival"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="battery_return_manager_node",
                name="battery_return_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_battery_return")),
                parameters=[*common, {"waypoint_file": waypoint_file}],
            ),
            Node(
                package="waver_patrol",
                executable="deep_learning_bridge_stub",
                name="deep_learning_bridge_stub",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_deep_learning_stub")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="sound_alert_stub",
                name="sound_alert_stub",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_sound_stub")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="safety_cmd_mux_node",
                name="safety_cmd_mux_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_safety_cmd_mux")),
                parameters=[
                    *common,
                    {
                        "require_scan": ParameterValue(require_scan, value_type=bool),
                        "mode_default": default_mode,
                        "nav2_cmd_topic": "/waver/cmd_vel_nav2",
                        "cmd_vel_auto_topic": "",
                        "scan_topic": scan_topic,
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="experiment_data_logger_node",
                name="experiment_data_logger_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_experiment_logger")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="teleop_node",
                name="waver_keyboard_teleop",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_keyboard_teleop")),
            ),
            Node(
                package="waver_patrol",
                executable="serial_cmd_vel_bridge",
                name="serial_cmd_vel_bridge",
                output="screen",
                condition=serial_condition,
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="radar_target_test_publisher_node",
                name="radar_target_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="fake_camera_classification_test_publisher_node",
                name="fake_camera_classification_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
        ]
    )

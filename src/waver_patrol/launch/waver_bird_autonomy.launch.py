from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_config = os.path.join(share, "config", "waver_bird_autonomy.yaml")
    default_waypoints = os.path.join(share, "waypoints", "waver_bird_patrol_demo.yaml")

    config_file = LaunchConfiguration("config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    default_mode = LaunchConfiguration("default_mode")
    require_scan = LaunchConfiguration("require_scan")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    use_tf_transform = LaunchConfiguration("use_tf_transform")

    common = [config_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    serial_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_serial_bridge"),
                "' == 'true' and '",
                LaunchConfiguration("include_existing_bringup"),
                "' == 'false' and '",
                LaunchConfiguration("include_existing_ugv_driver"),
                "' == 'false'",
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("default_mode", default_value="AUTO"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_sim_odom", default_value="false"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("include_existing_bringup", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("enable_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("enable_fixed_waypoint_patrol", default_value="true"),
            DeclareLaunchArgument("enable_lidar_motion_detector", default_value="true"),
            DeclareLaunchArgument("enable_target_tracking", default_value="true"),
            DeclareLaunchArgument("enable_battery_return", default_value="true"),
            DeclareLaunchArgument("enable_return_home_controller", default_value="true"),
            DeclareLaunchArgument("enable_deep_learning_stub", default_value="true"),
            DeclareLaunchArgument("enable_sound_stub", default_value="true"),
            DeclareLaunchArgument("enable_test_publishers", default_value="false"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="false"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="false"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/mid360_PointCloud2"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_forward_axis", default_value="x"),
            DeclareLaunchArgument("scan_lateral_axis", default_value="y"),
            DeclareLaunchArgument("scan_height_axis", default_value="z"),
            DeclareLaunchArgument("scan_positive_lateral_is_left", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("use_tf_transform", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("start_serial_bridge")),
                msg=(
                    "start_serial_bridge=true: verify existing ugv_driver/app.py/serial bridge is stopped. "
                    "Only one process may own the WAVE ROVER serial port."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("include_existing_bringup")),
                msg="include_existing_bringup=true: keep start_serial_bridge=false. Existing ugv_driver owns serial.",
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                msg="RViz is not started by this launch; open your preferred RViz config manually.",
            ),
            Node(
                package="waver_patrol",
                executable="fixed_waypoint_patrol_node",
                name="fixed_waypoint_patrol_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_fixed_waypoint_patrol")),
                parameters=[*common, {"waypoint_file": waypoint_file, "mode_default": default_mode}],
            ),
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
                        "positive_lateral_is_left": ParameterValue(LaunchConfiguration("scan_positive_lateral_is_left"), value_type=bool),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="pointcloud_lidar_objects_node",
                name="pointcloud_lidar_objects_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_lidar_objects")),
                parameters=[*common, {"pointcloud_topic": pointcloud_topic}],
            ),
            Node(
                package="waver_patrol",
                executable="lidar_aerial_motion_detector_node",
                name="lidar_aerial_motion_detector_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_lidar_motion_detector")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="target_body_tracker_node",
                name="target_body_tracker_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_target_tracking")),
                parameters=[*common, {"use_tf_transform": ParameterValue(use_tf_transform, value_type=bool)}],
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
                executable="return_home_controller_node",
                name="return_home_controller_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_return_home_controller")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="auto_behavior_mux_node",
                name="auto_behavior_mux_node",
                output="screen",
                parameters=[*common, {"default_mode": default_mode}],
            ),
            Node(
                package="waver_patrol",
                executable="safety_cmd_mux_node",
                name="safety_cmd_mux_node",
                output="screen",
                parameters=[
                    *common,
                    {
                        "require_scan": ParameterValue(require_scan, value_type=bool),
                        "mode_default": default_mode,
                        "scan_topic": scan_topic,
                    },
                ],
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
                executable="serial_cmd_vel_bridge",
                name="serial_cmd_vel_bridge",
                output="screen",
                condition=serial_condition,
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
                executable="simple_sim_odom_node",
                name="simple_sim_odom_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim_odom")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="aerial_target_test_publisher_node",
                name="aerial_target_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="lidar_objects_test_publisher_node",
                name="lidar_objects_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="battery_test_publisher_node",
                name="battery_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
        ]
    )

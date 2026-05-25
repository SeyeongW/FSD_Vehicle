from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    algorithm = LaunchConfiguration("algorithm")
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ugv_slam"), "launch", "cartographer.launch.py")
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_lidar_bringup": LaunchConfiguration("start_lidar_bringup"),
            "start_robot_pose_publisher": LaunchConfiguration("start_robot_pose_publisher"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=LaunchConfigurationEquals("algorithm", "cartographer"),
    )
    gmapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ugv_slam"), "launch", "gmapping.launch.py")
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_lidar_bringup": LaunchConfiguration("start_lidar_bringup"),
            "start_robot_pose_publisher": LaunchConfiguration("start_robot_pose_publisher"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "scan_topic": LaunchConfiguration("scan_topic"),
        }.items(),
        condition=LaunchConfigurationEquals("algorithm", "gmapping"),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("algorithm", default_value="cartographer"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("start_lidar_bringup", default_value="true"),
            DeclareLaunchArgument("start_robot_pose_publisher", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            cartographer,
            gmapping,
        ]
    )

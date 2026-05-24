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
        }.items(),
        condition=LaunchConfigurationEquals("algorithm", "gmapping"),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("algorithm", default_value="cartographer"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("start_lidar_bringup", default_value="true"),
            DeclareLaunchArgument("start_robot_pose_publisher", default_value="true"),
            cartographer,
            gmapping,
        ]
    )

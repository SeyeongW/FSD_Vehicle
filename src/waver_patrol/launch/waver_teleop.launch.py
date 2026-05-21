from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            Node(
                package="waver_patrol",
                executable="teleop_node",
                name="waver_teleop_node",
                output="screen",
                parameters=[{"config": LaunchConfiguration("config")}],
            ),
        ]
    )

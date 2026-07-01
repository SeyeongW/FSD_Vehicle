import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cartographer_config_package = LaunchConfiguration("cartographer_config_package")
    cartographer_config_dir = LaunchConfiguration("cartographer_config_dir")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    ws_path = os.environ.get("ROS2_WS5_PATH", os.path.expanduser("~/ros2_ws5/FSD_Vehicle"))
    pbstream_path = LaunchConfiguration("pbstream_path")
    default_cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare(cartographer_config_package), "config"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cartographer_config_package",
                default_value="cartographer",
                description="Package that provides Cartographer Lua config files",
            ),
            DeclareLaunchArgument(
                "cartographer_config_dir",
                default_value=default_cartographer_config_dir,
                description="Full path to Cartographer config directory",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="localization_2d.lua",
                description="Cartographer Lua configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description="Resolution of the published occupancy grid",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="1.0",
                description="OccupancyGrid publishing period",
            ),
            DeclareLaunchArgument(
                "pbstream_path",
                default_value=os.path.join(ws_path, "src/ugv_main/ugv_gazebo/maps/map.pbstream"),
                description="Cartographer state file to load for localization",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    configuration_basename,
                    "-load_state_filename",
                    pbstream_path,
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/occupancy_grid.launch.py"]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "resolution": resolution,
                    "publish_period_sec": publish_period_sec,
                }.items(),
            ),
        ]
    )

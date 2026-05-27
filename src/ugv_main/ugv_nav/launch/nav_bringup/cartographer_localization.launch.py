import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    ugv_cartographer_prefix = get_package_share_directory("ugv_cartographer")
    cartographer_config_dir = LaunchConfiguration(
        "cartographer_config_dir",
        default=os.path.join(ugv_cartographer_prefix, "config"),
    )
    configuration_basename = LaunchConfiguration(
        "configuration_basename",
        default="localization_2d.lua",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")
    ws_path = os.environ.get("UGV_WS_PATH", os.path.expanduser("~/ros2_ws/FSD_Vehicle"))
    pbstream_path = LaunchConfiguration(
        "pbstream_path",
        default=os.path.join(ws_path, "src/ugv_main/ugv_gazebo/maps/map.pbstream"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cartographer_config_dir",
                default_value=cartographer_config_dir,
                description="Full path to Cartographer config directory",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value=configuration_basename,
                description="Cartographer Lua configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value=resolution,
                description="Resolution of the published occupancy grid",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value=publish_period_sec,
                description="OccupancyGrid publishing period",
            ),
            DeclareLaunchArgument(
                "pbstream_path",
                default_value=pbstream_path,
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

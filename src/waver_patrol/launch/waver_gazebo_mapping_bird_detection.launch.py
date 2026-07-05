from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    mapping_debug_launch = os.path.join(waver_share, "launch", "waver_gazebo_mapping_debug.launch.py")

    deterministic_bird_status = (
        "import rclpy, time; "
        "from rclpy.node import Node; "
        "from std_msgs.msg import String, Float32, Bool; "
        "class P(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('deterministic_bird_ui_status_publisher');\n"
        "        self.cls=self.create_publisher(String,'/waver/target_class',10);\n"
        "        self.conf=self.create_publisher(Float32,'/waver/target_confidence',10);\n"
        "        self.bird=self.create_publisher(Bool,'/waver/bird_confirmed',10);\n"
        "        self.det=self.create_publisher(String,'/waver/bird_detector_state',10);\n"
        "        self.fus=self.create_publisher(String,'/waver/bird_fusion_state',10);\n"
        "        self.lidar=self.create_publisher(String,'/waver/lidar_target_state',10);\n"
        "        self.cam=self.create_publisher(String,'/waver/camera_alignment_state',10);\n"
        "        self.center=self.create_publisher(Bool,'/waver/camera_target_centered',10);\n"
        "        self.sound=self.create_publisher(String,'/waver/sound_alert_state',10);\n"
        "        self.done=self.create_publisher(Bool,'/waver/sound_task_done',10);\n"
        "        self.create_timer(0.2,self.t);\n"
        "    def t(self):\n"
        "        now=time.time();\n"
        "        self.cls.publish(String(data='bird'));\n"
        "        self.conf.publish(Float32(data=0.86));\n"
        "        self.bird.publish(Bool(data=True));\n"
        "        self.det.publish(String(data=f'SIM_ONLY_DETECTOR_READY t={now:.2f}'));\n"
        "        self.fus.publish(String(data=f'SIM_ONLY_FUSION_VALID t={now:.2f}'));\n"
        "        self.lidar.publish(String(data=f'SIM_ONLY_TARGET_LOCK display_only t={now:.2f}'));\n"
        "        self.cam.publish(String(data=f'SIM_ONLY_CAMERA_CENTERED display_only t={now:.2f}'));\n"
        "        self.center.publish(Bool(data=True));\n"
        "        self.sound.publish(String(data='SOUND_BLOCKED_MAPPING_MODE SIM_ONLY_DISPLAY_ONLY'));\n"
        "        self.done.publish(Bool(data=False));\n"
        "rclpy.init(); n=P(); rclpy.spin(n)"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("use_operator_panel", default_value="true"),
            DeclareLaunchArgument("mapping_backend", default_value="scan_mapper"),
            DeclareLaunchArgument("enable_bird_ui_status", default_value="true"),
            DeclareLaunchArgument("use_deterministic_bird_topics", default_value="true"),
            DeclareLaunchArgument("integrated_inspection_mode", default_value="false"),
            DeclareLaunchArgument("arm_sound_deterrent", default_value="false"),
            LogInfo(
                msg=(
                    "Waver Gazebo mapping + bird UI status smoke launch. "
                    "This is SIM_ONLY display plumbing, not real detector/fusion evidence. "
                    "Patrol, target approach, and sound deterrent stay disabled by default."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_debug_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "use_operator_panel": LaunchConfiguration("use_operator_panel"),
                    "mapping_backend": LaunchConfiguration("mapping_backend"),
                }.items(),
            ),
            ExecuteProcess(
                cmd=["python3", "-c", deterministic_bird_status],
                output="screen",
                name="deterministic_bird_ui_status_publisher_process",
                condition=IfCondition(LaunchConfiguration("use_deterministic_bird_topics")),
            ),
        ]
    )

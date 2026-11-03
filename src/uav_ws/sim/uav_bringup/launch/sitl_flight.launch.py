from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_config = (
        Path(get_package_share_directory('uav_control')) / 'config' / 'backend.yaml'
    )
    mission_config = (
        Path(get_package_share_directory('uav_mission')) / 'config' / 'mission.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('backend_file', default_value=str(backend_config)),
        Node(
            package='uav_control',
            executable='uav_control_node',
            name='uav_control',
            output='screen',
            parameters=[LaunchConfiguration('backend_file')],
        ),
        Node(
            package='uav_mission',
            executable='mission_node',
            name='uav_mission',
            output='screen',
            parameters=[str(mission_config)],
        ),
    ])

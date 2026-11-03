from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('uav_mission')) / 'config' / 'mission.yaml'
    return LaunchDescription([
        Node(
            package='uav_mission',
            executable='mission_node',
            name='uav_mission',
            output='screen',
            parameters=[str(config)],
        ),
    ])

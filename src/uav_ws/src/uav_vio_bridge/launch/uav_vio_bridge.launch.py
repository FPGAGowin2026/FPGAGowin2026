from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('uav_vio_bridge')) / 'config' / 'uav_vio_bridge.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(config)),
        Node(
            package='uav_vio_bridge',
            executable='uav_vio_bridge_node',
            name='uav_vio_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])

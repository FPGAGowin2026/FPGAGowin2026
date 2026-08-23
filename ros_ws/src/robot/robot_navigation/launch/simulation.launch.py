import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    navigation_share = get_package_share_directory('robot_navigation')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'fpga_sim.launch.py'))),
        Node(package='robot_navigation', executable='sim_laser_node'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_share, 'launch', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'slam': 'True'}.items(),
        ),
    ])

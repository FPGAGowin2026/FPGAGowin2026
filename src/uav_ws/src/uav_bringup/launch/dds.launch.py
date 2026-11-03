from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control = Path(get_package_share_directory('uav_control'))
    mission = Path(get_package_share_directory('uav_mission'))
    return LaunchDescription([
        DeclareLaunchArgument('start_mission', default_value='false',
                              description='Start automatic takeoff/hover/land mission'),
        DeclareLaunchArgument('control_params_file',
                              default_value=''),
        DeclareLaunchArgument('backend_file',
                              default_value=str(control / 'config/backend.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / 'launch/control.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('control_params_file'),
                'backend_file': LaunchConfiguration('backend_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mission / 'launch/mission.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_mission')),
        ),
    ])

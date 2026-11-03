from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('uav_control')) / 'config/backend.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('backend_file', default_value=str(config)),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional parameter file for the selected backend'),
        Node(
            package='uav_control', executable='uav_control_node',
            name='uav_control', output='screen',
            parameters=[LaunchConfiguration('backend_file'), {
                'backend_params_file': ParameterValue(
                    LaunchConfiguration('params_file'), value_type=str),
            }],
        ),
    ])

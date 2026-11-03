from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='uav_01'),
        DeclareLaunchArgument('valid_for_s', default_value='60.0'),
        Node(package='swarm_manager', executable='relay_goal_gateway', output='screen',
             parameters=[{'robot_id': ParameterValue(LaunchConfiguration('robot_id'), value_type=str),
                          'valid_for_s': ParameterValue(LaunchConfiguration('valid_for_s'), value_type=float)}]),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    timeout = ParameterValue(LaunchConfiguration('observation_timeout_s'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('legged_frame', default_value='legged_laser'),
        DeclareLaunchArgument('observation_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('sync_tolerance_s', default_value='0.15'),
        Node(package='swarm_manager', executable='handoff_candidate', output='screen',
             parameters=[{'observation_timeout_s': timeout}]),
        Node(package='swarm_manager', executable='target_projection', output='screen',
             parameters=[{'extrinsic_file': ParameterValue(
                 LaunchConfiguration('extrinsic_file'), value_type=str),
                          'observation_timeout_s': timeout,
                          'sync_tolerance_s': ParameterValue(
                              LaunchConfiguration('sync_tolerance_s'), value_type=float)}]),
        Node(package='swarm_manager', executable='target_to_legged', output='screen',
             parameters=[{'target_frame': ParameterValue(
                 LaunchConfiguration('legged_frame'), value_type=str),
                          'observation_timeout_s': timeout}]),
    ])

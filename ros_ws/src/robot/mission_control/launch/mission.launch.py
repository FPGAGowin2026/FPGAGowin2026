from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('search_goal_x_m', default_value='1.0'),
        DeclareLaunchArgument('search_goal_y_m', default_value='0.0'),
        Node(
            package='mission_control',
            executable='mission_control_node',
            output='screen',
            parameters=[{
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'search_goal_x_m': ParameterValue(
                    LaunchConfiguration('search_goal_x_m'), value_type=float),
                'search_goal_y_m': ParameterValue(
                    LaunchConfiguration('search_goal_y_m'), value_type=float),
            }],
        ),
    ])

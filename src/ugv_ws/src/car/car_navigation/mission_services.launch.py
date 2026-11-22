"""Launch the car mission independently or from the full car bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def mission_nodes(context):
    enabled = lambda name: IfCondition(LaunchConfiguration(name)).evaluate(context)
    config = LaunchConfiguration('config_file').perform(context)
    nodes = []
    if enabled('enable_command'):
        nodes.append(Node(package='ugv_bridge', executable='robot_command', output='screen',
                          parameters=[config] if config else []))
    if enabled('enable_mission'):
        mppi = enabled('enable_waypoint_mppi')
        nodes.append(Node(package='car_navigation', executable='robot_mission', output='screen',
                          parameters=[{'enable_waypoint_mppi': mppi}]))
        if mppi:
            params = PathJoinSubstitution([FindPackageShare('car_navigation'), 'config', 'waypoint_nav.yaml'])
            nodes.extend([
                Node(package='nav2_controller', executable='controller_server',
                     name='controller_server', namespace='waypoint_nav', parameters=[params],
                     # Fast DDS 2.6 can block endpoint creation during async costmap configuration.
                     additional_env={'RMW_FASTRTPS_PUBLICATION_MODE': 'SYNCHRONOUS'},
                     remappings=[('cmd_vel', 'cmd_vel_raw')], output='screen'),
                Node(package='nav2_collision_monitor', executable='collision_monitor',
                     name='collision_monitor', namespace='waypoint_nav', parameters=[params], output='screen'),
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager', namespace='waypoint_nav', output='screen',
                     parameters=[{'autostart': True, 'node_names': ['controller_server', 'collision_monitor']}]),
            ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('enable_command', default_value='true'),
        DeclareLaunchArgument('enable_mission', default_value='true'),
        DeclareLaunchArgument('enable_waypoint_mppi', default_value='true'),
        OpaqueFunction(function=mission_nodes),
    ])

"""Start selected mapping sources and the console; leave motion disabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def mapping_sources(context):
    members = [n.strip() for n in LaunchConfiguration('participants').perform(context).split(',')]
    if (len(set(members)) != len(members) or
            any(n not in ('car', 'acg720') for n in members)):
        raise ValueError('participants must contain unique car or acg720 IDs')
    participants = set(members)
    result = []
    for name, option, package, filename in (
            ('car', 'start_car_mapping', 'car_bringup', 'car_mapping.launch.py'),
            ('acg720', 'start_acg720_mapping', 'legged_bringup', 'acg720_mapping.launch.py')):
        enabled = LaunchConfiguration(option).perform(context).lower() in ('true', '1', 'yes')
        if name not in participants or not enabled:
            continue
        arguments = ({'camera_mount_file': LaunchConfiguration('acg_camera_mount_file')}
                     if name == 'acg720' else {})
        path = PathJoinSubstitution([FindPackageShare(package), 'launch', filename])
        result.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                               launch_arguments=arguments.items()))
    return result


def coordinator(context):
    car_map_topic = LaunchConfiguration('car_map_topic').perform(context)
    if not car_map_topic:
        read_only = LaunchConfiguration('start_car_mapping').perform(context).lower() in ('true', '1', 'yes')
        car_map_topic = '/map' if read_only else '/robot/observation_map'
    return [Node(package='swarm_cooperation', executable='swarm_console',
                 name='swarm_coordinator', output='screen',
                 arguments=['--bind', '127.0.0.1',
                            '--car-camera-info-file', LaunchConfiguration('car_camera_info_file'),
                            '--participants', LaunchConfiguration('participants'),
                            '--anchor', LaunchConfiguration('anchor'),
                            '--car-map-topic', car_map_topic,
                            '--acg-camera-info-file', LaunchConfiguration('acg_camera_info_file'),
                            '--acg-camera-mount-file', LaunchConfiguration('acg_camera_mount_file'),
                            '--allow-preliminary-intrinsics',
                            '--car-radius', LaunchConfiguration('car_radius'),
                            '--sensing', LaunchConfiguration('sensing'),
                            '--separation', LaunchConfiguration('separation'),
                            '--max-position-variance', LaunchConfiguration('max_position_variance'),
                            '--max-yaw-variance', LaunchConfiguration('max_yaw_variance')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('participants', default_value='car,acg720'),
        DeclareLaunchArgument('anchor', default_value='auto'),
        DeclareLaunchArgument('start_car_mapping', default_value='true'),
        DeclareLaunchArgument('start_acg720_mapping', default_value='false'),
        DeclareLaunchArgument('car_map_topic', default_value='',
                              description='Override the map input selected by the car startup mode.'),
        DeclareLaunchArgument('car_camera_info_file', default_value='', description='Car calibration YAML'),
        DeclareLaunchArgument('acg_camera_info_file', default_value='', description='Measured ACG camera intrinsics YAML'),
        DeclareLaunchArgument('acg_camera_mount_file', default_value='', description='Measured ACG camera-to-lidar mount YAML'),
        DeclareLaunchArgument('car_radius', default_value='0.21'),
        DeclareLaunchArgument('sensing', default_value='0.5'),
        DeclareLaunchArgument('separation', default_value='0.8'),
        DeclareLaunchArgument('max_position_variance', default_value='0.01'),
        DeclareLaunchArgument('max_yaw_variance', default_value='0.04'),

        OpaqueFunction(function=mapping_sources),
        OpaqueFunction(function=coordinator),
    ])

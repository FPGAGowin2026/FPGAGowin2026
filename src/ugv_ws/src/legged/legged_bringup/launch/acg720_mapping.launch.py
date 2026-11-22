"""Map the ACG720 lidar on the compute host; never start robot motion."""

from os import pathsep
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def calibration_setup(context):
    info = LaunchConfiguration('camera_info_file').perform(context)
    mount = LaunchConfiguration('camera_mount_file').perform(context)
    if not info and not mount:
        return [LogInfo(msg='ACG720 camera calibration is unconfigured; lidar mapping remains available.')]
    return [Node(package='legged_bridge', executable='acg720_camera_calibration',
                 namespace='acg720', name='camera_calibration', output='screen',
                 parameters=[{'camera_info_file': info, 'camera_mount_file': mount,
                              'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])]


def slam_setup(context, parameters):
    library_dir = LaunchConfiguration('slam_library_dir').perform(context)
    if library_dir == 'auto':
        try:
            private = Path(get_package_prefix('car_navigation')) / 'lib/car_navigation/slam'
        except PackageNotFoundError:
            library_dir = ''
        else:
            library_dir = str(private) if private.is_dir() else ''
    environment = {'RMW_FASTRTPS_PUBLICATION_MODE': EnvironmentVariable(
        'RMW_FASTRTPS_PUBLICATION_MODE', default_value='ASYNCHRONOUS')}
    if library_dir and library_dir != 'system':
        inherited = context.environment.get('LD_LIBRARY_PATH', '')
        environment['LD_LIBRARY_PATH'] = pathsep.join(filter(None, (library_dir, inherited)))
    return [Node(package='slam_toolbox', executable='async_slam_toolbox_node', namespace='acg720',
                 name='slam_toolbox', output='screen',
                 parameters=[*parameters, {'map_file_name': ParameterValue('', value_type=str)}],
                 additional_env=environment)]


def generate_launch_description():
    parameters = [LaunchConfiguration('mapping_config_file'), {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'qos_overrides./tf.publisher.depth': 1}]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_icp', default_value='true'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('enable_calibration', default_value='true'),
        DeclareLaunchArgument('mapping_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('legged_bringup'), 'config', 'acg720_mapping.yaml'])),
        DeclareLaunchArgument('slam_library_dir', default_value='auto',
                              description='auto, system, or a private SLAM library directory.'),
        DeclareLaunchArgument('camera_info_file', default_value='',
                              description='Measured ROS CameraInfo YAML for the 800x480 FPGA stream.'),
        DeclareLaunchArgument('camera_mount_file', default_value='',
                              description='Measured acg720_lidar -> acg720_camera_optical transform YAML.'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        GroupAction(actions=[
            SetRemap(src='/tf', dst=LaunchConfiguration('tf_topic')),
            SetRemap(src='/tf_static', dst=LaunchConfiguration('tf_static_topic')),
            Node(package='rtabmap_odom', executable='icp_odometry', namespace='acg720',
                 condition=IfCondition(LaunchConfiguration('enable_icp')),
                 name='icp_odometry', output='screen', parameters=parameters,
                 remappings=[('scan', '/acg720/scan'), ('odom', '/acg720/odom')]),
            OpaqueFunction(function=slam_setup, args=[parameters],
                           condition=IfCondition(LaunchConfiguration('enable_slam'))),
            OpaqueFunction(function=calibration_setup, condition=IfCondition(LaunchConfiguration('enable_calibration'))),
        ]),
    ])

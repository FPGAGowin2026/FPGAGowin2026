"""Start the FPGA bridge, TensorRT vision, and the mission controller."""
from pathlib import Path
import fcntl
import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


UNSET = '__unset__'
_bringup_lock = None


def acquire_bringup_lock(context):
    global _bringup_lock
    # 同一用户和ROS域只允许一组节点，避免重复视觉源及cmd_vel发布者互相干扰。
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    lock_path = Path(tempfile.gettempdir()) / f'bx72_bringup_{os.getuid()}_{domain}.lock'
    handle = lock_path.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('同一ROS域已有car_bringup运行，请使用现有实例，不要重复启动')
    _bringup_lock = handle
    return []

BRIDGE_OVERRIDES = {
    'bind_pc': ('bind_pc', str),
    'pc_ip': ('pc_ip', str),
    'pc_port': ('pc_port', int),
    'bind_ip': ('bind_fpga', str),
    'fpga_ip': ('fpga_ip', str),
    'control_port': ('fpga_port', int),
    'reply_port': ('ack_port', int),
    'video_port': ('video_port', int),
    'telemetry_port': ('telemetry_port', int),
    'video_fps': ('video_fps', float),
    'jpeg_quality': ('jpeg_quality', int),
}


def create_bridge_node(context):
    overrides = {}
    for argument, (parameter, value_type) in BRIDGE_OVERRIDES.items():
        value = LaunchConfiguration(argument).perform(context)
        if value != UNSET:
            overrides[parameter] = ParameterValue(
                value_type(value), value_type=value_type)

    return [Node(
        package='ugv_bridge',
        executable='ugv_bridge_node',
        name='ugv_bridge_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), overrides],
    )]


def create_local_navigation_node(context):
    # 共用网桥配置中的坐标系和安装外参，避免两份配置各自漂移。
    config_path = LaunchConfiguration('config_file').perform(context)
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    if not isinstance(config, dict):
        raise RuntimeError('bringup 配置必须为 YAML 映射')
    shared = {}
    for name in ('/**', 'ugv_bridge_node'):
        entry = config.get(name, {})
        if isinstance(entry, dict) and isinstance(entry.get('ros__parameters'), dict):
            shared.update(entry['ros__parameters'])
    names = {
        'odom_frame_id': 'odom_frame_id',
        'base_frame_id': 'base_frame_id',
        'n10_frame_id': 'lidar_frame_id',
        'lidar_x_m': 'lidar_x_m',
        'lidar_y_m': 'lidar_y_m',
        'lidar_yaw_rad': 'lidar_yaw_rad',
    }
    overrides = {target: shared[source] for source, target in names.items() if source in shared}
    return [Node(
        package='car_navigation',
        executable='jetson_local_navigation',
        name='jetson_local_navigation',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), overrides],
    )]


def generate_launch_description():
    config_file = str(
        Path(get_package_share_directory('car_bringup')) / 'config' / 'bringup.yaml')
    global_slam_config = str(
        Path(get_package_share_directory('car_navigation')) / 'config' / 'global_slam.yaml')
    arguments = [
        DeclareLaunchArgument('config_file', default_value=config_file),
        DeclareLaunchArgument('global_slam_config', default_value=global_slam_config),
        DeclareLaunchArgument('global_map_enabled', default_value='true'),
        DeclareLaunchArgument('global_navigation_enabled', default_value='true'),
        DeclareLaunchArgument('local_navigation_enabled', default_value='true'),
        *(DeclareLaunchArgument(name, default_value=UNSET)
          for name in BRIDGE_OVERRIDES),
    ]
    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=acquire_bringup_lock),
        OpaqueFunction(function=create_bridge_node),
        OpaqueFunction(
            function=create_local_navigation_node,
            condition=IfCondition(LaunchConfiguration('local_navigation_enabled')),
        ),
        Node(
            package='car_navigation',
            executable='global_map_node.py',
            name='global_map_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('global_map_enabled')),
            parameters=[LaunchConfiguration('global_slam_config')],
        ),
        Node(
            package='car_navigation',
            executable='jetson_global_navigation_node.py',
            name='jetson_global_navigation',
            output='screen',
            condition=IfCondition(LaunchConfiguration('global_navigation_enabled')),
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            condition=IfCondition(LaunchConfiguration('global_map_enabled')),
            parameters=[LaunchConfiguration('global_slam_config')],
        ),
        Node(
            package='car_mission',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
        Node(
            package='car_mission',
            executable='mission_control_node',
            name='mission_control_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])

"""启动无人机本机协同任务；PX4、MAVROS、VIO 源及深度驱动单独启动。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = Path(get_package_share_directory('uav_bringup')) / 'launch'
    control_config = Path(get_package_share_directory('uav_control')) / 'config' / 'control.yaml'
    mission_config = Path(get_package_share_directory('uav_mission')) / 'config' / 'mission.yaml'
    arguments = [
        DeclareLaunchArgument('depth_topic'),
        DeclareLaunchArgument('camera_frame'),
        DeclareLaunchArgument('fx'),
        DeclareLaunchArgument('fy'),
        DeclareLaunchArgument('cx'),
        DeclareLaunchArgument('cy'),
        DeclareLaunchArgument('depth_scaling_factor'),
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),
        DeclareLaunchArgument('body_frame', default_value='base_link'),
        DeclareLaunchArgument('robot_id', default_value='uav_01'),
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('max_vel', default_value='1.0'),
        DeclareLaunchArgument('max_acc', default_value='1.0'),
        DeclareLaunchArgument('planning_horizon', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_vio_bridge', default_value='true'),
    ]
    planner_arguments = {
        name: LaunchConfiguration(name) for name in (
            'depth_topic', 'camera_frame', 'fx', 'fy', 'cx', 'cy', 'depth_scaling_factor',
            'odom_topic', 'body_frame', 'drone_id', 'max_vel', 'max_acc',
            'planning_horizon', 'use_sim_time')
    }
    # 当前控制器和任务状态机均以 MAVROS 本地 map 为飞行控制坐标系。
    planner_arguments['planning_frame'] = 'map'
    ros_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        *arguments,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'vio.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_vio_bridge')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'ego_swarm.launch.py')),
            launch_arguments=planner_arguments.items(),
        ),
        Node(package='uav_control', executable='mavros_interface_node',
             name='mavros_interface', output='screen',
             parameters=[str(control_config), {'use_sim_time': ros_time}]),
        Node(package='uav_mission', executable='relay_mission_node',
             name='uav_mission', output='screen',
             parameters=[str(mission_config), {
                 'relay_robot_id': ParameterValue(LaunchConfiguration('robot_id'), value_type=str),
                 'relay_frame': 'map', 'use_sim_time': ros_time,
             }]),
    ])

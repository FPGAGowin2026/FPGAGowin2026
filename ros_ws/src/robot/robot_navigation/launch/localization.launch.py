import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def static_transform(child_frame, x, y, z, yaw='0.0'):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{child_frame}_static_tf',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--yaw', yaw, '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_footprint', '--child-frame-id', child_frame,
        ],
    )


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_navigation'), 'config', 'ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.0'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        # 默认零外参只服务主机仿真；实机启动必须按测量值覆盖 IMU 和激光位姿。
        static_transform('base_link', '0.0', '0.0', '0.0'),
        static_transform(
            'imu_link', LaunchConfiguration('imu_x'), LaunchConfiguration('imu_y'),
            LaunchConfiguration('imu_z')),
        static_transform(
            'laser_link', LaunchConfiguration('laser_x'), LaunchConfiguration('laser_y'),
            LaunchConfiguration('laser_z'), LaunchConfiguration('laser_yaw')),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])

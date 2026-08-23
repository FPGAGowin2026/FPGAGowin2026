import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    navigation_share = get_package_share_directory('robot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Nav2 Humble 的 bringup 使用 PythonExpression(['not ', slam])，布尔字面量必须可被 Python 求值。
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_share, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # Nav2 Humble 的官方参数已经覆盖完整插件集合；这里只改接本项目融合里程计，避免复制一份易漂移配置。
        GroupAction([
            SetRemap(src='/odom', dst='/odometry/filtered'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': LaunchConfiguration('slam'),
                    'map': LaunchConfiguration('map'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': 'False',
                }.items(),
            ),
        ]),
    ])

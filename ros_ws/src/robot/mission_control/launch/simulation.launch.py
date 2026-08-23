import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        include('robot_navigation', 'simulation.launch.py'),
        include('vision_bridge', 'simulation.launch.py'),
        # 等 SLAM 首张地图和 Nav2 生命周期节点激活后，再自动提交搜索区目标。
        TimerAction(period=5.0, actions=[
            include('mission_control', 'mission.launch.py', {'autostart': 'true'}),
        ]),
    ])

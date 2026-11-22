"""Reuse car bringup for telemetry, video, SLAM and the camera mount."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = [FindPackageShare('car_bringup'), 'launch']
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason='A mapping component exited'))])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(launch_dir + ['car_bringup.launch.py'])),
            launch_arguments={'enable_command': 'false', 'enable_mission': 'false',
                              'enable_vision': 'false', 'global_map_enabled': 'true'}.items()),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='vision_bridge', executable='synthetic_camera_node'),
        Node(package='vision_bridge', executable='red_target_node'),
    ])

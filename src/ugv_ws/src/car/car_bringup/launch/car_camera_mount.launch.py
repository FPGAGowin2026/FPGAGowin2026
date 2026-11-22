"""Publish the car camera's measured approximate mounting transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='car_camera_mount',
             arguments=['--x', '-0.055', '--y', '0.0', '--z', '0.215',
                        # Upward 10 degrees: body Ry(-10 deg) times the optical-axis rotation.
                        '--qx', '-0.4545194776720437', '--qy', '0.4545194776720437',
                        '--qz', '-0.5416752204197018', '--qw', '0.5416752204197018',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera']),
    ])

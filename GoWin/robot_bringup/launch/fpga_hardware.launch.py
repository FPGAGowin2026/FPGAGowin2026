from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # YAML 提供共享的标定与安全默认值；launch 参数只覆盖部署时必须变化的网卡端点。
    config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'fpga_gateway.yaml')

    return LaunchDescription([
        # 该入口只启动真实网关，不启动 fake 节点；地址和端口须与 FPGA 实板一致。
        DeclareLaunchArgument('fpga_ip', default_value='192.168.50.2'),
        DeclareLaunchArgument('control_port', default_value='5000'),
        DeclareLaunchArgument('telemetry_port', default_value='5001'),
        Node(
            package='fpga_gateway',
            executable='fpga_gateway_node',
            parameters=[config, {
                'fpga_ip': LaunchConfiguration('fpga_ip'),
                'control_port': ParameterValue(
                    LaunchConfiguration('control_port'), value_type=int),
                'telemetry_port': ParameterValue(
                    LaunchConfiguration('telemetry_port'), value_type=int),
            }],
        ),
    ])

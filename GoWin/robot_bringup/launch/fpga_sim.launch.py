from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 两个节点共用 YAML 的运动学和安全参数，再以本机回环端点覆盖硬件网络配置。
    config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'fpga_gateway.yaml')
    control_port = ParameterValue(
        LaunchConfiguration('control_port'), value_type=int)
    telemetry_port = ParameterValue(
        LaunchConfiguration('telemetry_port'), value_type=int)
    # 网关与 fake 节点在同一主机上通过 UDP 互通，端口可覆盖以支持并行仿真。
    gateway_endpoint = {
        'fpga_ip': '127.0.0.1',
        'bind_ip': '127.0.0.1',
        'control_port': control_port,
        'telemetry_port': telemetry_port,
    }
    simulator_endpoint = {
        'listen_ip': '127.0.0.1',
        'jetson_ip': '127.0.0.1',
        'control_port': control_port,
        'telemetry_port': telemetry_port,
    }

    return LaunchDescription([
        # 此 launch 只覆盖 ROS/UDP 主机闭环，不产生 FPGA 或电机硬件证据。
        DeclareLaunchArgument('control_port', default_value='15000'),
        DeclareLaunchArgument('telemetry_port', default_value='15001'),
        Node(
            package='fpga_gateway', executable='fpga_gateway_node',
            parameters=[config, gateway_endpoint]),
        Node(
            package='fpga_gateway', executable='fake_fpga_node',
            parameters=[config, simulator_endpoint]),
    ])

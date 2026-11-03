import importlib.util
from pathlib import Path

import pytest
import rclpy
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from rclpy.context import Context


LAUNCH_FILE = (Path(__file__).resolve().parents[1] / 'launch' /
               'car_bringup.launch.py')
CONFIG_FILE = Path(__file__).resolve().parents[1] / 'config' / 'bringup.yaml'


@pytest.fixture
def launch_module(monkeypatch):
    spec = importlib.util.spec_from_file_location('car_bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: f'/share/{package}')
    return module


def make_context(module, launch_args=None):
    context = LaunchContext()
    context.launch_configurations.update(launch_args or {})
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def capture_bridge_node(module, monkeypatch, context):
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.append(kwargs) or kwargs)
    module.create_bridge_node(context)
    assert len(captured) == 1
    return captured[0]


def test_omitted_network_arguments_leave_configuration_unmodified(
        launch_module, monkeypatch):
    config_file = '/tmp/custom_bringup.yaml'
    context = make_context(launch_module, {'config_file': config_file})
    node = capture_bridge_node(launch_module, monkeypatch, context)

    parameters = node['parameters']
    assert parameters[0].perform(context) == config_file
    assert parameters[1] == {}


def test_explicit_bind_and_ports_are_mapped_with_types(launch_module, monkeypatch):
    context = make_context(launch_module, {
        'bind_ip': '192.168.0.4',
        'control_port': '5100',
        'reply_port': '6109',
    })
    node = capture_bridge_node(launch_module, monkeypatch, context)

    overrides = {
        name: value.evaluate(context)
        for name, value in node['parameters'][1].items()
    }
    assert overrides == {
        'bind_fpga': '192.168.0.4',
        'fpga_port': 5100,
        'ack_port': 6109,
    }
    assert type(overrides['bind_fpga']) is str
    assert type(overrides['fpga_port']) is int
    assert type(overrides['ack_port']) is int


def test_explicit_empty_pc_ip_remains_a_string(launch_module, monkeypatch):
    context = make_context(launch_module, {'pc_ip': ''})
    node = capture_bridge_node(launch_module, monkeypatch, context)

    pc_ip = node['parameters'][1]['pc_ip'].evaluate(context)
    assert pc_ip == ''
    assert type(pc_ip) is str


def test_mapping_uses_navigation_package_and_its_own_configuration(launch_module, monkeypatch):
    context = make_context(launch_module, {'config_file': '/tmp/bringup.yaml'})
    captured = []

    def capture_node(**kwargs):
        captured.append(kwargs)
        return launch_module.OpaqueFunction(function=lambda _: [])

    monkeypatch.setattr(launch_module, 'Node', capture_node)
    launch_module.generate_launch_description()
    mapping = next(node for node in captured if node['name'] == 'global_map_node')
    coordinator = next(node for node in captured if node['name'] == 'jetson_global_navigation')
    slam = next(node for node in captured if node['name'] == 'slam_toolbox')
    assert mapping['package'] == 'car_navigation'
    assert mapping['executable'] == 'global_map_node.py'
    assert mapping['parameters'][0].perform(context) == '/share/car_navigation/config/global_slam.yaml'
    assert coordinator['package'] == 'car_navigation'
    assert coordinator['executable'] == 'jetson_global_navigation_node.py'
    assert slam['parameters'][0].perform(context) == '/share/car_navigation/config/global_slam.yaml'


def test_local_navigation_reuses_bridge_frames_and_extrinsics(launch_module, monkeypatch, tmp_path):
    config = tmp_path / 'bringup.yaml'
    config.write_text('''ugv_bridge_node:
  ros__parameters:
    odom_frame_id: custom_odom
    base_frame_id: custom_base
    n10_frame_id: custom_lidar
    lidar_x_m: 0.13
    lidar_y_m: 0.0
    lidar_yaw_rad: 0.0
''', encoding='utf-8')
    context = make_context(launch_module, {'config_file': str(config)})
    captured = []
    monkeypatch.setattr(launch_module, 'Node', lambda **kwargs: captured.append(kwargs) or kwargs)
    launch_module.create_local_navigation_node(context)
    assert len(captured) == 1
    assert captured[0]['package'] == 'car_navigation'
    assert captured[0]['executable'] == 'jetson_local_navigation'
    assert captured[0]['parameters'][1] == {
        'odom_frame_id': 'custom_odom', 'base_frame_id': 'custom_base',
        'lidar_frame_id': 'custom_lidar', 'lidar_x_m': 0.13,
        'lidar_y_m': 0.0, 'lidar_yaw_rad': 0.0,
    }


def test_bringup_config_supplies_motion_defaults(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context, domain_id=193)
    nodes = []
    try:
        for name in ('ugv_bridge_node', 'mission_control_node'):
            nodes.append(rclpy.create_node(
                name, context=context, use_global_arguments=False,
                cli_args=['--ros-args', '--params-file', str(CONFIG_FILE)]))
        bridge, mission = nodes
        assert bridge.declare_parameter('navigation_default_speed_mm_s', 100).value == 400
        assert bridge.declare_parameter('lidar_x_m', 0.0).value == 0.13
        assert bridge.declare_parameter('lidar_y_m', 1.0).value == 0.0
        assert bridge.declare_parameter('lidar_yaw_rad', 1.0).value == 0.0
        assert mission.declare_parameter('linear_speed_m_s', 0.1).value == 0.4
        assert mission.declare_parameter('search_angular_speed', 0.1).value == 0.8
        assert mission.declare_parameter('max_angular_speed', 0.1).value == 0.3
        assert mission.declare_parameter('observation_timeout_s', 0.1).value == 0.25
        assert bridge.declare_parameter('cmd_vel_timeout_s', 0.1).value == 0.2
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown(context=context)

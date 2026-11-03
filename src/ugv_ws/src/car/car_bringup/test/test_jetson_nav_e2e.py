"""A real bridge and Jetson planner connected to simulated sensors and UDP FPGA."""
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import time

import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


spec = importlib.util.spec_from_file_location(
    'bridge_fixture', Path(__file__).with_name('test_bx72_bidirectional.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
fixture.TEST_ENV.update(
    ROS_DOMAIN_ID=os.environ.get('UGV_TEST_ROS_DOMAIN_ID', str(140 + os.getpid() % 40)),
    ROS_LOCALHOST_ONLY='1')


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.generate_test_description()
    # Match the velocity-only FPGA kind8 contract: pose valid, self-test passed.
    fixture.TEST_FAKE.navigation['flags'] = 0x16
    node = launch_ros.actions.Node(
        package='car_navigation', executable='jetson_local_navigation',
        additional_env=fixture.TEST_ENV, output='screen')
    description.add_action(node)
    context['jetson_navigation_process'] = node
    return description, context


class TestJetsonNavigationEndToEnd(fixture.TestBx72Bidirectional):
    def setUp(self):
        super().setUp()
        self.status = None
        self.grid = None
        self.states = []
        self.count = 0
        self.clock_anchor = None
        self.fusion_pub = self.node.create_publisher(
            String, '/fpga/round5/fusion', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(
            Odometry, '/odom/filtered', qos_profile_sensor_data)
        self.scan_pub = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.node.create_subscription(String, '/jetson/navigation/state',
                                      lambda msg: self.states.append(json.loads(msg.data)), 10)
        self.node.create_subscription(String, '/jetson/navigation/local_grid', self.on_grid, 10)
        self.node.create_subscription(String, '/fpga/navigation_state',
                                      self.on_navigation_clock, qos_profile_sensor_data)
        self.node.create_timer(0.02, self.publish_sensors)

    def on_grid(self, msg):
        self.grid = json.loads(msg.data)

    def on_navigation_clock(self, msg):
        nav = json.loads(msg.data)
        boot, domain = nav.get('boot_id'), nav.get('odom_domain')
        if boot and domain and (self.clock_anchor is None or
                                self.clock_anchor[:2] != (boot, domain)):
            self.clock_anchor = (boot, domain, nav['now_us'], time.monotonic_ns())

    def collect_mission_results(self):
        while True:
            try:
                packet = self.pc.recvfrom(65535)[0]
            except BlockingIOError:
                return
            try:
                message = json.loads(packet)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if message.get('type') == 'mission_result':
                self.mission_results[message.get('request_id')] = message
            elif message.get('type') == 'status':
                self.status = message

    def publish_sensors(self):
        if self.clock_anchor is None:
            return
        # Derive simulated fusion stamps from the same received FPGA clock
        # anchor as the planner. Reading FakeFpga's next unsent clock sample
        # races the UDP/ROS path and can put simulated fusion in the future.
        _, _, origin_us, origin_ns = self.clock_anchor
        source_us = (origin_us + (time.monotonic_ns() - origin_ns) // 1000 - 30000) & 0xffffffff
        fusion = {'pose_valid': True, 'pose_stamp_us': source_us, 'scan_match_accepted': 0,
                  'state': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                            'vx_m_s': 0.0, 'vy_m_s': 0.0, 'wz_rad_s': 0.0}}
        self.fusion_pub.publish(String(data=json.dumps(fusion)))
        stamp = self.node.get_clock().now()
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp = stamp.to_msg()
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.count += 1
        if self.count % 5:
            return
        scan = LaserScan()
        scan.header.frame_id = 'n10_sensor'
        scan.header.stamp = Time(nanoseconds=stamp.nanoseconds - 40_000_000).to_msg()
        scan.angle_min, scan.angle_increment = 0.0, math.tau / 72.0
        scan.angle_max = scan.angle_increment * 71
        scan.scan_time, scan.range_min, scan.range_max = 0.1, 0.05, 8.0
        scan.ranges = [6.0 if i < 18 or i > 54 else 0.265 for i in range(72)]
        self.scan_pub.publish(scan)

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('protocol') == 'velocity_only_v1' and
            self.latest_navigation().get('boot_id', 0) != 0 and
            self.status is not None and self.status.get('ready') and
            self.status.get('auto_acknowledged') and self.grid is not None and
            self.grid.get('scan_generation', 0) > 0 and (self.grid.get('flags', 0) & 6) == 6,
            timeout=10.0), 'bridge or local grid not ready: '
            f'status={self.status}, grid_meta={None if self.grid is None else {k: self.grid.get(k) for k in ("boot_id", "flags", "scan_generation", "grid_frame_epoch")}}')

        request = self.send_mission('local_start', task_id=1, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=2.0, goal_y_m=0.0,
                                    position_only=True, duration_s=20.0)
        result = self.wait_for_mission_result(request)
        self.assertTrue(result and result['success'], f'local_start failed: {result}')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: any(packet[5] == 3 and struct.unpack_from('!h', packet, 20)[0] > 0
                        for _, packet in self.fake.snapshot()), timeout=6.0),
            'the real Jetson planner must drive a nonzero FPGA type3 speed through the bridge')

        before_cancel = len(self.fake.snapshot())
        request = self.send_mission('local_cancel', task_id=2)
        result = self.wait_for_mission_result(request)
        self.assertTrue(result and result['success'], f'local_cancel failed: {result}')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: any(packet[5] == 3 and packet[20:26] == b'\x00' * 6
                        for _, packet in self.fake.snapshot()[before_cancel:]) and
            any(state.get('task_id') == 1 and state.get('last_task_id') == 2 and
                state.get('status') == 7 and not state.get('active_task') for state in self.states),
            timeout=4.0), 'cancel must stop the FPGA and terminate the real Jetson task')
        self.spin_with_monitor_heartbeat(lambda: False, timeout=0.5)
        events = self.fake.snapshot()[before_cancel:]
        zero_at = next(at for at, packet in events
                       if packet[5] == 3 and packet[20:26] == b'\x00' * 6)
        self.assertFalse(any(at > zero_at + 0.1 and packet[5] == 3 and
                             packet[20:26] != b'\x00' * 6 for at, packet in events),
                         'the canceled task must not resume motion')
        self.assertTrue(any(packet[5] == 5 and packet[20] == 0 for _, packet in events))
        self.assertFalse(any(packet[5] == 7 for _, packet in self.fake.snapshot()))


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.TestCleanExit):
    def test_jetson_exit_code(self, proc_info, jetson_navigation_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=jetson_navigation_process)

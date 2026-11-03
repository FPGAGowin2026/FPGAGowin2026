"""协同任务失去网页心跳时，验证 Jetson 服务、网桥及假 FPGA 的完整停车链路。"""
import importlib.util
import json
import os
from pathlib import Path
import struct
import time

import launch_testing
import pytest
import rclpy
from std_msgs.msg import String
from ugv_bridge.msg import AuthorizedTwist
from ugv_bridge.srv import NavigationCommand


spec = importlib.util.spec_from_file_location(
    'bridge_fixture', Path(__file__).with_name('test_bx72_bidirectional.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
fixture.TEST_ENV.update(ROS_DOMAIN_ID=os.environ.get('UGV_TEST_ROS_DOMAIN_ID', '212'),
                        ROS_LOCALHOST_ONLY='1')


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.generate_test_description()
    fixture.TEST_FAKE.navigation['flags'] = 0x16  # velocity_only_v1
    return description, context


class TestNavigationLeaseStop(fixture.TestBx72Bidirectional):
    def setUp(self):
        super().setUp()
        self.test_navigation = None
        self.test_status = None
        self.nav_active = False
        self.nav_task = self.nav_last = self.nav_epoch = 0
        self.nav_bridge = ''
        self.nav_status = 0
        self.nav_mode = 1
        self.nav_actions = []
        self.nav_service = self.node.create_service(
            NavigationCommand, '/jetson/navigation/command', self.on_navigation)
        self.nav_state = self.node.create_publisher(String, '/jetson/navigation/state', 10)
        self.nav_velocity = self.node.create_publisher(
            AuthorizedTwist, '/jetson/navigation/cmd_vel', 10)
        self.nav_timer = self.node.create_timer(0.04, self.publish_navigation)

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
                self.test_status = message
            elif message.get('type') == 'telemetry' and message.get('stream') == 'navigation':
                self.test_navigation = message.get('data')

    def latest_navigation(self):
        return self.test_navigation

    def on_navigation(self, request, response):
        self.nav_actions.append((request.action, request.task_id))
        if request.action == 1:
            self.nav_task = self.nav_last = request.task_id
            self.nav_bridge = request.bridge_id
            self.nav_epoch = request.authorization_epoch
            self.nav_status = 3
            self.nav_mode = request.mode
            self.nav_active = True
        else:
            self.nav_active = False
            if request.task_id:
                self.nav_last = request.task_id
            if self.nav_task:
                self.nav_status = 7
        response.success = True
        response.message = '已接收'
        return response

    def publish_navigation(self):
        report = self.fake.navigation
        state = String()
        state.data = json.dumps({'bridge_id': self.nav_bridge,
            'authorization_epoch': self.nav_epoch,
            'boot_id': report['boot_id'], 'odom_domain': report['odom_domain'],
            'task_id': self.nav_task, 'last_task_id': self.nav_last,
            'status': self.nav_status, 'mode': self.nav_mode if self.nav_active else 1,
            'progress': 0, 'active_task': self.nav_active})
        self.nav_state.publish(state)
        if self.nav_active:
            velocity = AuthorizedTwist()
            velocity.header.stamp = self.node.get_clock().now().to_msg()
            velocity.header.frame_id = 'base_link'
            velocity.bridge_id = self.nav_bridge
            velocity.authorization_epoch = self.nav_epoch
            velocity.twist.linear.x = 0.12
            self.nav_velocity.publish(velocity)

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('boot_id', 0) != 0 and
            self.test_status is not None and self.test_status.get('ready') and
            self.test_status.get('auto_acknowledged') and
            ('ugv_bridge_node', '/') in self.node.get_node_names_and_namespaces() and
            any(name == '/jetson/navigation/command' for name, _ in
                self.node.get_client_names_and_types_by_node('ugv_bridge_node', '/')), timeout=8.0))
        request = self.send_mission('local_start', task_id=1, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=0.5, goal_y_m=0.0,
                                    duration_s=10.0)
        result = self.wait_for_mission_result(request)
        self.assertTrue(result and result['success'], f'Jetson 任务启动失败: {result}')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: any(packet[5] == 3 and struct.unpack_from('!h', packet, 20)[0] > 0
                        for _, packet in self.fake.snapshot()), timeout=5.0),
            'Jetson 速度应经网桥转为 FPGA type3 命令')
        events_before = len(self.fake.snapshot())
        self.fake.reorder_stop_acks.set()
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.collect_mission_results()
        sent = [packet for _, packet in self.fake.snapshot()[events_before:]]
        stop_index = next((i for i, packet in enumerate(sent)
                           if packet[5] == 3 and packet[20:26] == b'\x00' * 6), None)
        self.assertIsNotNone(stop_index, '心跳丢失后必须发零速')
        self.assertTrue(any(packet[5] == 5 and packet[20] == 0 for packet in sent[stop_index:]))
        self.assertFalse(any(packet[5] == 3 and packet[20:26] != b'\x00' * 6
                             for packet in sent[stop_index:]), '旧任务不能恢复速度')
        self.assertFalse(any(packet[5] == 7 for packet in sent), 'FPGA 不再接收导航分片')
        self.assertFalse(self.nav_active)
        self.assertIn((3, 0), self.nav_actions, '同时取消 Jetson 内的旧任务')
        self.assertTrue(self.fake.reordered_stop_acks.is_set(), '假 FPGA 须先回复模式 ACK 再回复零速 ACK')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('task_id') == 1 and
            self.latest_navigation().get('active_task') is False, timeout=2.0),
            '双 ACK 乱序后仍须等到新鲜零速遥测，然后解除停车屏障')
        self.fake.reorder_stop_acks.clear()
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.test_status is not None and self.test_status.get('ready') and
            self.test_status.get('auto_acknowledged'), timeout=3.0),
            '停车确认后控制链路应恢复')
        request = self.send_mission('local_start', task_id=2, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=0.5, goal_y_m=0.0,
                                    duration_s=10.0)
        result = self.wait_for_mission_result(request)
        self.assertTrue(result and result['success'], f'乱序停车后新任务应可启动: {result}')


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.TestCleanExit):
    pass

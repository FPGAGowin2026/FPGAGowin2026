"""Web UDP monitoring and commands through the real mission and bridge nodes."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import time
import unittest

import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


spec = importlib.util.spec_from_file_location(
    'ros_control_fixture', Path(__file__).with_name('test_ros_control.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
fixture.TEST_ENV.update(ROS_DOMAIN_ID=os.environ.get('UGV_TEST_ROS_DOMAIN_ID',
                                                   str(150 + os.getpid() % 40)),
                        ROS_LOCALHOST_ONLY='1')


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.generate_test_description()
    description.add_action(launch_ros.actions.Node(
        package='car_mission', executable='mission_control_node',
        parameters=[{'motion_enabled': True, 'target_id': 'fixture:readiness-never-published'}],
        additional_env=fixture.TEST_ENV, output='screen'))
    return description, context


class TestWebMission(unittest.TestCase):
    def setUp(self):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(domain_id=int(fixture.TEST_ENV['ROS_DOMAIN_ID']))
        self.node = rclpy.create_node('test_web_mission')
        self.publisher = self.node.create_publisher(Detection2DArray, '/vision/detections', 1)
        self.scan_publisher = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.diagnostics = []
        self.node.create_subscription(DiagnosticArray, '/fpga/bidirectional_status',
                                      self.diagnostics.append, 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.setblocking(False)
        self.sequence = 0
        self.messages = []
        self.monitor = True
        self.stream = True
        self.detect_target = True
        self.target_x = 100.0
        self.last_send = 0
        self.safety_nonce = None

    def tearDown(self):
        self.socket.close()
        fixture.TEST_FAKE.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def send(self, message):
        message.update(v=1, session='web-mission-test', seq=self.sequence)
        self.sequence += 1
        self.socket.sendto(json.dumps(message).encode(), ('127.0.0.1', fixture.TEST_PORTS['pc']))

    def observe(self):
        if not self.stream:
            return
        array = Detection2DArray()
        array.header.stamp = self.node.get_clock().now().to_msg()
        array.header.frame_id = 'camera'
        scan = LaserScan()
        scan.header.stamp = array.header.stamp
        scan.header.frame_id = 'laser'
        scan.angle_increment = 0.1
        scan.range_min, scan.range_max, scan.ranges = 0.05, 8.0, [1.0]
        self.scan_publisher.publish(scan)
        if not self.detect_target:
            self.publisher.publish(array)
            return
        detection = Detection2D()
        detection.header = array.header
        detection.id = 'web-session:1'
        detection.bbox.center.position.x = self.target_x
        detection.bbox.center.position.y = 240.0
        detection.bbox.size_x = 100.0
        detection.bbox.size_y = 180.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = '0'
        hypothesis.hypothesis.score = 0.95
        detection.results = [hypothesis]
        array.detections = [detection]
        self.publisher.publish(array)

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if time.monotonic() - self.last_send >= .05:
                if self.monitor:
                    self.send({'type': 'control', 'take_control': False,
                               'vx': 0, 'vy': 0, 'wz': 0, 'auto': False})
                self.observe()
                self.last_send = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=.01)
            while True:
                try:
                    packet = self.socket.recvfrom(65535)[0]
                except BlockingIOError:
                    break
                if packet.startswith(b'{'):
                    message = json.loads(packet)
                    if message.get('type') == 'vision':
                        self.assertLessEqual(len(packet), 1400)
                    self.messages.append(message)
            if predicate():
                return True
        return False

    def command(self, action, **fields):
        if action in ('enable_ros', 'start'):
            self.assertTrue(self.spin_until(self.ready_for_enable))
        def latest_nonce():
            return next((message.get('claim_nonce') for message in reversed(self.messages)
                         if message.get('type') == 'status' and message.get('claim_nonce')), None)
        if action not in ('stop', 'local_stop', 'local_cancel', 'emergency_stop', 'disable_ros'):
            self.assertTrue(self.spin_until(lambda: latest_nonce() and latest_nonce() != self.safety_nonce))
            fields['claim_nonce'] = latest_nonce()
        else:
            self.safety_nonce = latest_nonce()
        request_id = f'{action}-{self.sequence}'
        self.send({'type': 'mission_command', 'action': action, 'request_id': request_id, **fields})
        self.assertTrue(self.spin_until(lambda: any(
            message.get('type') == 'mission_result' and message.get('request_id') == request_id
            for message in self.messages)))
        return next(message for message in self.messages
                    if message.get('type') == 'mission_result' and message.get('request_id') == request_id)

    def assert_command(self, action, expected_success=True, **fields):
        result = self.command(action, **fields)
        self.assertEqual(result['success'], expected_success, f'mission_result={result!r}')
        return result

    def ros_authorized(self):
        for array in reversed(self.diagnostics):
            for status in array.status:
                if status.name == 'bx72_udp':
                    return dict((item.key, item.value) for item in status.values).get('ros_authorized') == 'True'
        return False

    def ready_for_enable(self):
        for array in reversed(self.diagnostics):
            for status in array.status:
                if status.name == 'bx72_udp':
                    values = dict((item.key, item.value) for item in status.values)
                    return values.get('emergency_stop') == 'True' or (
                        values.get('mode_ready') == 'True' and values.get('control_source') in ('none', 'pc'))
        return False

    def velocity_seen(self, expected, start):
        return any(packet[5] == 3 and struct.unpack('!hhh', packet[20:26]) == expected
                   for _, packet in fixture.TEST_FAKE.snapshot()[start:])

    def wait_mission_ready_with_empty_observations(self):
        # /mission/start checks authorization and fresh observations before
        # target selection. Empty frames and the impossible target_id guarantee
        # that this readiness probe cannot intentionally start a task.
        self.assertFalse(self.detect_target)
        probe = self.node.create_client(Trigger, '/mission/start')
        stop = self.node.create_client(Trigger, '/mission/stop')
        deadline = time.monotonic() + 5.0
        self.assertTrue(self.spin_until(
            lambda: probe.service_is_ready() and self.ready_for_enable(),
            timeout=max(0.0, deadline - time.monotonic())))
        waiting = {
            '缺少新鲜的网桥授权状态或急停激活',
            '没有新鲜视觉观测；请等待新帧后重试',
        }
        last_message = ''
        while time.monotonic() < deadline:
            diagnostic_count = len(self.diagnostics)
            future = probe.call_async(Trigger.Request())
            self.assertTrue(self.spin_until(
                future.done, timeout=max(0.0, deadline - time.monotonic())))
            response = future.result()
            if response.success:
                self.spin_until(stop.service_is_ready, timeout=1.0)
                stop_future = stop.call_async(Trigger.Request())
                self.spin_until(stop_future.done, timeout=1.0)
                self.fail('an empty-frame readiness probe unexpectedly started a task')
            last_message = response.message
            if last_message == '新鲜观测中没有指定 target_id 的合法目标':
                return
            self.assertIn(last_message, waiting)
            self.assertTrue(self.spin_until(
                lambda: len(self.diagnostics) > diagnostic_count,
                timeout=max(0.0, deadline - time.monotonic())))
        self.fail(f'mission did not become ready: {last_message}')

    def test_monitoring_commands_and_visual_turn_reach_fpga(self):
        self.detect_target = False
        self.assertTrue(self.spin_until(lambda: any(
            message.get('type') == 'vision' and not message['stale'] for message in self.messages)))
        self.wait_mission_ready_with_empty_observations()
        self.detect_target = True
        self.assert_command('start', target_id='web-session:1', class_id='0',
                            behavior='follow', color='')
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 300), start)))
        self.assertTrue(self.spin_until(self.ros_authorized), 'monitor heartbeats must not preempt ROS')
        self.assertTrue(self.spin_until(lambda: any(
            message.get('type') == 'vision' and message['mission_state'] == 'FOLLOWING' and
            message['selected_target'] and message['selected_target']['id'] == 'web-session:1'
            for message in self.messages)))
        self.target_x = 320.0
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((400, 0, 0), start)))
        self.target_x = 100.0

        # A short input gap stops an authorized search, and only fresh frames
        # under that same authorization can resume it.
        self.assert_command('stop')
        self.detect_target = False
        self.assert_command('start', class_id='0', behavior='search_only', color='')
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 800), start)))
        self.stream = False
        lost_message_start = len(self.messages)
        self.assertTrue(self.spin_until(lambda: any(
            message.get('type') == 'vision' and message['mission_state'] == 'LOST'
            for message in self.messages[lost_message_start:])))
        lost_packets = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 0), lost_packets)))
        self.stream = True
        resumed_message_start = len(self.messages)
        resumed_packets = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: any(
            message.get('type') == 'vision' and message['mission_state'] == 'SEARCHING'
            for message in self.messages[resumed_message_start:]) and
            self.velocity_seen((0, 0, 800), resumed_packets)))
        self.assert_command('stop')
        self.detect_target = True
        self.assert_command('start', target_id='web-session:1', class_id='0',
                            behavior='follow', color='')
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 300), start)))

        self.assert_command('local_start', task_id=1, mode='cooperative',
                            goal_x_m=1.0, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_until(lambda: not self.ros_authorized()),
                        'local_start must take over the ROS source in the same request')
        self.assert_command('local_cancel', task_id=2)
        self.assert_command('stop')
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 0), start)))
        self.assert_command('start')
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 300), start)))
        self.assertTrue(self.spin_until(self.ros_authorized))
        self.monitor = False
        # ROS observations continue; only the website heartbeat disappears.
        self.assertTrue(self.spin_until(lambda: not self.ros_authorized(), timeout=2.0))
        start = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 0), start)))
        self.monitor = True
        self.assert_command('enable_ros')
        reauthorized_at = len(fixture.TEST_FAKE.snapshot())
        self.spin_until(lambda: False, timeout=.35)
        self.assertFalse(any(packet[5] == 3 and struct.unpack('!hhh', packet[20:26]) != (0, 0, 0)
                             for _, packet in fixture.TEST_FAKE.snapshot()[reauthorized_at:]),
                         'a new ROS authorization must not revive the old mission')
        self.assert_command('emergency_stop')
        self.assert_command('enable_ros', expected_success=False)
        self.assert_command('release_emergency_stop')
        self.assertFalse(self.ros_authorized(), 'release must not restore ROS authorization')
        self.assertTrue(self.spin_until(self.ready_for_enable))
        self.send({'type': 'mission_command', 'action': 'start', 'request_id': 'start-then-stop',
                   'target_id': 'web-session:1', 'class_id': '0', 'behavior': 'follow', 'color': ''})
        self.send({'type': 'mission_command', 'action': 'stop', 'request_id': 'cancel-pending-start'})
        self.assertTrue(self.spin_until(lambda: all(any(
            message.get('type') == 'mission_result' and message.get('request_id') == request_id
            for message in self.messages) for request_id in ('start-then-stop', 'cancel-pending-start'))))
        stopped_at = len(fixture.TEST_FAKE.snapshot())
        self.assertTrue(self.spin_until(lambda: self.velocity_seen((0, 0, 0), stopped_at)))
        self.assertFalse(self.spin_until(self.ros_authorized, timeout=.6),
                         'a completed start response must not restore control after stop')


@launch_testing.post_shutdown_test()
class TestCleanExit(unittest.TestCase):
    def test_exit_code(self, proc_info, ugv_bridge_process):
        launch_testing.asserts.assertExitCodes(proc_info)

"""Loopback integration for real control ACKs, ROS image, and raw telemetry."""
import binascii
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import struct
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from ugv_bridge.msg import Bx72ImuRaw


TEST_ENV = {'ROS_DOMAIN_ID': str(32 + os.getpid() % 60)}
TEST_PORTS = {}
TEST_FAKE = None
PACKET = struct.Struct('!4sBBHIQ10sH')

_protocol_spec = importlib.util.spec_from_file_location(
    'bx72_test_protocol', Path(__file__).with_name('bx72_test_protocol.py'))
_protocol = importlib.util.module_from_spec(_protocol_spec)
_protocol_spec.loader.exec_module(_protocol)
navigation_packet = _protocol.navigation_packet
telemetry_packet = _protocol.telemetry_packet


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def free_ports(count):
    ports = set()
    while len(ports) < count:
        ports.add(free_port())
    return list(ports)


def encode_ack(request, token=None):
    ack = bytearray(request)
    ack[5] = {1: 2, 3: 4, 5: 6, 7: 8}[request[5]]
    if token is not None:
        struct.pack_into('!Q', ack, 12, token)
    ack[30:32] = struct.pack('!H', binascii.crc_hqx(ack[:30], 0xFFFF))
    return bytes(ack)


def rtp_packets(frame_id, image, sequence_start):
    chunk_size = 1200
    count = math.ceil(len(image) / chunk_size)
    for index in range(count):
        payload = image[index * chunk_size:(index + 1) * chunk_size]
        header = bytearray(12)
        header[0] = 0x80
        header[1] = 97 | (0x80 if index == count - 1 else 0)
        struct.pack_into('!HI', header, 2, (sequence_start + index) & 0xFFFF, frame_id)
        header[8:12] = b'ACG6'
        yield bytes(header) + payload


class FakeFpga:
    def __init__(self, fpga_port, ack_port, telemetry_port, video_port, bootstrap_drops=0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', fpga_port))
        self.sock.settimeout(0.02)
        self.ack_target = ('127.0.0.1', ack_port)
        self.telemetry_target = ('127.0.0.1', telemetry_port)
        self.video_target = ('127.0.0.1', video_port)
        self.events = []
        self.bootstrap_drops = bootstrap_drops
        self.bootstrap_attempts = 0
        self.delayed_navigation = None
        self.delayed_navigation_ticks = 0
        self.manual_requests_during_start_delay = []
        self.wrong_acks = 0
        self.rtp_sequence = 0
        self.last_video_send = None
        self.navigation = {
            'boot_id':0, 'odom_domain':1, 'task_id':0, 'map_version':1,
            'last_task_id':0, 'session_token':0,
            'expiry_us':0, 'status':0, 'error':0, 'mode':1, 'flags':0x86,
            'velocity':[0, 0, 0], 'goal_x_q16':0, 'goal_y_q16':0,
            'planner_expansions':0, 'progress':0, 'map_age_us':0, 'pose_age_us':0,
            'now_us':0,
        }
        self.task_fragments = {}
        self.task_token = None
        self.task_next_index = 0
        self.last_task_fragment = None
        self.reack_task_duplicate = False
        self.drop_task_ack_index = None
        self.drop_task_request_index = None
        self.dropped_task_ack = threading.Event()
        self.delivered_task_fragments = []
        self.accepted_tasks = []
        self.hold_stop_mode = threading.Event()
        self.reorder_stop_acks = threading.Event()
        self.reordered_stop_acks = threading.Event()
        self.held_zero_ack = None
        self.lock = threading.Lock()
        self.valid_ack = threading.Event()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def navigation_task_rejection(self, task):
        if len(task) != 72 or task[0] != 1 or any(task[48:56]) or any(task[64:72]):
            return 1
        action, mode, flags = task[1], task[2], task[3]
        task_id, domain, boot_id = struct.unpack_from('!III', task, 4)
        if (action not in (1, 2, 3) or mode > 3 or flags & ~3 or not task_id or
                domain != self.navigation['odom_domain'] or boot_id != self.navigation['boot_id']):
            return 1
        previous = self.navigation['last_task_id']
        delta = (task_id - previous) & 0xFFFFFFFF
        if (previous and (delta == 0 or delta >= 0x80000000)) or (not previous and task_id == 0):
            return 3
        expiry = struct.unpack_from('!Q', task, 40)[0]
        sent_at = struct.unpack_from('!Q', task, 56)[0]
        now_us = self.navigation['now_us']
        if expiry >> 32 or sent_at >> 32:
            return 2
        sent32 = sent_at & 0xFFFFFFFF
        expiry32 = expiry & 0xFFFFFFFF
        age = (now_us - sent32) & 0xFFFFFFFF
        duration = (expiry32 - sent32) & 0xFFFFFFFF
        if age >= 0x80000000 or age > 500000 or age >= duration or duration > 120000000:
            return 2
        if action == 1:
            speed, accel, tolerance, yaw_tolerance = struct.unpack_from('!HHHH', task, 32)
            hold_yaw, final_yaw = struct.unpack_from('!ii', task, 24)
            if (mode not in (2, 3) or speed < 1 or speed > 400 or accel < 1 or accel > 1000 or
                    tolerance < 1 or tolerance > 500 or yaw_tolerance < 1 or yaw_tolerance > 1000 or
                    not -205887 <= hold_yaw <= 205887 or not -205887 <= final_yaw <= 205887):
                return 2
        return 0

    def _run(self):
        sequence = 1
        next_telemetry = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            if now >= next_telemetry:
                sequence = (sequence + 1) & 0xFFFFFFFF
                now_us = time.monotonic_ns() // 1000
                self.navigation['now_us'] = now_us & 0xFFFFFFFF
                packet = telemetry_packet(sequence, now_us & 0xFFFFFFFF)
                try:
                    self.sock.sendto(packet, self.telemetry_target)
                    report = self.navigation
                    if self.delayed_navigation_ticks:
                        report = dict(self.delayed_navigation, now_us=self.navigation['now_us'])
                        self.delayed_navigation_ticks -= 1
                    self.sock.sendto(navigation_packet(sequence, now_us, report), self.telemetry_target)
                except OSError:
                    return
                next_telemetry = now + 0.1
            try:
                request, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(request) != 32 or request[:4] != b'BX72' or request[4] != 1:
                continue
            with self.lock:
                self.events.append((time.monotonic(), bytes(request)))
            if request[5] not in (1, 3, 5, 7):
                continue
            if request[5] in (3, 5):
                self.last_task_fragment = None
            if self.delayed_navigation_ticks and request[5] in (3, 5):
                self.manual_requests_during_start_delay.append(request[5])
            if request[5] == 5 and request[20] == 0 and self.navigation['status'] in (1, 2, 3, 5, 9):
                if self.hold_stop_mode.is_set():
                    continue
                with self.lock:
                    self.navigation.update(status=7, mode=1, expiry_us=0, velocity=[0, 0, 0])
            if request[5] == 3 and request[20:26] == b'\x00' * 6 and self.navigation['boot_id'] == 0:
                # 模拟接收相位尚未稳定：初始请求可能连续丢失，不能只发送一次。
                self.bootstrap_attempts += 1
                if self.bootstrap_attempts <= self.bootstrap_drops:
                    continue
                token_low = struct.unpack_from('!I', request, 16)[0]
                self.navigation['boot_id'] = token_low or 1
                self.navigation['session_token'] = struct.unpack_from('!Q', request, 12)[0]
            if request[5] == 7:
                index = request[20]
                token = bytes(request[12:20])
                if struct.unpack('!Q', token)[0] != self.navigation['session_token']:
                    continue
                if self.drop_task_request_index == index:
                    self.drop_task_request_index = None
                    continue
                if request == self.last_task_fragment:
                    if not self.reack_task_duplicate:
                        continue
                    self.sock.sendto(encode_ack(request), self.ack_target)
                    continue
                self.last_task_fragment = bytes(request)
                self.delivered_task_fragments.append(bytes(request))
                if index == 0:
                    self.task_fragments = {}
                    self.task_token = token
                    self.task_next_index = 0
                if token == self.task_token and index == self.task_next_index:
                    self.task_fragments[index] = bytes(request[21:30])
                    self.task_next_index += 1
                    if self.task_next_index == 8:
                        task = b''.join(self.task_fragments[i] for i in range(8))
                        action = task[1]
                        transaction_id = struct.unpack_from('!I', task, 4)[0]
                        rejection = self.navigation_task_rejection(task)
                        with self.lock:
                            if rejection:
                                self.navigation.update(status=0, error=0x80 | rejection)
                            elif action == 1:
                                self.accepted_tasks.append(transaction_id)
                                # 分片ACK先到，新任务状态稍后才到；期间仍可能收到旧终态。
                                self.delayed_navigation = dict(self.navigation)
                                self.delayed_navigation_ticks = 3
                                self.navigation.update(
                                    task_id=transaction_id,
                                    odom_domain=struct.unpack_from('!I', task, 8)[0],
                                    boot_id=struct.unpack_from('!I', task, 12)[0],
                                    goal_x_q16=struct.unpack_from('!i', task, 16)[0],
                                    goal_y_q16=struct.unpack_from('!i', task, 20)[0],
                                    last_task_id=transaction_id,
                                    expiry_us=struct.unpack_from('!Q', task, 40)[0],
                                    mode=task[2], status=3, error=0)
                            else:
                                # 取消事务有独立编号，但终态保留原活动任务的 task_id。
                                self.navigation.update(last_task_id=transaction_id,
                                    status=7, error=0, mode=1, expiry_us=0,
                                    velocity=[0, 0, 0])
                        self.task_fragments = {}
                        self.task_next_index = 0
                if self.drop_task_ack_index == index:
                    self.drop_task_ack_index = None
                    self.dropped_task_ack.set()
                    continue
            if self.valid_ack.is_set():
                ack = encode_ack(request)
            else:
                token = struct.unpack_from('!Q', request, 12)[0] ^ 1
                ack = encode_ack(request, token)
                with self.lock:
                    self.wrong_acks += 1
            try:
                if self.valid_ack.is_set() and self.reorder_stop_acks.is_set():
                    if request[5] == 3 and request[20:26] == b'\x00' * 6:
                        self.held_zero_ack = ack
                        continue
                    if request[5] == 5 and request[20] == 0 and self.held_zero_ack:
                        self.sock.sendto(ack, self.ack_target)
                        self.sock.sendto(self.held_zero_ack, self.ack_target)
                        self.held_zero_ack = None
                        self.reordered_stop_acks.set()
                        continue
                self.sock.sendto(ack, self.ack_target)
            except OSError:
                return

    def send_image(self, frame_id, yuyv=(62, 101, 62, 240)):
        # 红色BT.709 limited YUYV对，检查640x360网络图像到ROS BGR8的转换。
        image = bytes(yuyv) * (640 * 360 // 2)
        packet_count = math.ceil(len(image) / 1200)
        sequence_start = self.rtp_sequence
        started = time.monotonic()
        frame_period = 0.08
        sent_packets = 0
        for packet in rtp_packets(frame_id, image, sequence_start):
            self.sock.sendto(packet, self.video_target)
            sent_packets += 1
            if sent_packets % 32 == 0:
                # 以单调时钟安排小批发送，避免每个数据包都受sleep粒度累积影响。
                target = started + frame_period * sent_packets / packet_count
                remaining = target - time.monotonic()
                time.sleep(remaining if remaining > 0 else 0)
        finished = time.monotonic()
        self.rtp_sequence = (sequence_start + sent_packets) & 0xFFFF
        self.last_video_send = {
            'frame_id': frame_id,
            'sequence_start': sequence_start,
            'sequence_end': (self.rtp_sequence - 1) & 0xFFFF,
            'packets': sent_packets,
            'duration_ms': round((finished - started) * 1000.0, 3),
        }
        return dict(self.last_video_send)

    def snapshot(self):
        with self.lock:
            return list(self.events)

    def wrong_ack_count(self):
        with self.lock:
            return self.wrong_acks

    def close(self):
        self.stopping.set()
        self.sock.close()
        self.thread.join(timeout=1)


def make_test_description(*, ros_control_enabled=False):
    global TEST_FAKE
    (pc_port, fpga_port, ack_port, video_port, telemetry_port) = free_ports(5)
    TEST_PORTS.update(pc=pc_port, fpga=fpga_port, ack=ack_port,
                      video=video_port, telemetry=telemetry_port)
    TEST_FAKE = FakeFpga(fpga_port, ack_port, telemetry_port, video_port, bootstrap_drops=3)
    params = {
        'bind_pc': '127.0.0.1',
        'pc_ip': '',
        'pc_port': pc_port,
        'bind_fpga': '127.0.0.1',
        'fpga_ip': '127.0.0.1',
        'fpga_port': fpga_port,
        'ack_port': ack_port,
        'video_port': video_port,
        'telemetry_port': telemetry_port,
        'video_fps': 10.0,
        'jpeg_quality': 60,
        'ros_control_enabled': ros_control_enabled,
        'video_pixel_format': 'yuyv',
    }
    node = launch_ros.actions.Node(
        package='ugv_bridge', executable='ugv_bridge_node', name='ugv_bridge_node',
        parameters=[params], additional_env=TEST_ENV, output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
        'ugv_bridge_process': node,
    }


@pytest.mark.launch_test
def generate_test_description():
    return make_test_description()


class TestBx72Bidirectional(unittest.TestCase):
    def setUp(self):
        os.environ.update(TEST_ENV)
        rclpy.init(domain_id=int(TEST_ENV['ROS_DOMAIN_ID']))
        self.node = rclpy.create_node('test_ugv_bridge_contract')
        self.images = []
        self.imu = []
        self.diagnostics = []
        self.navigation = []
        self.static_transforms = []
        self.node.create_subscription(Image, '/camera/image_raw', self.images.append,
                                      qos_profile_sensor_data)
        self.node.create_subscription(Bx72ImuRaw, '/fpga/imu_raw', self.imu.append,
                                      qos_profile_sensor_data)
        self.node.create_subscription(DiagnosticArray, '/fpga/bidirectional_status',
                                      self.diagnostics.append, 10)
        self.node.create_subscription(String, '/fpga/navigation_state',
                                      self.navigation.append, qos_profile_sensor_data)
        static_tf_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=QoSReliabilityPolicy.RELIABLE)
        self.node.create_subscription(TFMessage, '/tf_static', self.static_transforms.append,
                                      static_tf_qos)
        self.fake = TEST_FAKE
        self.pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc.bind(('127.0.0.1', 0))
        self.pc.setblocking(False)
        self.pc_seq = 0
        self.next_frame_id = 1
        self.mission_results = {}
        self.bridge_video_status = None

    def tearDown(self):
        self.pc.close()
        self.fake.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def send_control(self):
        message = {'v': 1, 'type': 'control', 'session': 'ros-ack-test',
                   'seq': self.pc_seq, 'take_control':False, 'vx': 0, 'vy': 0, 'wz': 0, 'auto': False}
        self.pc_seq += 1
        self.pc.sendto(json.dumps(message, separators=(',', ':')).encode(),
                       ('127.0.0.1', TEST_PORTS['pc']))

    def send_mission(self, action, **fields):
        request_id = f'ros-nav-{self.pc_seq}'
        message = {'v':1, 'type':'mission_command', 'session':'ros-ack-test',
                   'seq':self.pc_seq, 'request_id':request_id, 'action':action}
        status = getattr(self, 'test_status', None) or getattr(self, 'status', None) or {}
        if action not in ('stop', 'local_stop', 'local_cancel', 'emergency_stop', 'disable_ros'):
            message['claim_nonce'] = status.get('claim_nonce')
        message.update(fields)
        self.pc_seq += 1
        self.pc.sendto(json.dumps(message, separators=(',', ':')).encode(),
                       ('127.0.0.1', TEST_PORTS['pc']))
        return request_id

    def send_monitor_heartbeat(self):
        # 只刷新PC 200ms监看租约，不获取人工运动权，也不打断局部任务。
        message = {'v':1, 'type':'control', 'session':'ros-ack-test',
                   'seq':self.pc_seq, 'take_control':False,
                   'vx':0, 'vy':0, 'wz':0, 'auto':False}
        self.pc_seq += 1
        self.pc.sendto(json.dumps(message, separators=(',', ':')).encode(),
                       ('127.0.0.1', TEST_PORTS['pc']))

    def collect_mission_results(self):
        while True:
            try:
                packet = self.pc.recvfrom(65535)[0]
            except BlockingIOError:
                return
            if not packet.startswith(b'{'):
                continue
            try:
                message = json.loads(packet)
            except json.JSONDecodeError:
                continue
            if message.get('type') == 'mission_result':
                self.mission_results[message.get('request_id')] = message

    def wait_for_mission_result(self, request_id, timeout=3.0):
        deadline = time.monotonic() + timeout
        next_heartbeat = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                self.send_monitor_heartbeat()
                next_heartbeat = now + 0.05
            self.collect_mission_results()
            if request_id in self.mission_results:
                return self.mission_results[request_id]
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.collect_mission_results()
        return self.mission_results.get(request_id)

    def spin_with_monitor_heartbeat(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        next_heartbeat = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                self.send_monitor_heartbeat()
                next_heartbeat = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.collect_mission_results()
            if predicate():
                return True
        return False

    def latest_navigation(self):
        for message in reversed(self.navigation):
            try:
                return json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                continue
        return None

    def send_heartbeats(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.send_control()
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def publisher_counts(self):
        image_publishers = len(self.node.get_publishers_info_by_topic('/camera/image_raw'))
        imu_publishers = len(self.node.get_publishers_info_by_topic('/fpga/imu_raw'))
        return image_publishers, imu_publishers

    def send_next_image(self, yuyv=(62, 101, 62, 240)):
        frame_id = self.next_frame_id
        self.next_frame_id += 1
        return self.fake.send_image(frame_id, yuyv=yuyv)

    def collect_bridge_video_status(self):
        while True:
            try:
                packet = self.pc.recvfrom(65535)[0]
            except BlockingIOError:
                break
            if not packet.startswith(b'{'):
                continue
            try:
                message = json.loads(packet)
            except json.JSONDecodeError:
                continue
            if message.get('type') == 'status':
                self.bridge_video_status = message
        return self.bridge_video_status

    def wait_for_initial_streams(self, timeout=8.0):
        # 等DDS端点发现和首批回调时持续发送完整帧；每帧使用新ID，避免易失发布器竞态。
        deadline = time.monotonic() + timeout
        next_frame = time.monotonic()
        image_publishers, imu_publishers = self.publisher_counts()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_frame:
                self.send_next_image()
                next_frame += 0.1
                if next_frame <= time.monotonic():
                    next_frame = time.monotonic() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.01)
            image_publishers, imu_publishers = self.publisher_counts()
            if image_publishers > 0 and imu_publishers > 0 and self.images and self.imu:
                return True, image_publishers, imu_publishers
        return False, image_publishers, imu_publishers

    @staticmethod
    def values(status):
        return {item.key: item.value for item in status.values}

    def latest_bridge_status(self):
        for diagnostic in reversed(self.diagnostics):
            for status in diagnostic.status:
                if status.name == 'bx72_udp':
                    return status
        return None

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        # 首帧期间持续供帧，并等ROS发布端点与image/IMU消息都实际出现。
        got_streams, image_publishers, imu_publishers = self.wait_for_initial_streams()
        self.assertTrue(got_streams,
                        'ROS stream timeout: '
                        f'image_count={len(self.images)}, imu_count={len(self.imu)}, '
                        f'image_publishers={image_publishers}, imu_publishers={imu_publishers}')
        image = self.images[-1]
        self.assertEqual((image.width, image.height, image.encoding), (640, 360, 'bgr8'))
        self.assertEqual(len(image.data), 640 * 360 * 3)
        self.assertEqual(list(image.data[:3]), [0, 0, 254])
        self.assertEqual(self.imu[-1].header.frame_id, 'imu660rc_sensor')
        self.assertTrue(self.spin_until(
            lambda: self.latest_bridge_status() is not None and
            int(self.values(self.latest_bridge_status()).get('ros_image_published', '0')) > 0,
            timeout=2.0), 'the initial ROS image publish must be counted')
        self.assertTrue(self.spin_until(
            lambda: any(transform.header.frame_id == 'base_link' and
                        transform.child_frame_id == 'imu660rc_sensor'
                        for message in self.static_transforms
                        for transform in message.transforms), timeout=2.0),
            '倒装IMU需要发布base_link到原始传感器坐标的静态TF')
        imu_tf = next(transform for message in self.static_transforms
                      for transform in message.transforms
                      if transform.header.frame_id == 'base_link' and
                      transform.child_frame_id == 'imu660rc_sensor')
        self.assertAlmostEqual(imu_tf.transform.rotation.x, 1.0, places=6)
        self.assertAlmostEqual(imu_tf.transform.rotation.y, 0.0, places=6)
        self.assertAlmostEqual(imu_tf.transform.rotation.z, 0.0, places=6)
        self.assertAlmostEqual(imu_tf.transform.rotation.w, 0.0, places=6)

        # Let the last frame expire before the first PC lease. The gateway must
        # not send that cached frame to the newly connected PC as live video.
        self.spin_until(lambda: False, timeout=0.4)
        image_count = len(self.images)
        before_gap = self.values(self.latest_bridge_status())
        before_publish_count = int(before_gap['ros_image_published'])
        before_diagnostic_count = len(self.diagnostics)

        # A zero-speed control sends a real type 3 request. Wrong-token replies
        # must not make the ACK-derived diagnostic online.
        self.send_heartbeats(0.25)
        self.assertTrue(self.spin_until(lambda: len(self.diagnostics) > before_diagnostic_count,
                                        timeout=1.0), 'expected an updated bridge diagnostic')
        self.assertEqual(len(self.images), image_count, 'an old frame must not be republished')
        gap_status = self.values(self.latest_bridge_status())
        self.assertEqual(int(gap_status['ros_image_published']), before_publish_count,
                         'expired cached video must not be republished to ROS')
        received = []
        while True:
            try:
                received.append(self.pc.recvfrom(65535)[0])
            except BlockingIOError:
                break
        self.assertFalse(any(packet.startswith(b'BXJV') for packet in received),
                         'a new PC lease must not receive expired cached video')
        # Resume with a real stream of new black frames. SensorDataQoS is
        # best-effort, so a single large image can be lost after publication.
        # Pre-outage frames are red and cannot masquerade as recovery.
        recovery_stamp_ns = self.node.get_clock().now().nanoseconds

        def is_fresh_black_image(candidate):
            stamp_ns = candidate.header.stamp.sec * 1_000_000_000 + candidate.header.stamp.nanosec
            return stamp_ns > recovery_stamp_ns and list(candidate.data[:3]) == [0, 0, 0]

        def fresh_ros_image_arrived():
            self.collect_bridge_video_status()
            return any(is_fresh_black_image(candidate)
                       for candidate in self.images[image_count:])

        deadline = time.monotonic() + 8.0
        resumed = False
        sent_frames = []
        while time.monotonic() + 0.1 < deadline:
            sent_frames.append(self.send_next_image(yuyv=(16, 128, 16, 128)))
            if self.spin_until(fresh_ros_image_arrived,
                               timeout=min(0.12, max(0.0, deadline - time.monotonic()))):
                resumed = True
                break
        final_status = self.latest_bridge_status()
        image_diagnostic = self.values(final_status) if final_status is not None else None
        self.assertTrue(resumed,
                        'ROS video must resume when a fresh complete frame arrives; '
                        f'sent_frames={len(sent_frames)}, '
                        f'last_sent_frame={sent_frames[-1] if sent_frames else None}, '
                        f'bridge_video={self.bridge_video_status}, '
                        f'image_publishers={self.publisher_counts()[0]}, '
                        f'images_before={image_count}, images_after={len(self.images)}, '
                        f'diagnostic_before={gap_status}, diagnostic_after={image_diagnostic}')
        resumed_image = next(candidate for candidate in self.images[image_count:]
                             if is_fresh_black_image(candidate))
        self.assertEqual((resumed_image.width, resumed_image.height, resumed_image.encoding),
                         (640, 360, 'bgr8'))
        self.assertEqual(len(resumed_image.data), 640 * 360 * 3)
        self.assertFalse(any(resumed_image.data), 'the complete resumed image must match the new black frame')
        self.assertTrue(self.spin_until(
            lambda: self.latest_bridge_status() is not None and
            int(self.values(self.latest_bridge_status()).get('ros_image_published', '0')) > before_publish_count,
            timeout=2.0), 'the resumed ROS image publish must be counted')
        events = self.fake.snapshot()
        wrong_acks = self.fake.wrong_ack_count()
        self.assertTrue(any(request[5] == 3 for _, request in events))
        self.assertGreater(wrong_acks, 0)
        self.assertTrue(self.spin_until(
            lambda: self.latest_bridge_status() is not None and
            self.values(self.latest_bridge_status()).get('control_ack_fresh') == 'False',
            timeout=3.0))

        self.fake.valid_ack.set()
        self.assertTrue(self.spin_until(
            lambda: (self.send_control() is None and
                     self.latest_bridge_status() is not None and
                     self.values(self.latest_bridge_status()).get('bidirectional') == 'True' and
                     self.values(self.latest_bridge_status()).get('mode_ready') == 'True'),
            timeout=5.0))
        status = self.latest_bridge_status()
        self.assertGreaterEqual(self.fake.bootstrap_attempts, 4,
                                '初始零速请求连续丢失后应自动重试并恢复握手')
        self.assertEqual(self.values(status).get('control_ack_fresh'), 'True')
        self.assertEqual(status.level[0] if isinstance(status.level, (bytes, bytearray)) else status.level, 0)

        # stop会撤销PC运动租约并重置FPGA零速/手动模式屏障；等新的诊断快照确认该屏障。
        stop_diag_start = len(self.diagnostics)
        stop_request = self.send_mission('stop')
        stop_result = self.wait_for_mission_result(stop_request)
        self.assertIsNotNone(stop_result, 'stop command must return a mission_result')
        self.assertTrue(stop_result['success'], f"stop failed: {stop_result.get('message')}")
        self.assertTrue(stop_result['unknown'],
                        'stop should report that the unavailable visual task state is unconfirmed')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: len(self.diagnostics) > stop_diag_start and
            self.latest_bridge_status() is not None and
            self.values(self.latest_bridge_status()).get('control_source') == 'none' and
            self.values(self.latest_bridge_status()).get('mode_ready') == 'True', timeout=2.0),
            'stop must be followed by a fresh zero-speed and manual-mode ACK barrier')

        idle_navigation = self.latest_navigation()
        self.assertIsNotNone(idle_navigation, 'fresh nav_full_v1 state is required for a task')
        self.assertEqual(idle_navigation['protocol'], 'nav_full_v1')
        self.assertGreater(idle_navigation['boot_id'], 0)
        self.assertGreater(idle_navigation['odom_domain'], 0)
        self.assertGreater(idle_navigation['session_token'], 0)
        # Jetson 任务与断链处理由 test_navigation_lease_stop.py 覆盖。


@launch_testing.post_shutdown_test()
class TestCleanExit(unittest.TestCase):
    def test_exit_code(self, proc_info, ugv_bridge_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=ugv_bridge_process)

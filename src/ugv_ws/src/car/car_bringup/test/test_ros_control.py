"""Loopback checks for the ROS/PC control lease and FPGA ACK barriers."""
import binascii
import importlib.util
import json
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
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from ugv_bridge.msg import AuthorizedTwist


TEST_ENV = {'ROS_DOMAIN_ID': os.environ.get('UGV_TEST_ROS_DOMAIN_ID', str(32 + os.getpid() % 60)),
            'ROS_LOCALHOST_ONLY': '1'}
TEST_PORTS = {}
TEST_FAKE = None

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


def encode_ack(request):
    ack = bytearray(request)
    ack[5] = {1: 2, 3: 4, 5: 6, 7: 8}[request[5]]
    ack[30:32] = struct.pack('!H', binascii.crc_hqx(ack[:30], 0xFFFF))
    return bytes(ack)


def zero_motor_packet(sequence, time_us):
    packet = bytearray(104)
    struct.pack_into('!4sBBHII', packet, 0, b'BX72', 1, 4, len(packet),
                     sequence, time_us & 0xFFFFFFFF)
    packet[16] = packet[17] = 0xE4  # 四个逻辑轮与物理轮一一对应。
    return bytes(packet)


def ready_health_packet(sequence, time_us):
    packet = bytearray(128)
    struct.pack_into('!4sBBHII', packet, 0, b'BX72', 1, 7, len(packet),
                     sequence, time_us & 0xFFFFFFFF)
    struct.pack_into('!I', packet, 124, 0x47520001)
    return bytes(packet)


class FakeFpga:
    def __init__(self, fpga_port, ack_port, telemetry_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', fpga_port))
        self.sock.settimeout(0.02)
        self.ack_target = ('127.0.0.1', ack_port)
        self.telemetry_target = ('127.0.0.1', telemetry_port)
        self.events = []
        self.task_fragments = {}
        self.task_token = None
        self.task_next_index = 0
        self.navigation = {
            'boot_id': 0, 'odom_domain': 1, 'task_id': 0, 'last_task_id': 0,
            'session_token': 0, 'expiry_us': 0, 'status': 0, 'error': 0,
            'mode': 1, 'flags': 0x86, 'velocity': [0, 0, 0],
            'goal_x_q16': 0, 'goal_y_q16': 0, 'planner_expansions': 0,
            'progress': 0, 'map_age_us': 0, 'pose_age_us': 0, 'map_version': 1,
        }
        self.lock = threading.Lock()
        self.ack_paused = threading.Event()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        sequence = 0
        next_telemetry = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            if now >= next_telemetry:
                sequence = (sequence + 4) & 0xFFFFFFFF
                now_us = time.monotonic_ns() // 1000
                try:
                    self.sock.sendto(telemetry_packet(sequence, now_us & 0xFFFFFFFF),
                                     self.telemetry_target)
                    self.sock.sendto(zero_motor_packet(sequence + 1, now_us),
                                     self.telemetry_target)
                    self.sock.sendto(ready_health_packet(sequence + 2, now_us),
                                     self.telemetry_target)
                    self.sock.sendto(navigation_packet(sequence + 3, now_us, self.navigation),
                                     self.telemetry_target)
                except OSError:
                    return
                next_telemetry = now + 0.1
            try:
                request, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(request) != 32 or request[:5] != b'BX72\x01' or request[5] not in (1, 3, 5, 7):
                continue
            with self.lock:
                self.events.append((time.monotonic(), bytes(request)))
            if request[5] == 3 and request[20:26] == b'\x00' * 6 and self.navigation['boot_id'] == 0:
                token = struct.unpack_from('!Q', request, 12)[0]
                self.navigation['boot_id'] = (token & 0xFFFFFFFF) or 1
                self.navigation['session_token'] = token
            if request[5] == 7:
                index = request[20]
                token = bytes(request[12:20])
                if struct.unpack('!Q', token)[0] != self.navigation['session_token']:
                    continue
                if index == 0:
                    self.task_fragments = {}
                    self.task_token = token
                    self.task_next_index = 0
                if token == self.task_token and index == self.task_next_index:
                    self.task_fragments[index] = bytes(request[21:30])
                    self.task_next_index += 1
                    if self.task_next_index == 8:
                        payload = b''.join(self.task_fragments[i] for i in range(8))
                        transaction_id = struct.unpack_from('!I', payload, 4)[0]
                        if payload[1] in (2, 3):
                            self.navigation.update(last_task_id=transaction_id, status=7,
                                error=0, mode=1, expiry_us=0, velocity=[0, 0, 0])
                        self.task_fragments = {}
                        self.task_next_index = 0
            if self.ack_paused.is_set():
                continue
            try:
                self.sock.sendto(encode_ack(request), self.ack_target)
            except OSError:
                return

    def snapshot(self):
        with self.lock:
            return list(self.events)

    def pause_acknowledgements(self):
        self.ack_paused.set()

    def resume_acknowledgements(self):
        self.ack_paused.clear()

    def close(self):
        self.stopping.set()
        self.sock.close()
        self.thread.join(timeout=1)


@pytest.mark.launch_test
def generate_test_description():
    global TEST_FAKE
    pc_port, fpga_port, ack_port, video_port, telemetry_port = free_ports(5)
    TEST_PORTS.update(pc=pc_port, fpga=fpga_port, ack=ack_port,
                      video=video_port, telemetry=telemetry_port)
    TEST_FAKE = FakeFpga(fpga_port, ack_port, telemetry_port)
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
        'ros_control_enabled': True,
        'cmd_vel_timeout_s': 0.2,
        'video_pixel_format': 'yuyv',
    }
    bridge = launch_ros.actions.Node(
        package='ugv_bridge', executable='ugv_bridge_node', name='ugv_bridge_node',
        parameters=[params], additional_env=TEST_ENV, output='screen')
    return launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()]), {
        'ugv_bridge_process': bridge,
    }


class TestRosControl(unittest.TestCase):
    def setUp(self):
        os.environ.update(TEST_ENV)
        rclpy.init(domain_id=int(TEST_ENV['ROS_DOMAIN_ID']))
        self.node = rclpy.create_node('test_ros_control')
        self.commands = self.node.create_publisher(Twist, '/cmd_vel', 1)
        self.mission_commands = self.node.create_publisher(AuthorizedTwist, '/mission/cmd_vel', 1)
        self.emergency_stop = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.enable_ros = self.node.create_client(SetBool, '/control/enable_ros')
        self.diagnostics = []
        self.navigation = []
        self.node.create_subscription(
            DiagnosticArray, '/fpga/bidirectional_status', self.diagnostics.append,
            qos_profile_sensor_data)
        self.node.create_subscription(
            String, '/fpga/navigation_state', self.record_navigation_state,
            qos_profile_sensor_data)
        self.fake = TEST_FAKE
        self.pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc.setblocking(False)
        self.pc_claim_nonce = None
        self.pc_seq = 0

    def tearDown(self):
        self.pc.close()
        self.fake.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def spin_for(self, duration):
        self.spin_until(lambda: False, timeout=duration)

    def latest_control_status(self, start=0):
        for diagnostic in reversed(self.diagnostics[start:]):
            for status in diagnostic.status:
                if status.name == 'bx72_udp':
                    return {item.key: item.value for item in status.values}
        return None

    def record_navigation_state(self, message):
        self.navigation.append((time.monotonic(), message.data))

    def latest_navigation_state(self):
        for _, encoded in reversed(self.navigation):
            try:
                return json.loads(encoded)
            except (TypeError, json.JSONDecodeError):
                continue
        return None

    def navigation_session_stable(self):
        if len(self.navigation) < 3:
            return False
        samples = self.navigation[-3:]
        states = []
        for _, encoded in samples:
            try:
                states.append(json.loads(encoded))
            except (TypeError, json.JSONDecodeError):
                return False
        identity = lambda state: (
            state.get('protocol'), state.get('boot_id'), state.get('odom_domain'),
            state.get('session_token'))
        first = identity(states[0])
        if (first[0] != 'nav_full_v1' or not first[1] or not first[2] or not first[3] or
                any(identity(state) != first for state in states[1:])):
            return False
        now_stamps = [state.get('now_us') for state in states]
        if any(stamp is None for stamp in now_stamps) or len(set(now_stamps)) != 3:
            return False
        return time.monotonic() - samples[-1][0] <= 0.25

    def wait_status(self, start=0, **expected):
        return self.spin_until(lambda: self._status_matches(expected, start))

    def wait_pc_mode_ready(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        next_heartbeat = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                # PC控制租约是200ms；持续发送零速保活以等待FPGA重做模式屏障。
                self.send_pc(self.next_pc_seq())
                next_heartbeat = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (self._status_matches({'control_source': 'pc', 'mode_ready': 'True'}) and
                    self.navigation_session_stable()):
                return True
        return False

    def _status_matches(self, expected, start=0):
        values = self.latest_control_status(start)
        return values is not None and all(values.get(key) == value for key, value in expected.items())

    def authorize_ros(self):
        self.assertTrue(self.enable_ros.wait_for_service(timeout_sec=5.0))
        status_start = len(self.diagnostics)
        request = SetBool.Request()
        request.data = True
        future = self.enable_ros.call_async(request)
        self.assertTrue(self.spin_until(future.done, timeout=5.0))
        response = future.result()
        if response and response.success:
            deadline = time.monotonic() + 3.0
            next_command = 0.0
            ready = False
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_command:
                    # ROS命令租约为200ms；屏障等待期间用新鲜零速样本续期。
                    self.publish_twist(0.0)
                    next_command = now + 0.05
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if self._status_matches({
                        'control_source': 'ros', 'ros_authorized': 'True',
                        'mode_ready': 'True'}, status_start):
                    ready = True
                    break
            self.assertTrue(ready,
                            'ROS authorization did not reach a fresh ready barrier: '
                            f"status={self.latest_control_status(status_start)}, "
                            f"navigation={self.latest_navigation_state()}")
        return response

    def publish_twist(self, x=0.0, y=0.0, yaw=0.0):
        message = Twist()
        message.linear.x = float(x)
        message.linear.y = float(y)
        message.angular.z = float(yaw)
        self.commands.publish(message)

    def send_pc(self, seq, vx=0, vy=0, wz=0, auto=False):
        while True:
            try:
                packet = self.pc.recvfrom(65535)[0]
                if packet.startswith(b'{'):
                    status = json.loads(packet)
                    if status.get('type') == 'status' and status.get('session') == 'pc-session':
                        self.pc_claim_nonce = status.get('claim_nonce')
            except BlockingIOError:
                break
        message = {'v': 1, 'type': 'control', 'session': 'pc-session', 'seq': seq,
                   'vx': vx, 'vy': vy, 'wz': wz, 'auto': auto,
                   'take_control': bool(self.pc_claim_nonce)}
        if self.pc_claim_nonce:
            message['claim_nonce'] = self.pc_claim_nonce
            if not auto and not (vx or vy or wz):
                message['claim'] = True
        self.pc.sendto(json.dumps(message, separators=(',', ':')).encode(),
                       ('127.0.0.1', TEST_PORTS['pc']))

    def next_pc_seq(self):
        seq = self.pc_seq
        self.pc_seq = (self.pc_seq + 1) & 0xFFFFFFFF
        return seq

    def events_after(self, index):
        return self.fake.snapshot()[index:]

    def command_events(self, events):
        return [(stamp, packet, struct.unpack('!hhh', packet[20:26]))
                for stamp, packet in events if packet[5] == 3]

    def wait_velocity(self, velocity, start=0, timeout=3.0):
        matched = self.spin_until(lambda: any(
            value == velocity for _, _, value in self.command_events(self.fake.snapshot()[start:])),
            timeout=timeout)
        if not matched:
            events = self.command_events(self.fake.snapshot()[start:])
            recent = [(round(stamp, 3), value) for stamp, _, value in events[-20:]]
            self.fail(f"timed out waiting for FPGA velocity {velocity}; "
                      f"control_status={self.latest_control_status()}, "
                      f"navigation={self.latest_navigation_state()}, "
                      f"recent_type3_commands={recent}")
        return True

    def assert_stop_barrier(self, events):
        last_nonzero_motion_index = max(
            (index for index, (_, packet) in enumerate(events)
             if packet[5] == 3 and
             struct.unpack('!hhh', packet[20:26]) != (0, 0, 0)), default=-1)
        tail = events[last_nonzero_motion_index + 1:]
        zero_position = next((index for index, (_, packet) in enumerate(tail)
                              if packet[5] == 3 and
                              struct.unpack('!hhh', packet[20:26]) == (0, 0, 0)), None)
        auto_off_position = next((index for index, (_, packet) in enumerate(tail)
                                  if packet[5] == 5 and packet[20] == 0), None)
        self.assertIsNotNone(zero_position)
        self.assertIsNotNone(auto_off_position)
        self.assertLess(zero_position, auto_off_position)
        self.assertFalse(any(packet[5] == 5 and packet[20] != 0
                             for _, packet in events))

    def test_ros_leases_timeouts_emergency_and_pc_preemption(self):
        self.assertTrue(self.wait_status(mode_ready='True'),
                        f"initial status={self.latest_control_status()}, "
                        f"navigation={self.latest_navigation_state()}, "
                        f"packets={len(self.fake.snapshot())}")

        # Establish a PC session first; ROS authorization must disarm it without
        # losing its sequence watermark, so a later new zero packet can rearm it.
        self.assertTrue(self.wait_pc_mode_ready(),
                        f"PC mode or nav session did not stabilize: "
                        f"status={self.latest_control_status()}, "
                        f"navigation={self.latest_navigation_state()}")
        self.assertEqual(self.latest_control_status().get('ros_authorized'), 'False')

        barrier_start = len(self.fake.snapshot())
        self.assertTrue(self.authorize_ros().success)
        self.assertTrue(self.wait_status(control_source='ros', ros_authorized='True',
                                         mode_ready='True'))
        barrier = self.events_after(barrier_start)
        self.assert_stop_barrier(barrier)

        # SI conversion, rounding and a new post-authorization command.
        ros_velocity = (123, -45, 200)
        self.publish_twist(.1234, -.0454, .2004)
        self.assertTrue(self.wait_velocity(ros_velocity))
        ordered_events = self.events_after(barrier_start)
        zero_index = next(index for index, (_, packet) in enumerate(ordered_events)
                          if packet[5] == 3 and
                          struct.unpack('!hhh', packet[20:26]) == (0, 0, 0))
        mode_off_index = next(index for index, (_, packet) in enumerate(ordered_events)
                              if packet[5] == 5 and packet[20] == 0)
        motion_index = next(index for index, (_, packet) in enumerate(ordered_events)
                            if packet[5] == 3 and
                            struct.unpack('!hhh', packet[20:26]) == ros_velocity)
        self.assertLess(zero_index, mode_off_index)
        self.assertLess(mode_off_index, motion_index)

        # Both signed manual yaw limits reach the packet; values beyond them clamp.
        for x, y, yaw, expected_yaw in ((.4, -.4, .8, 800),
                                         (.4, -.4, -.8, -800),
                                         (.6, -.6, 1.0, 800),
                                         (.6, -.6, -1.0, -800)):
            boundary_start = len(self.fake.snapshot())
            self.publish_twist(x, y, yaw)
            self.assertTrue(self.wait_velocity((400, -400, expected_yaw), start=boundary_start))

        # Missing ACKs revoke the lease; after the zero/off barrier is restored,
        # no cached ROS velocity is sent again.
        ack_failure_start = len(self.fake.snapshot())
        self.fake.pause_acknowledgements()
        self.publish_twist(.07)
        self.assertTrue(self.spin_until(lambda: any(
            value == (70, 0, 0) for _, _, value in
            self.command_events(self.fake.snapshot()[ack_failure_start:])), timeout=2.0))
        self.assertTrue(self.wait_status(ros_authorized='False'))
        after_ack_failure = len(self.fake.snapshot())
        self.fake.resume_acknowledgements()
        self.assertTrue(self.wait_status(mode_ready='True'))
        self.assert_stop_barrier(self.events_after(ack_failure_start))
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(after_ack_failure))))

        # Non-finite input revokes authorization; later samples cannot move until
        # SetBool authorizes a new command stream again.
        self.assertTrue(self.authorize_ros().success)
        self.publish_twist(float('nan'))
        self.assertTrue(self.wait_status(ros_authorized='False'))
        self.assertTrue(self.wait_status(mode_ready='True'))
        invalid_start = len(self.fake.snapshot())
        self.publish_twist(.2)
        self.spin_for(.12)
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(invalid_start))))

        self.assertTrue(self.wait_status(mode_ready='True'))
        self.assertTrue(self.authorize_ros().success)
        timeout_start = len(self.fake.snapshot())
        command_sent_at = time.monotonic()
        self.publish_twist(-.15, .05, -.1)
        self.assertTrue(self.wait_velocity((-150, 50, -100), start=timeout_start))
        cutoff = command_sent_at + 0.4
        self.assertTrue(self.spin_until(lambda: time.monotonic() >= cutoff, timeout=1.0))
        self.assertTrue(self.wait_status(ros_authorized='False'))
        self.assertFalse(any(stamp >= cutoff and value != (0, 0, 0)
                             for stamp, _, value in
                             self.command_events(self.events_after(timeout_start))))

        # Emergency stop revokes ROS; release does not restore its lease.
        self.assertTrue(self.wait_status(mode_ready='True'))
        self.assertTrue(self.authorize_ros().success)
        self.publish_twist(.1)
        self.assertTrue(self.wait_velocity((100, 0, 0)))
        emergency_start = len(self.fake.snapshot())
        self.emergency_stop.publish(Bool(data=True))
        self.assertTrue(self.wait_status(emergency_stop='True', ros_authorized='False',
                                         mode_ready='True'))
        self.assert_stop_barrier(self.events_after(emergency_start))
        self.assertFalse(self.authorize_ros().success)
        stopped = len(self.fake.snapshot())
        self.emergency_stop.publish(Bool(data=False))
        self.assertTrue(self.wait_status(emergency_stop='False'))
        self.publish_twist(.2)
        self.spin_for(.12)
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(stopped))))

        # A PC auto request cannot take over ROS. A newer zero-speed manual packet
        # from the same PC session does take over, followed by the PC velocity.
        self.assertTrue(self.wait_status(mode_ready='True'))
        self.assertTrue(self.authorize_ros().success)
        self.publish_twist(.08)
        self.assertTrue(self.wait_velocity((80, 0, 0)))
        rejected_auto_seq = self.next_pc_seq()
        self.send_pc(rejected_auto_seq, auto=True)
        self.assertTrue(self.wait_status(control_source='ros', ros_authorized='True'))
        self.send_pc(rejected_auto_seq)
        self.assertTrue(self.wait_status(control_source='pc', ros_authorized='False'))
        self.send_pc(self.next_pc_seq(), vx=125)
        self.assertTrue(self.wait_velocity((125, 0, 0)))

        # E-stop also disarms PC. A later nonzero packet from that lease cannot
        # restore motion without another zero-speed handshake.
        pc_stop_start = len(self.fake.snapshot())
        self.emergency_stop.publish(Bool(data=True))
        self.assertTrue(self.wait_status(emergency_stop='True', mode_ready='True'))
        self.assert_stop_barrier(self.events_after(pc_stop_start))
        stopped = len(self.fake.snapshot())
        self.send_pc(self.next_pc_seq(), vx=200)
        self.spin_for(.1)
        self.emergency_stop.publish(Bool(data=False))
        self.assertTrue(self.wait_status(emergency_stop='False'))
        self.send_pc(self.next_pc_seq(), vx=200)
        self.spin_for(.12)
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(stopped))))


        # Explicit disable also revokes ROS, even while its command is fresh.
        self.assertTrue(self.wait_status(mode_ready='True'))
        self.assertTrue(self.authorize_ros().success)
        moving_start = len(self.fake.snapshot())
        self.publish_twist(yaw=.09)
        self.assertTrue(self.wait_velocity((0, 0, 90), start=moving_start))
        request = SetBool.Request()
        request.data = False
        status_start = len(self.diagnostics)
        future = self.enable_ros.call_async(request)
        self.assertTrue(self.spin_until(future.done))
        self.assertTrue(future.result().success)
        self.assertTrue(self.wait_status(start=status_start, ros_authorized='False',
                                        mode_ready='True'))
        disabled_start = len(self.fake.snapshot())
        self.publish_twist(yaw=.2)
        self.spin_for(.12)
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(disabled_start))))
        self.assertTrue(self.authorize_ros().success)
        rearmed_start = len(self.fake.snapshot())
        self.publish_twist(yaw=.11)
        self.assertTrue(self.wait_velocity((0, 0, 110), start=rearmed_start))

        # An old mission command must not cross a revoke/re-authorize race,
        # even when its message timestamp is newly generated.
        old_status = self.latest_control_status()
        old_epoch = int(old_status['ros_authorization_epoch'])
        bridge_id = old_status['bridge_id']
        request = SetBool.Request()
        request.data = False
        future = self.enable_ros.call_async(request)
        self.assertTrue(self.spin_until(future.done))
        self.assertTrue(future.result().success)
        self.assertTrue(self.wait_status(ros_authorized='False', mode_ready='True'))
        self.assertTrue(self.authorize_ros().success)
        new_status = self.latest_control_status()
        new_epoch = int(new_status['ros_authorization_epoch'])
        self.assertGreater(new_epoch, old_epoch)

        def publish_mission(epoch, identity, yaw=0.17):
            command = AuthorizedTwist()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.header.frame_id = 'base_link'
            command.bridge_id = identity
            command.authorization_epoch = epoch
            command.twist.angular.z = yaw
            self.mission_commands.publish(command)

        stale_start = len(self.fake.snapshot())
        for _ in range(2):
            # Invalid commands do not renew the 200 ms ROS lease. Send valid
            # zero commands while testing that each invalid command is rejected.
            publish_mission(new_epoch, bridge_id, yaw=0.0)
            self.spin_for(.025)
            publish_mission(old_epoch, bridge_id)
            self.spin_for(.025)
            publish_mission(new_epoch, bridge_id, yaw=0.0)
            self.spin_for(.025)
            publish_mission(new_epoch, 'different-bridge')
            self.spin_for(.025)
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(stale_start))))

        # Wait for a new diagnostic proving that this exact authorization is
        # still active, keeping it alive with accepted zero commands.
        status_start = len(self.diagnostics)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            publish_mission(new_epoch, bridge_id, yaw=0.0)
            self.spin_for(.025)
            if self._status_matches({'ros_authorized': 'True', 'control_source': 'ros',
                                     'ros_authorization_epoch': str(new_epoch)}, status_start):
                break
        self.assertTrue(self._status_matches({'ros_authorized': 'True', 'control_source': 'ros',
                                             'ros_authorization_epoch': str(new_epoch)}, status_start),
                        f'valid zero command did not retain authorization: {self.latest_control_status()}')

        def publish_until_velocity(yaw, velocity, start, timeout=2.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                publish_mission(new_epoch, bridge_id, yaw=yaw)
                self.spin_for(.025)
                if any(value == velocity for _, _, value in
                       self.command_events(self.events_after(start))):
                    return True
            return False

        for yaw, expected_yaw in ((.17, 170), (.8, 800), (-.8, -800)):
            current_start = len(self.fake.snapshot())
            self.assertTrue(publish_until_velocity(yaw, (0, 0, expected_yaw), current_start),
                            f'current mission command was not applied: {self.latest_control_status()}')

        # Once a valid zero has reached the fake FPGA, send only invalid
        # commands. They must neither move the vehicle nor renew the lease.
        zero_start = len(self.fake.snapshot())
        self.assertTrue(publish_until_velocity(0.0, (0, 0, 0), zero_start))
        invalid_only_start = len(self.fake.snapshot())
        status_start = len(self.diagnostics)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            publish_mission(old_epoch, bridge_id)
            self.spin_for(.025)
            publish_mission(new_epoch, 'different-bridge')
            self.spin_for(.025)
            if self._status_matches({'ros_authorized': 'False', 'control_source': 'none'}, status_start):
                break
        self.assertTrue(self._status_matches({'ros_authorized': 'False', 'control_source': 'none'}, status_start),
                        f'invalid commands unexpectedly retained authorization: {self.latest_control_status()}')
        expired_status = self.latest_control_status(status_start)
        self.assertEqual(expired_status['bridge_id'], bridge_id)
        self.assertEqual(int(expired_status['ros_authorization_epoch']), new_epoch + 1)
        self.assertEqual(expired_status['timeouts'], new_status['timeouts'],
                         'the lease must expire without an FPGA ACK timeout')
        self.assertFalse(any(value != (0, 0, 0) for _, _, value in
                             self.command_events(self.events_after(invalid_only_start))))



@launch_testing.post_shutdown_test()
class TestCleanExit(unittest.TestCase):
    def test_exit_code(self, proc_info, ugv_bridge_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=ugv_bridge_process)

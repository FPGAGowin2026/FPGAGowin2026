"""Two independent UDP observers against the existing localhost fake FPGA."""
import importlib.util
import json
from pathlib import Path
import socket
import struct
import threading
import time

import launch_testing
import pytest
import rclpy
from std_msgs.msg import Bool, String
from ugv_bridge.srv import GlobalNavigation


spec = importlib.util.spec_from_file_location(
    'navigation_fixture', Path(__file__).with_name('test_navigation_lease_stop.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    return fixture.generate_test_description()


def merge_fragments(target, source):
    for key, value in source.items():
        if key not in target:
            target[key] = value
        elif isinstance(value, dict) and isinstance(target[key], dict):
            merge_fragments(target[key], value)
        elif isinstance(value, list) and isinstance(target[key], list):
            target[key].extend(value)
        elif target[key] != value:
            raise AssertionError(f'conflicting fragment {key}')


class ModeAckGate:
    """Delay one fake FPGA mode ACK without touching any real network port."""
    def __init__(self, sock, ack_target):
        self.sock, self.ack_target = sock, ack_target
        self.hold = threading.Event()
        self.held = threading.Event()
        self.lock = threading.Lock()
        self.packet = None

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def sendto(self, packet, target):
        if self.hold.is_set() and target == self.ack_target and len(packet) == 32 and packet[:4] == b'BX72' and packet[5] == 6:
            with self.lock:
                self.packet = (packet, target)
                self.held.set()
            return len(packet)
        return self.sock.sendto(packet, target)

    def release(self):
        self.hold.clear()
        with self.lock:
            packet, self.packet = self.packet, None
        if packet:
            self.sock.sendto(*packet)


class TestMultiMonitor(fixture.TestNavigationLeaseStop):
    def setUp(self):
        super().setUp()
        self.viewer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.viewer.bind(('127.0.0.1', 0))
        self.viewer.setblocking(False)
        self.viewer_seq = 0
        self.streams = {'owner': [], 'viewer': []}
        self.statuses = {'owner': [], 'viewer': []}
        self.fragments = {'owner': {}, 'viewer': {}}
        self.video_fragments = {'owner': {}, 'viewer': {}}
        self.video_frames = {'owner': {}, 'viewer': {}}
        self.results = {}
        self.global_state = self.node.create_publisher(String, '/jetson/global_navigation/state', 10)
        self.local_grid = self.node.create_publisher(String, '/jetson/navigation/local_grid', 10)
        self.stop_publisher = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.fixture_seq = 0
        self.global_active = False
        self.mode_ack = ModeAckGate(self.fake.sock, self.fake.ack_target)
        self.fake.sock = self.mode_ack

    def tearDown(self):
        self.viewer.close()
        super().tearDown()

    def observe_packet(self, name, packet):
        try:
            message = json.loads(packet)
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if message.get('type') == 'status':
            self.statuses[name].append(message)
            if name == 'owner':
                self.test_status = message
        elif message.get('type') in ('mission_result', 'navigation_task_result'):
            self.results[message.get('request_id')] = message
            if name == 'owner':
                self.mission_results[message.get('request_id')] = message
        elif message.get('type') == 'telemetry':
            stream, data = message.get('stream'), message.get('data', {})
            if name == 'owner' and stream == 'navigation':
                self.test_navigation = data
            if stream not in ('local_grid', 'navigation_task'):
                return
            if 'part' not in message:
                self.streams[name].append((stream, data))
                return
            key = (stream, data.get('sequence'))
            pieces = self.fragments[name].setdefault(key, {})
            pieces[message['part']] = data
            if len(pieces) == message['parts']:
                merged = {}
                for index in range(message['parts']):
                    merge_fragments(merged, pieces[index])
                self.streams[name].append((stream, merged))
                del self.fragments[name][key]

    def collect_mission_results(self):
        for name, sock in (('owner', self.pc), ('viewer', self.viewer)):
            while True:
                try:
                    packet = sock.recvfrom(65535)[0]
                except BlockingIOError:
                    break
                if packet.startswith(b'BXJV') and len(packet) > 12:
                    frame, part, count = struct.unpack_from('!IHH', packet, 4)
                    if 0 <= part < count <= 512:
                        pieces = self.video_fragments[name].setdefault(frame, {})
                        pieces[part] = packet[12:]
                        if len(pieces) == count:
                            self.video_frames[name][frame] = b''.join(pieces[i] for i in range(count))
                            del self.video_fragments[name][frame]
                elif packet.startswith(b'{'):
                    self.observe_packet(name, packet)

    def send_viewer_heartbeat(self, *, manual=False, claim=False):
        message = {'v':1, 'type':'control', 'session':'observer-b', 'seq':self.viewer_seq,
                   'take_control':manual, 'vx':0, 'vy':0, 'wz':0, 'auto':False}
        if manual:
            message['claim_nonce'] = self.statuses['viewer'][-1]['claim_nonce']
            if claim:
                message['claim'] = True
        self.viewer_seq += 1
        self.viewer.sendto(json.dumps(message).encode(), ('127.0.0.1', fixture.fixture.TEST_PORTS['pc']))

    def send_control(self):
        nonce = (self.test_status or {}).get('claim_nonce')
        if not nonce:
            return self.send_monitor_heartbeat()
        message = {'v':1, 'type':'control', 'session':'ros-ack-test', 'seq':self.pc_seq,
                   'take_control':True, 'claim':True, 'vx':0, 'vy':0, 'wz':0, 'auto':False, 'claim_nonce':nonce}
        self.pc_seq += 1
        self.pc.sendto(json.dumps(message).encode(), ('127.0.0.1', fixture.fixture.TEST_PORTS['pc']))

    def send_mission(self, action, **fields):
        if action not in ('stop', 'local_stop', 'local_cancel', 'emergency_stop', 'disable_ros'):
            fields['claim_nonce'] = (self.test_status or {}).get('claim_nonce')
        return super().send_mission(action, **fields)

    def publish_test_streams(self):
        self.fixture_seq += 1
        now_us = int(self.fake.navigation['now_us']) & 0xffffffff
        grid = {'type':'local_grid', 'protocol':'jetson_grid_snapshot_v1', 'source':'jetson',
            'sequence':self.fixture_seq, 'snapshot_id':self.fixture_seq, 'time_us':now_us,
            'boot_id':self.fake.navigation['boot_id'], 'odom_domain':self.fake.navigation['odom_domain'],
            'capture_begin_us':now_us, 'capture_end_us':now_us, 'grid_generation':1,
            'scan_generation':1, 'complete_scan_stamp_us':now_us, 'last_ray_stamp_us':now_us,
            'grid_frame_epoch':1, 'transform_revision':1, 'flags':7, 'width':64, 'height':64,
            'resolution_mm':50, 'origin_x_cell':-32, 'origin_y_cell':-32,
            'grid_from_odom':{'map_anchor_x_mm':0,'map_anchor_y_mm':0,'odom_anchor_x_mm':0,
                              'odom_anchor_y_mm':0,'yaw_offset_mrad':0},
            'packed_cells':[85] * 1024}
        self.local_grid.publish(String(data=json.dumps(grid)))
        state = {'active':self.global_active, 'status':'following' if self.global_active else 'cancelled',
                 'route':[{'x':i / 20,'y':0} for i in range(120)]}
        self.global_state.publish(String(data=json.dumps(state)))

    def spin_both(self, predicate, timeout, *, owner=True, manual=False, viewer_manual=False, publish=False):
        deadline, next_heartbeat, next_state = time.monotonic() + timeout, 0.0, 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                if owner:
                    self.send_control() if manual else self.send_monitor_heartbeat()
                self.send_viewer_heartbeat(manual=viewer_manual)
                next_heartbeat = now + .05
            if now >= next_state:
                if publish:
                    self.publish_test_streams()
                else:
                    self.global_state.publish(String(data=json.dumps({'active':self.global_active})))
                next_state = now + .2
            rclpy.spin_once(self.node, timeout_sec=.01)
            self.collect_mission_results()
            if predicate():
                return True
        return False

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.assertTrue(self.spin_both(lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('boot_id', 0) != 0 and
            all(any(s.get('session') == session and s.get('ready') and not s.get('pc_timeout')
                    for s in self.statuses[name]) for name, session in
                (('owner','ros-ack-test'), ('viewer','observer-b'))), 10.0))

        self.assertTrue(self.spin_both(lambda: all(
            any(stream == 'local_grid' and len(data.get('packed_cells', [])) == 1024
                for stream, data in self.streams[name]) and
            any(stream == 'navigation_task' and data.get('session') == session and
                len(data.get('state', {}).get('route', [])) == 120
                for stream, data in self.streams[name])
            for name, session in (('owner','ros-ack-test'), ('viewer','observer-b'))),
            8.0, publish=True))

        self.fake.send_image(1)
        self.assertTrue(self.spin_both(lambda: bool(set(self.video_frames['owner']) &
                                                   set(self.video_frames['viewer'])), 3.0))
        frame = next(iter(set(self.video_frames['owner']) & set(self.video_frames['viewer'])))
        jpeg = self.video_frames['owner'][frame]
        self.assertEqual(jpeg, self.video_frames['viewer'][frame])
        self.assertTrue(jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9'))

        self.assertTrue(self.spin_both(lambda: all(any(
            s.get('control_source') == 'pc' and (s.get('last_ack') or {}).get('pc_session') == 'ros-ack-test'
            for s in self.statuses[name]) for name in ('owner','viewer')), 6.0, manual=True))

        # A releases first. The first status after B's claim must already say
        # B owns control, or a pending web arm would discard this valid claim.
        self.send_monitor_heartbeat()
        self.assertTrue(self.spin_both(lambda: self.statuses['owner'][-1].get('pc_control_owner') is False and
            self.statuses['owner'][-1].get('control_source') != 'pc' and
            self.statuses['viewer'][-1].get('ready') is True, 4.0))
        before = len(self.statuses['viewer'])
        claim_seq = self.viewer_seq
        self.send_viewer_heartbeat(manual=True, claim=True)
        self.assertTrue(self.spin_both(lambda: any(
            s.get('pc_control_owner') is True and s.get('pc_seq', -1) >= claim_seq and
            (s.get('last_ack') or {}).get('pc_session') == 'observer-b'
            for s in self.statuses['viewer'][before:]), 5.0, viewer_manual=True))
        first = next(s for s in self.statuses['viewer'][before:] if s.get('pc_seq', -1) >= claim_seq)
        self.assertTrue(first.get('pc_control_owner'), f'claim status arrived before handoff completed: {first}')

        self.send_viewer_heartbeat()
        self.assertTrue(self.spin_both(lambda: self.statuses['viewer'][-1].get('pc_control_owner') is False and
            self.statuses['viewer'][-1].get('control_source') != 'pc' and
            self.statuses['owner'][-1].get('ready') is True and
            self.statuses['owner'][-1].get('auto_acknowledged') is True, 4.0))
        # Hold the mode ACK following A's new zero-speed ACK so the task is
        # submitted while manual-mode confirmation is genuinely outstanding.
        self.mode_ack.hold.set()
        self.assertTrue(self.spin_both(lambda: self.statuses['owner'][-1].get('pc_control_owner') is True and
            (self.statuses['owner'][-1].get('last_ack') or {}).get('pc_session') == 'ros-ack-test' and
            (self.statuses['owner'][-1].get('last_ack') or {}).get('type') == 3 and
            self.statuses['owner'][-1].get('ready') is True and
            self.statuses['owner'][-1].get('auto_acknowledged') is False and
            self.statuses['owner'][-1].get('emergency_stop') is False,
            5.0, manual=True), f"zero-speed ACK did not precede manual-mode ACK: {self.statuses['owner'][-1]}")

        start_seq = self.pc_seq
        request = self.send_mission('local_start', task_id=1, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=.5, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_both(lambda: self.mode_ack.held.is_set() and self.test_status and
            self.test_status.get('pc_seq', -1) >= start_seq, 1.0),
            'the bridge must receive the task while the fake FPGA holds its manual-mode ACK')
        self.assertNotIn(request, self.mission_results, 'the task must wait without a success or failure reply')
        self.assertFalse(self.nav_active, 'no local motion may start before the manual-mode ACK')
        self.mode_ack.release()
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 6.0))
        self.assertTrue(self.mission_results[request]['success'], self.mission_results[request])
        self.assertTrue(self.spin_both(lambda: self.nav_active, 3.0))
        previous_actions = list(self.nav_actions)
        rejected = {'v':1, 'type':'mission_command', 'session':'observer-b', 'seq':self.viewer_seq,
                    'request_id':'observer-overlap', 'action':'local_start', 'task_id':2,
                    'mode':'cooperative', 'allow_disconnect':False, 'goal_x_m':.4, 'goal_y_m':0.0,
                    'duration_s':10.0, 'claim_nonce':self.statuses['viewer'][-1]['claim_nonce']}
        self.viewer_seq += 1
        self.viewer.sendto(json.dumps(rejected).encode(), ('127.0.0.1', fixture.fixture.TEST_PORTS['pc']))
        self.assertTrue(self.spin_both(lambda: 'observer-overlap' in self.results, 3.0))
        self.assertFalse(self.results['observer-overlap']['success'])
        self.spin_both(lambda: False, .7)
        self.assertTrue(self.nav_active, 'observer heartbeat must not replace the navigation owner')
        self.assertEqual(self.nav_actions, previous_actions)

        self.assertTrue(self.spin_both(lambda: not self.nav_active, 3.0, owner=False),
                        'another observer must not renew a missing navigation owner')
        self.assertIn((3, 0), self.nav_actions)
        self.assertTrue(self.spin_both(lambda: self.test_status is not None and
            self.test_status.get('ready') and self.test_status.get('auto_acknowledged'), 4.0))
        request = self.send_mission('local_start', task_id=2, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=.5, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 6.0))
        self.assertTrue(self.mission_results[request]['success'])
        stop = {'v':1,'type':'navigation_task','session':'observer-b','seq':self.viewer_seq,
                'request_id':'observer-stop','action':'stop'}
        self.viewer_seq += 1
        self.viewer.sendto(json.dumps(stop).encode(), ('127.0.0.1', fixture.fixture.TEST_PORTS['pc']))
        self.assertTrue(self.spin_both(lambda: 'observer-stop' in self.results and not self.nav_active, 5.0))
        self.assertTrue(self.results['observer-stop']['success'])
        self.assertTrue(self.results['observer-stop']['unknown'],
                        'local stop is immediate while the global coordinator is unavailable')
        self.assertTrue(self.spin_both(lambda: self.statuses['owner'][-1].get('global_cancel_pending') is True, 3.0))
        request = self.send_mission('local_start', task_id=3, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=.5, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 3.0))
        self.assertFalse(self.mission_results[request]['success'])
        self.assertIn('全局取消', self.mission_results[request]['message'])
        cancellations = []
        cancellation_finished = {'value':False}
        def accept_global_cancel(request, response):
            cancellations.append(json.loads(request.request_json))
            # A queued/old inactive topic cannot establish that this particular
            # accepted cancel has finished.
            self.global_active = False
            response.success, response.unknown, response.message = True, False, 'cancelled'
            response.state_json = json.dumps({'active':not cancellation_finished['value']})
            return response
        self.global_service = self.node.create_service(GlobalNavigation, '/jetson/global_navigation/command',
                                                       accept_global_cancel)
        self.assertTrue(self.spin_both(lambda: bool(cancellations), 5.0))
        self.spin_both(lambda: False, .6)
        self.assertTrue(self.statuses['owner'][-1].get('global_cancel_pending'),
                        'an accepted cancel and stale inactive topic cannot clear the fence')
        cancellation_finished['value'] = True
        self.assertTrue(self.spin_both(lambda: cancellations and
            self.statuses['owner'][-1].get('global_cancel_pending') is False, 5.0))
        self.assertEqual(cancellations[-1]['action'], 'cancel')

        nonce_before_ros_stop = self.statuses['owner'][-1]['claim_nonce']
        cancel_count = len(cancellations)
        self.stop_publisher.publish(Bool(data=True))
        self.assertTrue(self.spin_both(lambda: self.statuses['owner'] and
            self.statuses['owner'][-1].get('emergency_stop') is True and
            self.statuses['owner'][-1].get('claim_nonce') != nonce_before_ros_stop and
            len(cancellations) > cancel_count, 3.0))
        delayed = {'v':1, 'type':'mission_command', 'session':'ros-ack-test', 'seq':self.pc_seq,
                   'request_id':'delayed-release', 'action':'release_emergency_stop',
                   'claim_nonce':nonce_before_ros_stop}
        self.pc_seq += 1
        self.pc.sendto(json.dumps(delayed).encode(), ('127.0.0.1', fixture.fixture.TEST_PORTS['pc']))
        self.assertTrue(self.spin_both(lambda: 'delayed-release' in self.mission_results, 3.0))
        self.assertFalse(self.mission_results['delayed-release']['success'])
        self.assertIn('会话或序号', self.mission_results['delayed-release']['message'])
        self.assertTrue(self.statuses['owner'][-1].get('emergency_stop'))

        release = {'v':1, 'type':'mission_command', 'session':'ros-ack-test', 'seq':self.pc_seq,
                   'request_id':'queued-release', 'action':'release_emergency_stop',
                   'claim_nonce':self.statuses['owner'][-1]['claim_nonce']}
        self.pc_seq += 1
        urgent = {'v':1, 'type':'mission_command', 'session':'ros-ack-test', 'seq':self.pc_seq,
                  'request_id':'queued-estop', 'action':'emergency_stop'}
        self.pc_seq += 1
        target = ('127.0.0.1', fixture.fixture.TEST_PORTS['pc'])
        self.pc.sendto(json.dumps(release).encode(), target)
        self.pc.sendto(json.dumps(urgent).encode(), target)
        self.assertTrue(self.spin_both(lambda: 'queued-release' in self.mission_results and
                                       'queued-estop' in self.mission_results, 3.0))
        self.assertTrue(self.mission_results['queued-release']['success'])
        self.assertTrue(self.mission_results['queued-estop']['success'])
        self.assertTrue(self.spin_both(lambda: self.statuses['owner'][-1].get('emergency_stop') is True, 3.0))

        self.assertTrue(self.spin_both(lambda: self.statuses['owner'][-1].get('global_cancel_pending') is False, 5.0))
        again = {'v':1, 'type':'mission_command', 'session':'ros-ack-test', 'seq':self.pc_seq,
                 'request_id':'web-release-before-ros-stop', 'action':'release_emergency_stop',
                 'claim_nonce':self.statuses['owner'][-1]['claim_nonce']}
        self.pc_seq += 1
        self.pc.sendto(json.dumps(again).encode(), target)
        self.assertTrue(self.spin_both(lambda: 'web-release-before-ros-stop' in self.mission_results, 3.0))
        self.assertTrue(self.mission_results['web-release-before-ros-stop']['success'])
        cancel_count = len(cancellations)
        self.stop_publisher.publish(Bool(data=True))
        self.assertTrue(self.spin_both(lambda: len(cancellations) > cancel_count and
            self.statuses['owner'][-1].get('emergency_stop') is True, 3.0))
        self.spin_both(lambda: False, .3)
        self.assertTrue(self.statuses['owner'][-1].get('emergency_stop'),
                        'an old bridge-published release must not undo an external stop')


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.fixture.TestCleanExit):
    pass

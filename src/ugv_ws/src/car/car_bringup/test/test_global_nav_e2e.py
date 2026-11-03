"""A browser goal goes through the real global coordinator to a loopback FPGA."""
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


spec = importlib.util.spec_from_file_location(
    'global_bridge_fixture', Path(__file__).with_name('test_bx72_bidirectional.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
fixture.TEST_ENV.update(ROS_DOMAIN_ID=os.environ.get('UGV_TEST_ROS_DOMAIN_ID', '212'),
                        ROS_LOCALHOST_ONLY='1')


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.make_test_description(ros_control_enabled=True)
    fixture.TEST_FAKE.navigation['flags'] = 0x26  # FPGA nav_grid_v1
    node = launch_ros.actions.Node(package='car_navigation',
        executable='jetson_global_navigation_node.py', additional_env=fixture.TEST_ENV,
        output='screen')
    description.add_action(node)
    context['global_navigation_process'] = node
    return description, context


class TestGlobalNavigation(fixture.TestBx72Bidirectional):
    def setUp(self):
        super().setUp()
        self.nav = None
        self.status = None
        self.global_state = None
        self.global_results = {}
        self.stop_publisher = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.web_states = []
        self.snapshot_id = 0
        self.map_x, self.map_revision = 1.0, 1
        test = self

        class MapHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != '/global-map.json' or not test.nav:
                    self.send_error(404)
                    return
                transform = {'x':test.map_x, 'y':-0.5, 'yaw':math.pi / 2}
                document = {'v':1, 'type':'global_mapping', 'protocol':'slam_toolbox_view_v1',
                    'source':'slam_toolbox', 'available':True, 'status':'building',
                    'domain':f"{test.nav['boot_id']}:{test.nav['odom_domain']}",
                    'generation':'test-map', 'revision':test.map_revision,
                    'width':80, 'height':80, 'resolution_m':0.05,
                    'origin':{'x':-1.0,'y':-2.0}, 'cells_rle':[1,6400],
                    'map_from_odom':transform, 'map_from_odom_age_ms':0,
                    'pose':dict(transform)}
                payload = json.dumps(document).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, *_args):
                pass

        # Jetson global-map HTTP address is loopback-only; no hardware or live SLAM is contacted.
        self.map_server = HTTPServer(('127.0.0.1', 8766), MapHandler)
        self.map_thread = threading.Thread(target=self.map_server.serve_forever, daemon=True)
        self.map_thread.start()
        self.grid_pub = self.node.create_publisher(
            String, '/navigation/executor/local_grid', qos_profile_sensor_data)
        self.fusion_pub = self.node.create_publisher(
            String, '/fpga/round5/fusion', qos_profile_sensor_data)
        self.node.create_subscription(String, '/navigation/executor/state',
                                      self.on_navigation, qos_profile_sensor_data)
        self.node.create_subscription(String, '/jetson/global_navigation/state',
                                      self.on_global_state, 10)
        self.node.create_timer(0.08, self.publish_observations)

    def tearDown(self):
        self.map_server.shutdown()
        self.map_thread.join(timeout=2.0)
        self.map_server.server_close()
        super().tearDown()

    def on_navigation(self, message):
        self.nav = json.loads(message.data)

    def on_global_state(self, message):
        self.global_state = json.loads(message.data)

    def publish_observations(self):
        if not self.nav or not self.nav.get('boot_id'):
            return
        now = self.nav['now_us']
        stamp = (now - 1000) & 0xffffffff
        fusion = {'flags': 1, 'pose_stamp_us': stamp,
                  'state': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}}
        self.fusion_pub.publish(String(data=json.dumps(fusion)))
        self.snapshot_id += 1
        grid = {'type':'local_grid', 'protocol':'nav_grid_snapshot_v1', 'source':'fpga',
                'sequence':self.snapshot_id, 'snapshot_id':self.snapshot_id,
                'time_us':now, 'capture_begin_us':stamp, 'capture_end_us':now,
                'boot_id':self.nav['boot_id'], 'odom_domain':self.nav['odom_domain'],
                'grid_generation':self.snapshot_id, 'scan_generation':self.snapshot_id,
                'complete_scan_stamp_us':stamp, 'last_ray_stamp_us':stamp,
                'grid_frame_epoch':1, 'transform_revision':1, 'flags':7,
                'width':64, 'height':64, 'resolution_mm':50,
                'origin_x_cell':-32, 'origin_y_cell':-32,
                'grid_from_odom':{'map_anchor_x_mm':0, 'map_anchor_y_mm':0,
                                  'odom_anchor_x_mm':0, 'odom_anchor_y_mm':0,
                                  'yaw_offset_mrad':0},
                'packed_cells':[0x55] * 1024}
        self.grid_pub.publish(String(data=json.dumps(grid)))

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
            if message.get('type') == 'navigation_task_result':
                self.global_results[message.get('request_id')] = message
            elif (message.get('type') == 'telemetry' and message.get('stream') == 'navigation_task' and
                  message.get('part') is None and message.get('data', {}).get('session') == 'ros-ack-test'):
                self.web_states.append(message['data'].get('state'))
            elif message.get('type') == 'status':
                self.status = message

    def send_global(self, action, **fields):
        request_id = f'global-{self.pc_seq}'
        request = {'v':1, 'type':'navigation_task', 'session':'ros-ack-test',
                   'seq':self.pc_seq, 'request_id':request_id, 'action':action, **fields}
        if action == 'start':
            request['claim_nonce'] = (self.status or {}).get('claim_nonce')
        self.pc_seq += 1
        self.pc.sendto(json.dumps(request, separators=(',', ':')).encode(),
                       ('127.0.0.1', fixture.TEST_PORTS['pc']))
        return request_id

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.fake.reack_task_duplicate = True
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.nav is not None and self.nav.get('protocol') == 'nav_grid_v1' and
            self.nav.get('boot_id', 0) != 0 and self.status is not None and
            self.status.get('ready') and self.status.get('auto_acknowledged') and
            self.global_state is not None, timeout=10.0),
            f'global planner or bridge is not ready: nav={self.nav}, status={self.status}')
        request = self.send_global('start', frame='odom',
            domain=f"{self.nav['boot_id']}:{self.nav['odom_domain']}", generation=None,
            goals=[{'x':0.5, 'y':0.2}], max_speed_mm_s=200, max_accel_mm_s2=200,
            tolerance_mm=100, yaw_tolerance_mrad=100, duration_s=10.0)
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: request in self.global_results, timeout=5.0), 'global command did not reply')
        self.assertTrue(self.global_results[request]['success'], self.global_results[request])
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.fake.accepted_tasks == [1] and self.global_state is not None and
            self.global_state.get('status') == 'following', timeout=7.0),
            f'the FPGA did not confirm the waypoint: {self.global_state}')
        task = b''.join(packet[21:30] for packet in self.fake.delivered_task_fragments[:8])
        self.assertEqual(struct.unpack_from('!ii', task, 16), (32768, 13107))
        self.assertTrue(task[3] & 2, 'global waypoints use position-only completion')
        self.assertEqual(task[2], 3, 'the active task survives browser disconnect')

        # ROS authorization during a global task first stops it and reports
        # that authorization must be requested again after confirmation.
        control = self.node.create_client(SetBool, '/control/enable_ros')
        self.assertTrue(control.wait_for_service(timeout_sec=3.0))
        pending = control.call_async(SetBool.Request(data=True))
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: pending.done(), timeout=5.0), 'control authorization did not reply')
        self.assertFalse(pending.result().success)
        self.assertIn('global navigation is stopping', pending.result().message)
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.global_state is not None and self.global_state.get('status') == 'cancelled' and
            not self.global_state.get('active') and self.status.get('global_cancel_pending') is False,
            timeout=7.0),
            f'the FPGA cancellation was not confirmed: {self.global_state}')

        # A non-identity map transform must be applied by Jetson, and a later
        # map correction must cancel the old segment before issuing another.
        offset = len(self.fake.delivered_task_fragments)
        request = self.send_global('start', frame='map',
            domain=f"{self.nav['boot_id']}:{self.nav['odom_domain']}", generation='test-map',
            goals=[{'x':0.8,'y':0.0}], max_speed_mm_s=200, max_accel_mm_s2=200,
            tolerance_mm=100, yaw_tolerance_mrad=100, duration_s=10.0)
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: request in self.global_results, timeout=5.0), 'map goal did not reply')
        self.assertTrue(self.global_results[request]['success'], self.global_results[request])
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: 3 in self.fake.accepted_tasks and self.global_state.get('status') == 'following',
            timeout=8.0), f'map goal was not dispatched: {self.global_state}')
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: any(state and state.get('frame') == 'map' and state.get('active') and
                        state.get('goals') == [{'x':0.8,'y':0.0}] for state in self.web_states),
            timeout=3.0), 'the bridge did not forward the real map goal state to the website')
        task = b''.join(packet[21:30] for packet in self.fake.delivered_task_fragments[offset:offset + 8])
        self.assertEqual(struct.unpack_from('!ii', task, 16), (32768, 13107))

        self.map_x, self.map_revision = 1.1, 2
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: 5 in self.fake.accepted_tasks and self.global_state.get('status') == 'following',
            timeout=10.0), f'map correction did not produce a new segment: {self.global_state}')
        fragments = self.fake.delivered_task_fragments[offset:]
        transactions = [b''.join(packet[21:30] for packet in fragments[i:i+8])
                        for i in range(0, len(fragments) - 7, 8)]
        self.assertEqual([(data[1], struct.unpack_from('!I',data,4)[0]) for data in transactions],
                         [(1,3),(3,4),(1,5)], 'old segment must stop before the new segment')
        self.assertEqual(struct.unpack_from('!ii', transactions[-1], 16), (32768, 19661))

        # ROS emergency stop must cancel the active global goal, even if it is
        # released quickly; a later tick must not dispatch the old next segment.
        self.stop_publisher.publish(Bool(data=True))
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.status.get('emergency_stop') is True and self.global_state is not None and
            self.global_state.get('status') == 'cancelled' and not self.global_state.get('active') and
            self.status.get('global_cancel_pending') is False,
            timeout=7.0), f'emergency stop did not cancel the global goal: {self.global_state}')
        accepted = list(self.fake.accepted_tasks)
        self.stop_publisher.publish(Bool(data=False))
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.status.get('emergency_stop') is False, timeout=3.0))
        self.spin_with_monitor_heartbeat(lambda: False, timeout=1.0)
        self.assertFalse(self.global_state.get('active'))
        self.assertEqual(self.fake.accepted_tasks, accepted)


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.TestCleanExit):
    def test_global_exit_code(self, proc_info, global_navigation_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=global_navigation_process)

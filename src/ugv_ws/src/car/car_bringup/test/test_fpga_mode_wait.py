"""A retried Jetson cancel cannot block an FPGA-local start."""
import importlib.util
from pathlib import Path
import socket
import threading

import launch_testing
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ugv_bridge.srv import NavigationCommand


spec = importlib.util.spec_from_file_location(
    'multimonitor_fpga_fixture', Path(__file__).with_name('test_multimonitor.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.generate_test_description()
    fixture.fixture.fixture.TEST_FAKE.navigation['flags'] = 0x26  # FPGA nav_grid_v1
    return description, context


class TestFpgaModeWait(fixture.TestMultiMonitor):
    def setUp(self):
        # Base fixture only: expose a separate callback thread to hold the
        # background Jetson cancellation while the FPGA request is pending.
        fixture.fixture.fixture.TestBx72Bidirectional.setUp(self)
        self.test_navigation = self.test_status = None
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
        self.global_active = False
        self.mode_ack = fixture.ModeAckGate(self.fake.sock, self.fake.ack_target)
        self.fake.sock = self.mode_ack
        self.cancel_entered, self.release_cancel, self.cancel_replied = (threading.Event() for _ in range(3))
        self.cancel_count = 0
        self.service_node = rclpy.create_node('held_jetson_cancel')
        self.group = ReentrantCallbackGroup()
        self.service = self.service_node.create_service(
            NavigationCommand, '/jetson/navigation/command', self.on_background_cancel,
            callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.service_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.release_cancel.set()
        self.executor.shutdown(timeout_sec=3)
        self.executor_thread.join(timeout=3)
        self.service_node.destroy_node()
        super().tearDown()

    def on_background_cancel(self, request, response):
        if request.action == 3 and request.task_id == 0:
            self.cancel_count += 1
            if self.cancel_count >= 2:
                # Reaching the second call proves the bridge processed the
                # first failure and scheduled its retry.
                self.cancel_entered.set()
                self.release_cancel.wait(timeout=5)
                self.cancel_replied.set()
        response.success, response.message = False, 'simulated stale Jetson cancel failure'
        return response

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.assertTrue(self.spin_both(lambda: self.cancel_entered.is_set() and self.latest_navigation() and
            self.latest_navigation().get('protocol') == 'nav_grid_v1' and self.test_status and
            self.test_status.get('ready') is True and self.test_status.get('auto_acknowledged') is True, 10.0))
        self.mode_ack.hold.set()
        self.assertTrue(self.spin_both(lambda: self.test_status.get('pc_control_owner') is True and
            (self.test_status.get('last_ack') or {}).get('type') == 3 and
            self.test_status.get('auto_acknowledged') is False, 5.0, manual=True))
        start_seq = self.pc_seq
        request = self.send_mission('local_start', task_id=1, mode='cooperative',
            allow_disconnect=False, goal_x_m=.5, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_both(lambda: self.mode_ack.held.is_set() and self.test_status and
            self.test_status.get('pc_seq', -1) >= start_seq, 1.0))
        self.assertNotIn(request, self.mission_results,
                         'the FPGA task must wait for its own mode ACK')
        self.assertEqual(self.fake.accepted_tasks, [])
        self.mode_ack.release()
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 6.0))
        self.assertTrue(self.mission_results[request]['success'], self.mission_results[request])
        self.assertTrue(self.spin_both(lambda: 1 in self.fake.accepted_tasks, 3.0))
        self.assertFalse(self.cancel_replied.is_set(), 'FPGA start cannot wait for the Jetson retry')
        self.release_cancel.set()
        self.assertTrue(self.spin_both(lambda: self.cancel_replied.is_set(), 2.0))


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.fixture.fixture.TestCleanExit):
    pass

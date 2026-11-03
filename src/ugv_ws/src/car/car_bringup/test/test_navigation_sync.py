"""The real bridge must finish an old-task cancellation before a new start."""
import importlib.util
import json
import os
from pathlib import Path
import threading

import launch_testing
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ugv_bridge.srv import NavigationCommand


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
    fixture.TEST_FAKE.navigation['flags'] = 0x16  # velocity_only_v1
    return description, context


class TestNavigationSync(fixture.TestBx72Bidirectional):
    def setUp(self):
        super().setUp()
        self.status = None
        self.cancel_entered = threading.Event()
        self.release_cancel = threading.Event()
        self.cancel_finished = threading.Event()
        self.start_seen = threading.Event()
        self.out_of_order = threading.Event()
        self.service_node = rclpy.create_node('delayed_navigation_service')
        self.group = ReentrantCallbackGroup()
        self.service = self.service_node.create_service(
            NavigationCommand, '/jetson/navigation/command', self.on_navigation,
            callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.service_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.fake.valid_ack.set()

    def tearDown(self):
        self.release_cancel.set()
        self.executor.shutdown(timeout_sec=3)
        self.executor_thread.join(timeout=3)
        self.service_node.destroy_node()
        super().tearDown()

    def on_navigation(self, request, response):
        if request.action == 3 and request.task_id == 0:
            self.cancel_entered.set()
            self.release_cancel.wait(timeout=8)
            self.cancel_finished.set()
        elif request.action == 1:
            if not self.cancel_finished.is_set():
                self.out_of_order.set()
            self.start_seen.set()
        response.success, response.message = True, 'accepted'
        return response

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

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.cancel_entered.is_set() and self.latest_navigation() is not None and
            self.latest_navigation().get('boot_id', 0) != 0 and self.status is not None and
            self.status.get('ready') and self.status.get('auto_acknowledged'), timeout=8.0),
            f'cancel_entered={self.cancel_entered.is_set()}, '
            f'navigation={self.latest_navigation()}, status={self.status}')
        request = self.send_mission('local_start', task_id=1, mode='cooperative',
                                    allow_disconnect=False, goal_x_m=0.5, goal_y_m=0.0,
                                    duration_s=10.0)
        self.assertFalse(self.spin_with_monitor_heartbeat(self.start_seen.is_set, timeout=0.35),
                         'the start must not reach the service while force-cancel is held')
        self.assertNotIn(request, self.mission_results, 'the web start must remain pending')
        self.release_cancel.set()
        result = self.wait_for_mission_result(request, timeout=4.0)
        self.assertTrue(result and result['success'], f'pending start did not resume: {result}')
        self.assertTrue(self.start_seen.is_set())
        self.assertFalse(self.out_of_order.is_set())


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.TestCleanExit):
    pass

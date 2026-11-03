"""Loopback test for a lost FPGA task-fragment ACK between waypoints."""
import importlib.util
import json
import os
import time
from pathlib import Path

import launch_testing
import pytest
from ugv_bridge.srv import NavigationCommand


spec = importlib.util.spec_from_file_location(
    'waypoint_ack_fixture', Path(__file__).with_name('test_bx72_bidirectional.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
fixture.TEST_ENV.update(ROS_DOMAIN_ID=os.environ.get('UGV_TEST_ROS_DOMAIN_ID', '212'),
                        ROS_LOCALHOST_ONLY='1')


@pytest.mark.launch_test
def generate_test_description():
    description, context = fixture.generate_test_description()
    fixture.TEST_FAKE.navigation['flags'] = 0x26  # nav_grid_v1
    return description, context


class TestWaypointAck(fixture.TestBx72Bidirectional):
    def setUp(self):
        super().setUp()
        self.test_navigation = None
        self.test_status = None
        self.status_updates = 0

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
                self.status_updates += 1
            elif message.get('type') == 'telemetry' and message.get('stream') == 'navigation':
                self.test_navigation = message.get('data')

    def latest_navigation(self):
        return self.test_navigation

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.fake.reack_task_duplicate = True
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('protocol') == 'nav_grid_v1' and
            self.latest_navigation().get('boot_id', 0) != 0 and
            self.test_status is not None and self.test_status.get('ready') and
            self.test_status.get('auto_acknowledged'), timeout=8.0),
            f"navigation={self.latest_navigation()}, status={self.test_status}, "
            f"bootstrap_attempts={self.fake.bootstrap_attempts}, events={len(self.fake.snapshot())}")

        invalid = self.send_mission('local_start', task_id=1, mode='cooperative',
                                    position_only='true', duration_s=10.0)
        self.assertFalse(self.wait_for_mission_result(invalid)['success'])
        self.assertEqual(self.fake.accepted_tasks, [], 'invalid input cannot reach the FPGA')
        executor = self.node.create_client(NavigationCommand, '/navigation/executor/command')
        self.assertTrue(self.spin_with_monitor_heartbeat(executor.service_is_ready, timeout=5.0))
        command = NavigationCommand.Request()
        command.action, command.mode, command.flags, command.task_id = 1, 3, 2, 1
        command.boot_id = self.latest_navigation()['boot_id']
        command.odom_domain = self.latest_navigation()['odom_domain']
        command.goal_x_m, command.goal_y_m = 0.5, 0.0
        command.max_speed_mm_s, command.max_accel_mm_s2 = 200, 200
        command.tolerance_mm, command.yaw_tolerance_mrad = 100, 100
        command.duration_s = 10.0
        first = executor.call_async(command)
        self.assertTrue(self.spin_with_monitor_heartbeat(first.done, timeout=3.0))
        self.assertTrue(first.result().success, first.result().message)
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('task_id') == 1 and
            self.latest_navigation().get('active_task'), timeout=5.0))
        events_before_probe = len(self.fake.snapshot())
        self.assertTrue(self.spin_with_monitor_heartbeat(
            lambda: any(packet[5] == 1 for _, packet in self.fake.snapshot()[events_before_probe:]),
            timeout=1.0))
        self.assertFalse(any(packet[5] == 3 for _, packet in self.fake.snapshot()[events_before_probe:]),
                         'FPGA-owned execution must not be overwritten by a velocity stream')

        def arrive(task_id):
            self.assertTrue(self.spin_with_monitor_heartbeat(
                lambda: self.latest_navigation() is not None and
                self.latest_navigation().get('task_id') == task_id and
                self.latest_navigation().get('active_task'), timeout=5.0))
            with self.fake.lock:
                self.fake.navigation.update(status=4, mode=1, expiry_us=0, velocity=[0, 0, 0])
            status_start = self.status_updates
            self.assertTrue(self.spin_with_monitor_heartbeat(
                lambda: self.latest_navigation() is not None and
                self.latest_navigation().get('task_id') == task_id and
                self.latest_navigation().get('status') == 4 and
                self.status_updates >= status_start + 2 and self.test_status.get('ready') and
                self.test_status.get('auto_acknowledged'), timeout=5.0))

        arrive(1)

        deliveries_before = len(self.fake.delivered_task_fragments)
        events_before = len(self.fake.snapshot())
        self.fake.drop_task_ack_index = 0
        second = self.send_mission('local_start', task_id=2, mode='cooperative',
                                   allow_disconnect=False, goal_x_m=1.0, goal_y_m=0.0,
                                   position_only=True, duration_s=10.0)
        result = self.wait_for_mission_result(second)
        self.assertTrue(self.fake.dropped_task_ack.is_set(), 'the first task ACK was dropped after FPGA receipt')
        self.assertIsNotNone(result)
        self.assertTrue(result['success'], f"the second waypoint did not recover its lost ACK: {result}")
        fragments = self.fake.delivered_task_fragments[deliveries_before:]
        self.assertEqual([fragment[20] for fragment in fragments], list(range(8)),
                         'each fragment must reach the FPGA task FIFO exactly once')
        self.assertEqual(b''.join(fragment[21:30] for fragment in fragments)[3], 2,
                         'position-only waypoints must set payload flag bit 1')
        sent = [request for _, request in self.fake.snapshot()[events_before:] if request[5] == 7]
        self.assertEqual(sent[:2], [fragments[0], fragments[0]],
                         'a lost ACK is retried with the identical fragment and sequence')

        arrive(2)
        self.fake.dropped_task_ack.clear()
        self.fake.drop_task_ack_index = 7
        deliveries_before = len(self.fake.delivered_task_fragments)
        events_before = len(self.fake.snapshot())
        third = self.send_mission('local_start', task_id=3, mode='cooperative',
                                  allow_disconnect=False, goal_x_m=1.5, goal_y_m=0.0,
                                  duration_s=10.0)
        result = self.wait_for_mission_result(third)
        self.assertTrue(self.fake.dropped_task_ack.is_set())
        self.assertTrue(result['success'], f"the final fragment ACK was not recovered: {result}")
        fragments = self.fake.delivered_task_fragments[deliveries_before:]
        self.assertEqual([fragment[20] for fragment in fragments], list(range(8)))
        sent = [request for _, request in self.fake.snapshot()[events_before:]
                if request[5] == 7 and request[20] == 7]
        self.assertEqual(sent, [fragments[-1], fragments[-1]])
        self.assertEqual(self.fake.accepted_tasks.count(3), 1, 'the final retry cannot execute the task twice')

        arrive(3)
        self.fake.drop_task_request_index = 3
        deliveries_before = len(self.fake.delivered_task_fragments)
        events_before = len(self.fake.snapshot())
        fourth = self.send_mission('local_start', task_id=4, mode='cooperative',
                                   allow_disconnect=False, goal_x_m=2.0, goal_y_m=0.0,
                                   duration_s=10.0)
        result = self.wait_for_mission_result(fourth)
        self.assertTrue(result['success'], f"the lost fragment request was not recovered: {result}")
        fragments = self.fake.delivered_task_fragments[deliveries_before:]
        self.assertEqual([fragment[20] for fragment in fragments], list(range(8)))
        sent = [request for _, request in self.fake.snapshot()[events_before:]
                if request[5] == 7 and request[20] == 3]
        self.assertEqual(sent, [fragments[3], fragments[3]])

        arrive(4)
        self.fake.reack_task_duplicate = False
        self.fake.drop_task_ack_index = 0
        started = time.monotonic()
        fifth = self.send_mission('local_start', task_id=5, mode='cooperative',
                                  allow_disconnect=False, goal_x_m=2.5, goal_y_m=0.0,
                                  duration_s=10.0)
        result = self.wait_for_mission_result(fifth)
        self.assertFalse(result['success'])
        self.assertTrue(result['unknown'])
        self.assertLess(time.monotonic() - started, 1.0, 'a missing ACK must not retry indefinitely')


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.TestCleanExit):
    pass

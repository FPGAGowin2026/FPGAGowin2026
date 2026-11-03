"""Pending local starts are cancelled on takeover or a missing FPGA mode ACK."""
import importlib.util
from pathlib import Path

import launch_testing
import pytest


spec = importlib.util.spec_from_file_location(
    'multimonitor_fixture', Path(__file__).with_name('test_multimonitor.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    return fixture.generate_test_description()


class TestNavigationModeWait(fixture.TestMultiMonitor):
    def wait_manual_zero_before_mode(self):
        self.mode_ack.held.clear()
        self.mode_ack.hold.set()
        return self.spin_both(lambda: self.statuses['owner'] and
            self.statuses['owner'][-1].get('pc_control_owner') is True and
            (self.statuses['owner'][-1].get('last_ack') or {}).get('type') == 3 and
            (self.statuses['owner'][-1].get('last_ack') or {}).get('pc_session') == 'ros-ack-test' and
            self.statuses['owner'][-1].get('ready') is True and
            self.statuses['owner'][-1].get('auto_acknowledged') is False, 5.0, manual=True)

    def start_while_mode_ack_is_held(self):
        start_seq = self.pc_seq
        request = self.send_mission('local_start', task_id=1, mode='cooperative',
            allow_disconnect=False, goal_x_m=.5, goal_y_m=0.0, duration_s=10.0)
        self.assertTrue(self.spin_both(lambda: self.mode_ack.held.is_set() and self.test_status and
            self.test_status.get('pc_seq', -1) >= start_seq, 1.0))
        self.assertNotIn(request, self.mission_results)
        self.assertFalse(self.nav_active)
        return request

    def test_no_pc_ros_streams_and_matching_speed_ack(self):
        self.fake.valid_ack.set()
        self.assertTrue(self.spin_both(lambda: self.latest_navigation() is not None and
            self.latest_navigation().get('boot_id', 0) != 0 and self.test_status is not None and
            self.test_status.get('ready') is True and self.test_status.get('auto_acknowledged') is True and
            self.statuses['viewer'] and self.statuses['viewer'][-1].get('claim_nonce'), 10.0))

        self.assertTrue(self.wait_manual_zero_before_mode())
        request = self.start_while_mode_ack_is_held()
        self.send_viewer_heartbeat(manual=True, claim=True)
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 2.0))
        result = self.mission_results[request]
        self.assertFalse(result['success'], result)
        self.assertIn('停车或人工接管', result['message'])
        self.assertFalse(any(action == 1 for action, _ in self.nav_actions))
        self.mode_ack.release()

        # Let the real bridge's zero/mode fence finish, then make another
        # explicit handoff and withhold its mode ACK until the transport timeout.
        self.spin_both(lambda: False, .4)
        self.assertTrue(self.spin_both(lambda: self.test_status and self.test_status.get('ready') is True and
            self.test_status.get('auto_acknowledged') is True, 4.0))
        self.assertTrue(self.wait_manual_zero_before_mode())
        request = self.start_while_mode_ack_is_held()
        self.assertTrue(self.spin_both(lambda: request in self.mission_results, 2.0))
        result = self.mission_results[request]
        self.assertFalse(result['success'], result)
        self.assertIn('启动未发送', result['message'])
        self.assertFalse(any(action == 1 for action, _ in self.nav_actions))
        self.mode_ack.release()


@launch_testing.post_shutdown_test()
class TestCleanExit(fixture.fixture.fixture.TestCleanExit):
    pass

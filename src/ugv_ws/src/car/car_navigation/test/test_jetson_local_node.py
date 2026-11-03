"""Minimal ROS contract check of the actual Jetson node, with no physical hardware."""

import json
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ugv_bridge.msg import AuthorizedTwist
from ugv_bridge.srv import NavigationCommand


TEST_DOMAIN = int(os.environ.get('UGV_TEST_ROS_DOMAIN_ID', str(105 + os.getpid() % 35)))
TEST_ENV = {'ROS_DOMAIN_ID': str(TEST_DOMAIN), 'ROS_LOCALHOST_ONLY': '1'}


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='car_navigation', executable='jetson_local_navigation',
        additional_env=TEST_ENV, output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestJetsonLocalNode(unittest.TestCase):
    def setUp(self):
        os.environ.update(TEST_ENV)
        rclpy.init(domain_id=TEST_DOMAIN)
        self.node = rclpy.create_node('jetson_local_navigation_contract')
        self.nav_pub = self.node.create_publisher(String, '/fpga/round5/navigation', qos_profile_sensor_data)
        self.fusion_pub = self.node.create_publisher(String, '/fpga/round5/fusion', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, '/odom/filtered', qos_profile_sensor_data)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.states, self.maps, self.velocities = [], [], []
        self.node.create_subscription(String, '/jetson/navigation/state',
                                      lambda m: self.states.append(json.loads(m.data)), 10)
        self.node.create_subscription(String, '/jetson/navigation/local_grid',
                                      lambda m: self.maps.append(json.loads(m.data)), 10)
        self.node.create_subscription(AuthorizedTwist, '/jetson/navigation/cmd_vel',
                                      lambda m: self.velocities.append((time.monotonic(), m)), 10)
        self.client = self.node.create_client(NavigationCommand, '/jetson/navigation/command')
        self.started = time.monotonic()
        self.count = 0
        self.guard_stop = False
        self.node.create_timer(0.02, self.publish_inputs)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def publish_inputs(self):
        source_us = (1_000_000 + int((time.monotonic() - self.started) * 1_000_000)) & 0xffffffff
        nav = {'protocol': 'velocity_only_v1', 'boot_id': 7, 'odom_domain': 1, 'now_us': source_us,
               'pose_valid': True, 'guard_stop': self.guard_stop}
        fusion = {'pose_valid': True, 'pose_stamp_us': source_us, 'scan_match_accepted': 0,
                  'state': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                            'vx_m_s': 0.0, 'vy_m_s': 0.0, 'wz_rad_s': 0.0}}
        self.nav_pub.publish(String(data=json.dumps(nav)))
        self.fusion_pub.publish(String(data=json.dumps(fusion)))

        stamp = self.node.get_clock().now()
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = stamp.to_msg()
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        self.count += 1
        if self.count % 5:
            return
        scan = LaserScan()
        scan.header.frame_id = 'n10_sensor'
        scan.header.stamp = Time(nanoseconds=stamp.nanoseconds - 40_000_000).to_msg()
        scan.angle_min = 0.0
        scan.angle_increment = math.tau / 72.0
        scan.angle_max = scan.angle_increment * 71
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = 0.05, 8.0
        # Forward free corridor and a rear body return, matching the legacy C fixture.
        scan.ranges = [6.0 if i < 18 or i > 54 else 0.265 for i in range(72)]
        self.scan_pub.publish(scan)

    def wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return bool(predicate())

    def command(self, action, task_id, epoch=101, expired=False, issued_ns=None,
                bridge='contract-bridge', rejected=False, speed=200):
        request = NavigationCommand.Request()
        request.action, request.task_id = action, task_id
        request.mode, request.flags = 3, 2
        request.boot_id, request.odom_domain = 7, 1
        request.goal_x_m, request.goal_y_m = 2.0, 0.0
        request.hold_yaw_rad, request.final_yaw_rad = 0.0, 0.0
        request.max_speed_mm_s, request.max_accel_mm_s2 = speed, 500
        request.tolerance_mm, request.yaw_tolerance_mrad = 100, 100
        request.duration_s = 20.0
        request.bridge_id, request.authorization_epoch = bridge, epoch
        request.issued_steady_ns = time.monotonic_ns() if issued_ns is None else issued_ns
        request.start_deadline_steady_ns = time.monotonic_ns() + (-1 if expired else 2_000_000_000)
        future = self.client.call_async(request)
        self.assertTrue(self.wait_for(future.done), 'navigation service did not respond')
        self.assertEqual(future.result().success, not (expired or rejected), future.result().message)

    def latest_state(self, task, last, status):
        return any(s.get('task_id') == task and s.get('last_task_id') == last and
                   s.get('status') == status for s in self.states)

    @staticmethod
    def moving(message):
        return any(abs(value) > 1e-6 for value in
                   (message.twist.linear.x, message.twist.linear.y, message.twist.angular.z))

    def test_map_authorized_motion_cancel_and_stop(self):
        self.assertTrue(self.wait_for(self.client.service_is_ready))
        self.assertTrue(self.wait_for(lambda: self.latest_state(0, 0, 0)))
        self.assertTrue(self.wait_for(lambda: any(
            grid.get('protocol') == 'jetson_grid_snapshot_v1' and grid.get('source') == 'jetson' and
            grid.get('boot_id') == 7 and grid.get('odom_domain') == 1 and
            grid.get('scan_generation', 0) > 0 and (grid.get('flags', 0) & 6) == 6 and
            len(grid.get('packed_cells', [])) == 1024 for grid in self.maps)),
            'valid pose and complete scans must produce a ready local map')

        old_issue = time.monotonic_ns()
        self.command(3, 0, epoch=100, bridge='new-bridge')
        self.command(1, 1, epoch=100, bridge='old-bridge', issued_ns=old_issue, rejected=True)
        self.command(1, 1, expired=True)
        self.command(1, 1, issued_ns=time.monotonic_ns() + 10_000_000_000, rejected=True)
        self.command(1, 1)
        self.assertTrue(self.wait_for(lambda: any(
            m.bridge_id == 'contract-bridge' and m.authorization_epoch == 101 and self.moving(m)
            for _, m in self.velocities)), 'accepted waypoint must produce authorized motion')
        self.assertTrue(self.wait_for(lambda: any(
            s.get('task_id') == 1 and s.get('bridge_id') == 'contract-bridge' and
            s.get('authorization_epoch') == 101 and s.get('active_task') for s in self.states)))

        invalid_at = time.monotonic()
        self.command(1, 2, epoch=102, bridge='new-bridge', speed=0, rejected=True)
        self.command(1, 1, epoch=102, bridge='new-bridge', rejected=True)
        self.assertTrue(self.wait_for(lambda: any(
            at > invalid_at and m.bridge_id == 'contract-bridge' and
            m.authorization_epoch == 101 and self.moving(m) for at, m in self.velocities)),
            'invalid or repeated tasks must not cancel or replace the current owner')

        late_cancel_at = time.monotonic()
        self.command(3, 0, epoch=100, bridge='old-bridge', issued_ns=old_issue)
        self.assertTrue(self.wait_for(lambda: any(
            at > late_cancel_at and m.authorization_epoch == 101 and self.moving(m)
            for at, m in self.velocities)), 'a stale cancel must not stop a newer task')

        # 只模拟 kind8 的有效 guard 反馈；hard-kill 和其释放流水期已由 FPGA 屏蔽。
        self.guard_stop = True
        guard_at = time.monotonic()
        self.assertTrue(self.wait_for(lambda: any(
            at > guard_at and m.authorization_epoch == 101 and not self.moving(m)
            for at, m in self.velocities)), 'guard feedback must produce a zero command')
        self.wait_for(lambda: False, timeout=0.3)
        self.assertFalse(any(at > guard_at + 0.1 and self.moving(m)
                             for at, m in self.velocities if m.authorization_epoch == 101))
        self.assertTrue(self.states[-1].get('active_task'), 'guard stop must retain the task')
        self.guard_stop = False
        resumed_at = time.monotonic()
        self.assertTrue(self.wait_for(lambda: any(
            at > resumed_at and m.authorization_epoch == 101 and self.moving(m)
            for at, m in self.velocities)), 'cleared guard must let the same task replan')

        self.command(3, 2)
        self.assertTrue(self.wait_for(lambda: self.latest_state(1, 2, 7)),
                        'cancel reports the task and the newer transaction separately')
        terminal_at = time.monotonic()
        self.wait_for(lambda: False, timeout=0.4)
        self.assertFalse(any(at > terminal_at + 0.05 and self.moving(m)
                             for at, m in self.velocities if m.authorization_epoch == 101))

        self.command(1, 3, epoch=102)
        self.assertTrue(self.wait_for(lambda: any(
            m.authorization_epoch == 102 and self.moving(m) for _, m in self.velocities)))
        self.command(2, 4, epoch=102)
        self.assertTrue(self.wait_for(lambda: self.latest_state(3, 4, 7)))
        self.assertTrue(any(m.authorization_epoch == 102 and not self.moving(m)
                            for _, m in self.velocities))

import os
import time
import unittest
from collections import deque

import launch
from launch.events import matches_action
from launch.events.process import ShutdownProcess
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# 每次进程使用独立 DDS domain，并强制 Fast DDS 共享内存传输，隔离同机其他 ROS 图。
# 本测试的 UDP 端口固定为回环地址，因此不访问 FPGA 或外部网络。
TEST_DOMAIN_ID = 32 + os.getpid() % 60
TEST_ENV = {
    'FASTDDS_BUILTIN_TRANSPORTS': 'SHM',
    'ROS_DOMAIN_ID': str(TEST_DOMAIN_ID),
}


@pytest.mark.launch_test
def generate_test_description():
    # 网关与 fake 节点复用实机协议，但只在本机 25000/25001 回环端口形成闭环。
    gateway = launch_ros.actions.Node(
        package='fpga_gateway',
        executable='fpga_gateway_node',
        additional_env=TEST_ENV,
        parameters=[{
            'fpga_ip': '127.0.0.1',
            'bind_ip': '127.0.0.1',
            'control_port': 25000,
            'telemetry_port': 25001,
            'control_rate_hz': 100.0,
            'receive_rate_hz': 500.0,
            'command_timeout_ms': 200,
            'link_timeout_ms': 300,
        }],
    )
    fake_fpga = launch_ros.actions.Node(
        package='fpga_gateway',
        executable='fake_fpga_node',
        additional_env=TEST_ENV,
        parameters=[{
            'listen_ip': '127.0.0.1',
            'jetson_ip': '127.0.0.1',
            'control_port': 25000,
            'telemetry_port': 25001,
            'telemetry_rate_hz': 100.0,
            'command_watchdog_ms': 300,
        }],
    )
    return launch.LaunchDescription([
        gateway,
        fake_fpga,
        launch_testing.actions.ReadyToTest(),
    ]), {'fake_fpga_process': fake_fpga}


class TestFpgaSimControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.update(TEST_ENV)
        if not rclpy.ok():
            rclpy.init(domain_id=TEST_DOMAIN_ID)
        cls._odom = deque(maxlen=512)
        cls._status = deque(maxlen=64)
        cls._node = rclpy.create_node('fpga_sim_control_test')
        cls._cmd_pub = cls._node.create_publisher(Twist, '/cmd_vel', 10)
        cls._estop_pub = cls._node.create_publisher(Bool, '/emergency_stop', 10)
        cls._odom_sub = cls._node.create_subscription(Odometry, '/wheel/odom', cls._on_odom, 200)
        cls._status_sub = cls._node.create_subscription(
            DiagnosticArray, '/fpga/status', cls._on_status, 10)

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def _on_odom(cls, message):
        # 单调时间把里程计样本与命令、急停动作关联，不受系统时钟跳变影响。
        cls._odom.append((time.monotonic(), message))

    @classmethod
    def _on_status(cls, message):
        for status in message.status:
            if status.name == 'fpga_gateway':
                cls._status.append((time.monotonic(), status))

    @classmethod
    def _wait(cls, predicate, timeout, detail):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls._node, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError(detail)

    @classmethod
    def _latest_matching(cls, predicate):
        for stamp, message in reversed(cls._odom):
            if predicate(stamp, message):
                return stamp, message
        return None

    @classmethod
    def _publish_until(cls, publisher, message, predicate, detail, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(cls._node, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError(detail)

    @classmethod
    def _wait_for_status(cls, predicate, timeout, detail):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls._node, timeout_sec=0.01)
            for _, status in reversed(cls._status):
                if predicate(status):
                    return status
        raise AssertionError(detail)

    @staticmethod
    def _twist(linear, angular):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        return message

    @staticmethod
    def _stopped(message):
        return (
            abs(message.twist.twist.linear.x) < 0.02 and
            abs(message.twist.twist.angular.z) < 0.02)

    def test_control_timeout_and_emergency_stop(self, launch_service, fake_fpga_process):
        # 首帧里程计同时证明 ROS 发现、UDP 端点和 fake 遥测链路已就绪。
        self._wait(
            lambda: len(self._odom) > 0,
            20.0, 'wheel odometry was not discovered')

        warmup_deadline = time.monotonic() + 0.25
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(self._node, timeout_sec=0.01)
        self._odom.clear()
        self._wait(
            lambda: len(self._snapshot()) >= 50,
            5.0, 'did not receive enough odometry samples')
        samples = self._snapshot()
        first = samples[0][1].header.stamp
        last = samples[-1][1].header.stamp
        # 用发布端写入的时间戳测量节拍，避免测试执行器的回调吞吐影响结果。
        span = (last.sec - first.sec) + (last.nanosec - first.nanosec) * 1e-9
        self.assertGreater(span, 0.0)
        rate_hz = (len(samples) - 1) / span
        self.assertGreaterEqual(rate_hz, 70.0)
        self.assertLessEqual(rate_hz, 130.0)

        command = self._twist(0.20, 0.40)
        command_start = time.monotonic()
        self._publish_until(
            self._cmd_pub, command,
            lambda: self._matches_twist(0.20, 0.40, command_start),
            'odom did not follow cmd_vel')

        # 停止发布后越过 200 ms 命令超时，里程计必须回到零速度。
        quiet_after = time.monotonic() + 0.25
        self._wait(
            lambda: self._latest_matching(
                lambda stamp, message: stamp >= quiet_after and
                self._stopped(message)) is not None,
            2.0, 'odom did not stop after command timeout')

        recovered = self._twist(0.15, -0.30)
        recovery_start = time.monotonic()
        self._publish_until(
            self._cmd_pub, recovered,
            lambda: self._matches_twist(0.15, -0.30, recovery_start),
            'odom did not recover after a new cmd_vel')

        # 释放急停不会恢复旧命令；只有后续新 cmd_vel 才允许重新运动。
        estop = Bool(data=True)
        estop_start = time.monotonic()
        self._publish_until(
            self._estop_pub, estop,
            lambda: self._latest_matching(
                lambda stamp, message: stamp >= estop_start and
                self._stopped(message)) is not None,
            'odom did not stop after emergency_stop')

        release = Bool(data=False)
        release_after = time.monotonic()
        self._publish_until(
            self._estop_pub, release,
            lambda: self._latest_matching(
                lambda stamp, message: stamp >= release_after and
                self._stopped(message)) is not None,
            'odom did not stay stopped after releasing emergency_stop without cmd_vel')

        final_command_start = time.monotonic()
        self._publish_until(
            self._cmd_pub, command,
            lambda: self._matches_twist(0.20, 0.40, final_command_start),
            'odom did not recover after emergency_stop was released')
        self._wait_for_status(
            lambda status: status.level == DiagnosticStatus.OK,
            3.0, 'gateway did not report an online OK status')
        # 主动终止 fake 进程等价于遥测源消失，验证网关的断链诊断而非硬件拔线。
        launch_service.emit_event(ShutdownProcess(
            process_matcher=matches_action(fake_fpga_process)))
        self._wait_for_status(
            lambda status: status.level == DiagnosticStatus.ERROR and
            status.message == 'FPGA link offline',
            5.0, 'gateway did not report FPGA link offline')

    @classmethod
    def _snapshot(cls):
        return list(cls._odom)

    @classmethod
    def _matches_twist(cls, linear, angular, after):
        return cls._latest_matching(
            lambda stamp, message: stamp >= after and
            abs(message.twist.twist.linear.x - linear) < 0.02 and
            abs(message.twist.twist.angular.z - angular) < 0.02) is not None

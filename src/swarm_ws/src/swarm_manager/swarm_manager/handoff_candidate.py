"""Expose the existing wheel search result, without commanding either robot."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from swarm_manager.observation import FUTURE_TOLERANCE_NS, stamp_ns, valid_detection


class HandoffCandidate(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('handoff_candidate', parameter_overrides=parameter_overrides)
        timeout = float(self.declare_parameter('observation_timeout_s', 0.25).value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('observation_timeout_s must be positive')
        self.timeout = timeout
        self.state = ''
        self.reason = 'awaiting_found'
        self.candidate = None
        self.received_at = None
        self.last_source_ns = None
        self.last_ros_ns = None
        self.output = self.create_publisher(Detection2DArray, '/swarm/target_candidate', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/swarm/handoff_status', 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_sub = self.create_subscription(String, '/mission/state', self.on_state, state_qos)
        self.target_sub = self.create_subscription(
            Detection2DArray, '/mission/selected_target', self.on_target, 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.publish, clock=self.steady_clock)

    def clear(self, reason):
        self.candidate = None
        self.received_at = None
        self.reason = reason

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.clear('clock_reset')
            self.last_source_ns = None
        self.last_ros_ns = now
        return now

    def on_state(self, message):
        if message.data != self.state:
            self.state = message.data
            self.last_source_ns = None
            self.clear('awaiting_target' if self.state == 'FOUND' else 'awaiting_found')

    def on_target(self, message):
        now = self.now_ns()
        source = stamp_ns(message.header.stamp)
        if self.state != 'FOUND' or source is None:
            return
        # Late, repeated or future frames cannot refresh or invalidate a newer candidate.
        if source - now > FUTURE_TOLERANCE_NS or (self.last_source_ns is not None and source <= self.last_source_ns):
            return
        self.last_source_ns = source
        if (now - source) / 1e9 > self.timeout:
            self.clear('target_expired')
        elif not message.detections:
            self.clear('target_missing')
        elif not valid_detection(message):
            self.clear('invalid_target')
        else:
            self.candidate = message
            self.received_at = time.monotonic()
            self.reason = 'candidate_available'

    def publish(self):
        now = self.now_ns()
        if self.candidate is not None:
            source = stamp_ns(self.candidate.header.stamp)
            if (source is None or source - now > FUTURE_TOLERANCE_NS or (now - source) / 1e9 > self.timeout or
                    time.monotonic() - self.received_at > self.timeout):
                self.clear('target_expired')

        if self.candidate is not None:
            self.output.publish(self.candidate)
        else:
            empty = Detection2DArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            self.output.publish(empty)

        status = DiagnosticStatus()
        status.name = 'swarm/target_handoff'
        status.hardware_id = 'car_mission'
        status.level = DiagnosticStatus.OK if self.candidate is not None else DiagnosticStatus.WARN
        status.message = self.reason
        status.values = [KeyValue(key='mission_state', value=self.state),
                         KeyValue(key='meaning', value='2D candidate only; no physical handoff')]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self.diagnostics.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandoffCandidate()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

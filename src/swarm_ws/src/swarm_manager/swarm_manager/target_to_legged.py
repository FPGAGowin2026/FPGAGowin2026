"""Transform a wheel LiDAR candidate into the quadruped laser frame using TF."""

import copy
import math
import time

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from swarm_interfaces.msg import TargetPositionCandidate
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from swarm_manager.observation import FUTURE_TOLERANCE_NS, stamp_ns


class TargetToLegged(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('target_to_legged', parameter_overrides=parameter_overrides)
        self.target_frame = self.declare_parameter('target_frame', 'legged_laser').value
        if not isinstance(self.target_frame, str) or not self.target_frame.strip():
            raise ValueError('target_frame must be nonempty')
        self.timeout = float(self.declare_parameter('observation_timeout_s', 0.25).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('observation_timeout_s must be positive')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.current = None
        self.last_stamp = None
        self.received = None
        self.last_ros_ns = None
        self.reason = 'upstream_missing'
        self.publisher = self.create_publisher(TargetPositionCandidate, '/swarm/legged_target_candidate', 10)
        self.subscription = self.create_subscription(
            TargetPositionCandidate, '/swarm/target_position_candidate', self.on_candidate, 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.publish, clock=self.steady_clock)

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.current = self.last_stamp = self.received = None
            self.reason = 'clock_reset'
        self.last_ros_ns = now
        return now

    def on_candidate(self, message):
        now = self.now_ns()
        if not message.valid:
            self.current = None
            self.received = None
            self.reason = 'upstream_invalid'
            return
        stamp = stamp_ns(message.header.stamp)
        # The projection node already checked observation pairing and cluster quality.
        if (stamp is None or stamp - now > FUTURE_TOLERANCE_NS or now - stamp > self.timeout * 1e9 or
                not message.header.frame_id or not message.target_id or
                not all(math.isfinite(v) for v in (message.position.x, message.position.y,
                                                   message.position.z))):
            self.current = None
            self.received = None
            self.reason = 'upstream_invalid'
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.current = message
        self.received = time.monotonic()
        self.reason = 'awaiting_transform'

    def publish(self):
        now = self.now_ns()
        output = TargetPositionCandidate()
        output.header.stamp = self.get_clock().now().to_msg()
        output.reason = self.reason
        if self.current is not None:
            if (self.last_stamp - now > FUTURE_TOLERANCE_NS or now - self.last_stamp > self.timeout * 1e9 or
                    time.monotonic() - self.received > self.timeout):
                self.current = None
                self.received = None
                self.reason = output.reason = 'upstream_stale'
            else:
                try:
                    transform = self.buffer.lookup_transform(
                        self.target_frame, self.current.header.frame_id,
                        Time.from_msg(self.current.header.stamp))
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    values = (translation.x, translation.y, translation.z,
                              rotation.x, rotation.y, rotation.z, rotation.w)
                    norm = sum(v * v for v in values[3:])
                    if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 1e-3:
                        raise ValueError('invalid transform')
                    point = PointStamped()
                    point.header = self.current.header
                    point.point = self.current.position
                    transformed = do_transform_point(point, transform)
                    if not all(math.isfinite(v) for v in (transformed.point.x, transformed.point.y,
                                                          transformed.point.z)):
                        raise ValueError('invalid transformed point')
                    output = copy.deepcopy(self.current)
                    output.header.frame_id = self.target_frame
                    output.position = transformed.point
                    output.reason = 'candidate_available'
                except (TransformException, ValueError):
                    output.reason = 'transform_unavailable'
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetToLegged()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Convert an explicit operator-selected pose to one relay task transaction."""

import copy
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import RelayGoal, RelayStatus

from swarm_manager.observation import FUTURE_TOLERANCE_NS, stamp_ns


class RelayGoalGateway(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('relay_goal_gateway', parameter_overrides=parameter_overrides)
        self.robot_id = self.declare_parameter('robot_id', 'uav_01').value
        seconds = float(self.declare_parameter('valid_for_s', 60.0).value)
        if not isinstance(self.robot_id, str) or not self.robot_id.strip():
            raise ValueError('robot_id must be a nonempty string')
        if not math.isfinite(seconds) or not 0 < seconds < 2_147_483_647:
            raise ValueError('valid_for_s must be finite and positive')
        self.valid_for_ns = round(seconds * 1_000_000_000)
        if self.valid_for_ns <= 0:
            raise ValueError('valid_for_s must be at least one nanosecond')
        self.last_task_id = 0
        self.last_status = None
        self.goal_pub = self.create_publisher(RelayGoal, '/swarm/uav/relay_goal', 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/swarm/uav/relay_target', self.on_target, 10)
        self.status_sub = self.create_subscription(
            RelayStatus, '/swarm/uav/relay_status', self.on_status, 10)

    def on_target(self, message):
        source_ns = stamp_ns(message.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm2 = sum(value * value for value in (q.x, q.y, q.z, q.w))
        if (not message.header.frame_id.strip() or source_ns is None or
                source_ns - now_ns > FUTURE_TOLERANCE_NS or
                now_ns - source_ns >= self.valid_for_ns or
                not all(math.isfinite(value) for value in values) or
                not math.isfinite(norm2) or norm2 <= 1e-12):
            self.get_logger().warning('Ignoring invalid, future or expired relay target')
            return
        if self.last_task_id == 0xffffffff:
            self.get_logger().error('Relay task ID space exhausted; restart coordination before sending new goals')
            return

        self.last_task_id += 1
        goal = RelayGoal()
        goal.header = copy.deepcopy(message.header)
        goal.task_id = self.last_task_id
        goal.robot_id = self.robot_id
        goal.target = copy.deepcopy(message.pose)
        norm = math.sqrt(norm2)
        goal.target.orientation.x /= norm
        goal.target.orientation.y /= norm
        goal.target.orientation.z /= norm
        goal.target.orientation.w /= norm
        seconds, nanoseconds = divmod(self.valid_for_ns, 1_000_000_000)
        goal.valid_for = Duration(sec=seconds, nanosec=nanoseconds)
        # Reliable, volatile, once: never replay an old motion request to a later subscriber.
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Relay goal {goal.task_id} published for {goal.robot_id}')

    def on_status(self, message):
        if message.robot_id != self.robot_id or message.state > RelayStatus.CANCELED:
            return
        previous = self.last_status
        self.last_status = copy.deepcopy(message)
        if (previous is None or previous.task_id != message.task_id or
                previous.state != message.state or previous.reason != message.reason):
            self.get_logger().info(
                f'Relay status robot={message.robot_id} task={message.task_id} '
                f'state={message.state} reason={message.reason}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RelayGoalGateway()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

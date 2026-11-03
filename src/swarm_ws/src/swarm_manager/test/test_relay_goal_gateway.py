import copy
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from swarm_interfaces.msg import RelayGoal, RelayStatus

from swarm_manager.relay_goal_gateway import RelayGoalGateway


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def target(node):
    msg = PoseStamped()
    msg.header.frame_id = 'swarm_map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1.0, 2.0, 1.2
    msg.pose.orientation.w = 1.0
    return msg


def test_goal_and_status_topics_preserve_identity_and_reject_stale_input():
    gateway = RelayGoalGateway()
    driver = Node('relay_gateway_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(driver)
    goals = []
    publisher = driver.create_publisher(PoseStamped, '/swarm/uav/relay_target', 10)
    status_pub = driver.create_publisher(RelayStatus, '/swarm/uav/relay_status', 10)
    driver.create_subscription(RelayGoal, '/swarm/uav/relay_goal', goals.append, 10)

    def spin_until(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        spin_until(lambda: publisher.get_subscription_count() and status_pub.get_subscription_count()
                   and gateway.goal_pub.get_subscription_count())
        first = target(driver)
        publisher.publish(first)
        spin_until(lambda: len(goals) == 1)
        assert goals[0].task_id == 1
        assert goals[0].robot_id == 'uav_01'
        assert goals[0].header == first.header and goals[0].target == first.pose
        assert goals[0].valid_for.sec == 60 and goals[0].valid_for.nanosec == 0

        publisher.publish(target(driver))
        spin_until(lambda: len(goals) == 2)
        assert goals[1].task_id == 2

        slightly_ahead = target(driver)
        slightly_ahead.header.stamp = (driver.get_clock().now() + Duration(nanoseconds=20_000_000)).to_msg()
        slightly_ahead.pose.orientation.w = 2.0
        publisher.publish(slightly_ahead)
        spin_until(lambda: len(goals) == 3)
        assert goals[2].task_id == 3 and goals[2].target.orientation.w == 1.0

        status = RelayStatus()
        status.robot_id, status.task_id, status.state = 'uav_01', 2, RelayStatus.ACCEPTED
        status.reason = 'received'
        status_pub.publish(status)
        spin_until(lambda: gateway.last_status is not None)
        assert gateway.last_status.task_id == 2 and gateway.last_status.state == RelayStatus.ACCEPTED

        invalid = copy.deepcopy(target(driver))
        invalid.pose.position.x = math.nan
        gateway.on_target(invalid)
        invalid = target(driver)
        invalid.pose.orientation.w = 0.0
        gateway.on_target(invalid)
        invalid = target(driver)
        invalid.header.stamp.sec -= 61
        gateway.on_target(invalid)
        assert gateway.last_task_id == 3
    finally:
        executor.remove_node(gateway)
        executor.remove_node(driver)
        gateway.destroy_node()
        driver.destroy_node()
        executor.shutdown()

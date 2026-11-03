import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from swarm_interfaces.msg import TargetPositionCandidate
from tf2_ros import StaticTransformBroadcaster

from swarm_manager.target_to_legged import TargetToLegged


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_cross_robot_transform_requires_tf_and_invalidates_on_loss():
    node = TargetToLegged()
    driver = Node('target_to_legged_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    seen = []
    publisher = driver.create_publisher(TargetPositionCandidate, '/swarm/target_position_candidate', 10)
    driver.create_subscription(TargetPositionCandidate, '/swarm/legged_target_candidate', seen.append, 10)
    broadcaster = StaticTransformBroadcaster(driver)

    def spin_until(predicate, send_candidate=False):
        deadline = time.monotonic() + 3
        next_send = 0.0
        while not predicate() and time.monotonic() < deadline:
            if send_candidate and time.monotonic() >= next_send:
                message = TargetPositionCandidate()
                message.header.stamp = driver.get_clock().now().to_msg()
                message.header.frame_id = 'n10_sensor'
                message.target_id = 'test:1'
                message.valid = True
                message.position.x = 2.0
                publisher.publish(message)
                next_send = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0)
        spin_until(lambda: any(m.reason == 'transform_unavailable' for m in seen), send_candidate=True)
        assert not any(m.valid for m in seen)
        transform = TransformStamped()
        transform.header.stamp = driver.get_clock().now().to_msg()
        transform.header.frame_id = 'n10_sensor'
        transform.child_frame_id = 'legged_laser'
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        spin_until(lambda: any(m.valid for m in seen), send_candidate=True)
        result = next(m for m in seen if m.valid)
        assert result.header.frame_id == 'legged_laser'
        assert result.target_id == 'test:1' and result.position.x == pytest.approx(1.0)
        seen.clear()
        invalid = TargetPositionCandidate()
        invalid.valid = False
        publisher.publish(invalid)
        spin_until(lambda: any(not m.valid and m.reason == 'upstream_invalid' for m in seen))
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        driver.destroy_node()
        node.destroy_node()
        executor.shutdown()

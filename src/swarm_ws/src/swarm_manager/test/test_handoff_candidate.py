import copy
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from swarm_manager.handoff_candidate import HandoffCandidate


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    result = HandoffCandidate()
    yield result
    result.destroy_node()


def target(node, target_id='session:1'):
    message = Detection2DArray()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'camera'
    detection = Detection2D()
    detection.header = message.header
    detection.id = target_id
    detection.bbox.center.position.x = 200.0
    detection.bbox.center.position.y = 100.0
    detection.bbox.size_x = 40.0
    detection.bbox.size_y = 50.0
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = '0'
    hypothesis.hypothesis.score = 0.9
    detection.results = [hypothesis]
    message.detections = [detection]
    return message


def state(node, value):
    node.on_state(String(data=value))


def test_only_found_with_fresh_valid_target_becomes_candidate(node):
    first = target(node)
    node.on_target(first)
    assert node.candidate is None
    state(node, 'FOUND')
    node.on_target(target(node))
    assert node.candidate.detections[0].id == 'session:1'
    assert node.reason == 'candidate_available'
    without_class = target(node)
    without_class.detections[0].results = []
    node.on_target(without_class)
    assert node.candidate is without_class
    state(node, 'LOST')
    assert node.candidate is None
    assert node.reason == 'awaiting_found'


def test_empty_and_invalid_new_frames_clear_candidate(node):
    state(node, 'FOUND')
    node.on_target(target(node))
    empty = target(node)
    empty.detections = []
    node.on_target(empty)
    assert node.candidate is None and node.reason == 'target_missing'
    invalid = target(node, '')
    node.on_target(invalid)
    assert node.candidate is None and node.reason == 'invalid_target'
    mismatch = target(node)
    mismatch.detections[0].header = copy.deepcopy(mismatch.header)
    mismatch.detections[0].header.frame_id = 'other_camera'
    node.on_target(mismatch)
    assert node.candidate is None and node.reason == 'invalid_target'


def test_old_duplicate_and_future_frames_cannot_refresh_or_replace(node):
    state(node, 'FOUND')
    first = target(node)
    node.on_target(first)
    duplicate = copy.deepcopy(first)
    duplicate.detections = []
    node.on_target(duplicate)
    assert node.candidate.detections[0].id == 'session:1'
    future = target(node, 'future')
    future.header.stamp.sec += 5
    future.detections[0].header.stamp = future.header.stamp
    node.on_target(future)
    assert node.candidate.detections[0].id == 'session:1'
    time.sleep(0.3)
    node.publish()
    assert node.candidate is None and node.reason == 'target_expired'


def test_stale_source_cannot_be_revived_by_new_receipt(node):
    state(node, 'FOUND')
    stale = target(node)
    stale.header.stamp.sec -= 1
    stale.detections[0].header.stamp = stale.header.stamp
    node.on_target(stale)
    assert node.candidate is None and node.reason == 'target_expired'


def test_observation_timeout_can_be_extended():
    node = HandoffCandidate(parameter_overrides=[Parameter('observation_timeout_s', value=0.75)])
    try:
        assert node.timeout == 0.75
        state(node, 'FOUND')
        node.on_target(target(node))
        node.received_at -= 0.3
        node.publish()
        assert node.candidate is not None
    finally:
        node.destroy_node()


def test_receipt_timeout_and_backward_clock_clear_candidate(node):
    state(node, 'FOUND')
    node.on_target(target(node))
    node.received_at -= 1
    node.publish()
    assert node.candidate is None and node.reason == 'target_expired'
    node.on_target(target(node))
    assert node.candidate is not None
    node.last_ros_ns = node.get_clock().now().nanoseconds + 1_000_000_000
    node.publish()
    assert node.candidate is None and node.reason == 'clock_reset'
    assert node.last_source_ns is None


def test_real_ros_topics_deliver_and_clear_candidate(node):
    driver = Node('handoff_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    seen = []
    state_pub = driver.create_publisher(
        String, '/mission/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    target_pub = driver.create_publisher(Detection2DArray, '/mission/selected_target', 10)
    driver.create_subscription(Detection2DArray, '/swarm/target_candidate', seen.append, 10)

    def spin_until(predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        spin_until(lambda: state_pub.get_subscription_count() and target_pub.get_subscription_count())
        state_pub.publish(String(data='FOUND'))
        spin_until(lambda: node.state == 'FOUND')
        target_pub.publish(target(driver))
        spin_until(lambda: any(m.detections for m in seen))
        delivered = next(m for m in seen if m.detections)
        assert delivered.header.frame_id == 'camera'
        assert delivered.detections[0].id == 'session:1'
        seen.clear()
        state_pub.publish(String(data='LOST'))
        spin_until(lambda: node.state == 'LOST')
        spin_until(lambda: any(not m.detections for m in seen))
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()

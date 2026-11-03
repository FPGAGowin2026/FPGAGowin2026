import json
import math
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from swarm_interfaces.msg import TargetPositionCandidate
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from swarm_manager.target_projection import TargetProjection, load_extrinsics, project_candidate


EXTRINSICS = {
    'camera_frame': 'camera_optical', 'lidar_frame': 'n10_sensor',
    # Synthetic laser x-forward/y-left/z-up to camera x-right/y-down/z-forward.
    'camera_from_lidar': [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0],
}


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def extrinsic_file(tmp_path):
    path = tmp_path / 'extrinsic.json'
    path.write_text(json.dumps(EXTRINSICS), encoding='utf-8')
    return path


def observations():
    camera = CameraInfo()
    camera.header.frame_id = 'camera_optical'
    camera.width, camera.height = 640, 360
    camera.distortion_model = 'plumb_bob'
    camera.k = [300.0, 0.0, 320.0, 0.0, 300.0, 180.0, 0.0, 0.0, 1.0]
    camera.d = [0.0] * 5
    target = Detection2DArray()
    target.header.frame_id = 'camera_optical'
    target.header.stamp.sec = 1
    detection = Detection2D()
    detection.header = target.header
    detection.id = 'test:1'
    detection.bbox.center.position.x, detection.bbox.center.position.y = 320.0, 180.0
    detection.bbox.size_x, detection.bbox.size_y = 200.0, 100.0
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id, result.hypothesis.score = '0', 0.9
    detection.results = [result]
    target.detections = [detection]
    scan = LaserScan()
    scan.header.frame_id = 'n10_sensor'
    scan.angle_min, scan.angle_increment = -0.2, 0.05
    scan.range_min, scan.range_max = 0.05, 20.0
    scan.ranges = [2.0] * 9
    return target, scan, camera


def test_rigid_calibration_is_required_and_validated(extrinsic_file, tmp_path):
    assert load_extrinsics('') is None
    assert len(load_extrinsics(str(extrinsic_file))['matrix']) == 12
    invalid = dict(EXTRINSICS)
    invalid['camera_from_lidar'] = [0] * 12
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(invalid), encoding='utf-8')
    with pytest.raises(ValueError):
        load_extrinsics(str(path))


def test_project_consistent_returns_and_reject_ambiguity(extrinsic_file):
    target, scan, camera = observations()
    calibration = load_extrinsics(str(extrinsic_file))
    assert project_candidate(target, scan, camera, None)[0] == 'calibration_missing'
    reason, point, count, spread = project_candidate(target, scan, camera, calibration)
    assert reason == 'candidate_available' and count == 9
    assert point[0] == pytest.approx(2.0, abs=0.05)
    assert point[1] == pytest.approx(0.0, abs=0.01)
    assert spread == pytest.approx(0.0)
    target.detections[0].results = []
    assert project_candidate(target, scan, camera, calibration)[0] == 'candidate_available'
    scan.ranges[4] = 4.0
    assert project_candidate(target, scan, camera, calibration)[0] == 'ambiguous_lidar_returns'
    scan.ranges = [float('nan')] * 7 + [2.0, 2.0]
    assert project_candidate(target, scan, camera, calibration)[0] == 'insufficient_lidar_returns'


def test_frame_camera_and_box_checks(extrinsic_file):
    target, scan, camera = observations()
    calibration = load_extrinsics(str(extrinsic_file))
    scan.header.frame_id = 'different_lidar'
    assert project_candidate(target, scan, camera, calibration)[0] == 'frame_or_detection_invalid'
    scan.header.frame_id = 'n10_sensor'
    camera.d = []
    assert project_candidate(target, scan, camera, calibration)[0] == 'camera_info_invalid'
    camera.d = [0.0] * 5
    target.detections[0].bbox.center.theta = 0.5
    assert project_candidate(target, scan, camera, calibration)[0] == 'rotated_box_unsupported'


def test_ros_stream_requires_calibration_and_invalidates_on_loss(extrinsic_file):
    node = TargetProjection(parameter_overrides=[Parameter('extrinsic_file', value=str(extrinsic_file))])
    driver = Node('target_projection_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    seen = []
    target_pub = driver.create_publisher(Detection2DArray, '/swarm/target_candidate', 10)
    scan_pub = driver.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    info_pub = driver.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
    driver.create_subscription(TargetPositionCandidate, '/swarm/target_position_candidate', seen.append, 10)

    def spin_until(predicate):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        spin_until(lambda: target_pub.get_subscription_count() and scan_pub.get_subscription_count() and
                   info_pub.get_subscription_count())
        target, scan, camera = observations()
        stamp = driver.get_clock().now().to_msg()
        target.header.stamp = stamp
        target.detections[0].header.stamp = stamp
        scan.header.stamp = stamp
        info_pub.publish(camera)
        target_pub.publish(target)
        scan_pub.publish(scan)
        spin_until(lambda: any(message.valid for message in seen))
        result = next(message for message in seen if message.valid)
        assert result.header.frame_id == 'n10_sensor' and result.target_id == 'test:1'
        assert result.supporting_points == 9 and math.isfinite(result.position.x)
        seen.clear()
        empty = Detection2DArray()
        target_pub.publish(empty)
        spin_until(lambda: any(not message.valid and message.reason == 'target_missing' for message in seen))
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        driver.destroy_node()
        node.destroy_node()
        executor.shutdown()

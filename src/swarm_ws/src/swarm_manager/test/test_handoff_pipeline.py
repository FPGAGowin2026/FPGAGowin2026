import json
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from swarm_interfaces.msg import TargetPositionCandidate
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from swarm_manager.handoff_candidate import HandoffCandidate
from swarm_manager.target_projection import TargetProjection
from swarm_manager.target_to_legged import TargetToLegged


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_found_to_legged_candidate_and_lost_invalidation(tmp_path):
    calibration = tmp_path / 'synthetic_calibration.json'
    calibration.write_text(json.dumps({
        'camera_frame': 'camera_optical', 'lidar_frame': 'n10_sensor',
        'camera_from_lidar': [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0],
    }), encoding='utf-8')
    handoff = HandoffCandidate()
    projection = TargetProjection(parameter_overrides=[Parameter('extrinsic_file', value=str(calibration))])
    transform_node = TargetToLegged()
    driver = Node('handoff_pipeline_test_driver')
    executor = SingleThreadedExecutor()
    for node in (handoff, projection, transform_node, driver):
        executor.add_node(node)
    state_pub = driver.create_publisher(
        String, '/mission/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    detection_pub = driver.create_publisher(Detection2DArray, '/mission/selected_target', 10)
    scan_pub = driver.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    info_pub = driver.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
    seen = []
    driver.create_subscription(TargetPositionCandidate, '/swarm/legged_target_candidate', seen.append, 10)
    broadcaster = StaticTransformBroadcaster(driver)

    def spin():
        for _ in range(6):
            executor.spin_once(timeout_sec=0.01)

    def send_inputs():
        stamp = driver.get_clock().now().to_msg()
        camera = CameraInfo()
        camera.header.frame_id = 'camera_optical'
        camera.width, camera.height = 640, 360
        camera.distortion_model = 'plumb_bob'
        camera.k = [300., 0., 320., 0., 300., 180., 0., 0., 1.]
        camera.d = [0.] * 5
        info_pub.publish(camera)
        target = Detection2DArray()
        target.header.frame_id, target.header.stamp = 'camera_optical', stamp
        detection = Detection2D()
        detection.header = target.header
        detection.id = 'session:1'
        detection.bbox.center.position.x, detection.bbox.center.position.y = 320., 180.
        detection.bbox.size_x, detection.bbox.size_y = 200., 100.
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id, result.hypothesis.score = '0', 0.9
        detection.results = [result]
        target.detections = [detection]
        detection_pub.publish(target)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'n10_sensor', stamp
        scan.angle_min, scan.angle_increment = -0.2, 0.05
        scan.range_min, scan.range_max = 0.05, 20.
        scan.ranges = [2.] * 9
        scan_pub.publish(scan)

    try:
        deadline = time.monotonic() + 4
        while not (state_pub.get_subscription_count() and detection_pub.get_subscription_count() and
                   scan_pub.get_subscription_count() and info_pub.get_subscription_count()):
            assert time.monotonic() < deadline
            spin()
        deadline = time.monotonic() + 5
        transforms = []
        for parent, child, offset in (('map', 'odom', 0.), ('odom', 'n10_sensor', 0.),
                                      ('map', 'legged_map', 1.), ('legged_map', 'legged_odom', 0.),
                                      ('legged_odom', 'legged_laser', 0.)):
            transform = TransformStamped()
            transform.header.stamp = driver.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x = offset
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
        state_pub.publish(String(data='FOUND'))
        while not any(item.valid for item in seen) and time.monotonic() < deadline:
            send_inputs()
            spin()
        assert any(item.valid for item in seen)
        output = next(item for item in reversed(seen) if item.valid)
        assert output.header.frame_id == 'legged_laser' and output.target_id == 'session:1'
        assert output.position.x == pytest.approx(1.0, abs=0.05)
        seen.clear()
        state_pub.publish(String(data='LOST'))
        deadline = time.monotonic() + 3
        while not any(not item.valid and item.reason == 'upstream_invalid' for item in seen):
            assert time.monotonic() < deadline
            spin()
    finally:
        for node in (handoff, projection, transform_node, driver):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()

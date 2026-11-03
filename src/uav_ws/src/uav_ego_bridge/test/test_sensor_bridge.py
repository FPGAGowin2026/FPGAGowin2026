"""Exercise real ROS topics, timing and TF without PX4 or hardware."""
import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


def test_sensor_timing_rotation_and_tf():
    os.environ.setdefault('ROS_DOMAIN_ID', '21')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('ego_sensor_bridge_test_driver')
    odom_pub = node.create_publisher(Odometry, '/ego_test/odom', qos_profile_sensor_data)
    depth_pub = node.create_publisher(Image, '/ego_test/depth', qos_profile_sensor_data)
    odom_out, camera_out, depth_out, ready = [], [], [], []
    node.create_subscription(Odometry, '/uav_ego/odom', odom_out.append, 10)
    node.create_subscription(PoseStamped, '/uav_ego/camera_pose', camera_out.append, 10)
    node.create_subscription(Image, '/uav_ego/depth', depth_out.append, 10)
    node.create_subscription(Bool, '/uav_ego/sensor_ready', ready.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    executable = Path(get_package_prefix('uav_ego_bridge')) / 'lib/uav_ego_bridge/ego_sensor_bridge'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'odom_topic:=/ego_test/odom',
        '-p', 'depth_topic:=/ego_test/depth', '-p', 'camera_frame:=camera_optical',
        '-p', 'max_source_age_s:=3.0', '-p', 'max_receive_age_s:=0.8',
        '-p', 'max_pose_gap_s:=0.5'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        # On Linux this permits posix_spawn instead of forking an initialized DDS process.
        close_fds=False)

    def spin_until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, 'sensor bridge exited early'
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate()

    def odom(stamp_ns, x):
        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = math.sqrt(0.5)
        msg.pose.pose.orientation.w = math.sqrt(0.5)
        msg.twist.twist.linear.x = 1.0
        return msg

    try:
        spin_until(lambda: odom_pub.get_subscription_count() and depth_pub.get_subscription_count())
        spin_until(lambda: ready and not ready[-1].data)
        current_ns = node.get_clock().now().nanoseconds
        first_ns, image_ns, second_ns = current_ns - 200_000_000, current_ns - 150_000_000, current_ns - 100_000_000
        depth = Image()
        depth.header.frame_id = 'camera_optical'
        depth.header.stamp = Time(nanoseconds=image_ns).to_msg()
        depth.width, depth.height, depth.encoding, depth.step = 2, 2, '16UC1', 4
        depth.data = bytes([232, 3] * 4)
        odom_pub.publish(odom(first_ns, 1.0))
        odom_pub.publish(odom(second_ns, 3.0))
        depth_pub.publish(depth)
        spin_until(lambda: len(odom_out) >= 2)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        assert not camera_out and not depth_out, 'missing measured extrinsic must not produce depth pose'
        assert not ready[-1].data

        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_optical'
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        deadline = time.monotonic() + 5
        while not (camera_out and depth_out) and time.monotonic() < deadline:
            odom_pub.publish(odom(node.get_clock().now().nanoseconds, 4.0))
            rclpy.spin_once(node, timeout_sec=0.05)
        assert camera_out and depth_out
        camera = camera_out[-1]
        assert camera.header.frame_id == 'map' and camera.header.stamp == depth.header.stamp
        assert depth_out[-1].header.stamp == depth.header.stamp
        assert camera.pose.position.x == pytest.approx(2.0, abs=1e-5)
        assert camera.pose.position.y == pytest.approx(1.0, abs=1e-5)
        assert odom_out[-1].header.frame_id == 'map'
        assert odom_out[-1].twist.twist.linear.x == pytest.approx(0.0, abs=1e-5)
        assert odom_out[-1].twist.twist.linear.y == pytest.approx(1.0, abs=1e-5)
        spin_until(lambda: ready and ready[-1].data)
        spin_until(lambda: ready and not ready[-1].data, timeout=3)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()

"""Project wheel LiDAR returns into a selected image detection, without commanding motion."""

import json
import math
import statistics
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from swarm_interfaces.msg import TargetPositionCandidate
from vision_msgs.msg import Detection2DArray

from swarm_manager.observation import FUTURE_TOLERANCE_NS, stamp_ns, valid_detection


def load_extrinsics(path):
    if not path:
        return None
    with open(path, encoding='utf-8') as source:
        data = json.load(source)
    camera_frame, lidar_frame = data['camera_frame'], data['lidar_frame']
    matrix = data['camera_from_lidar']
    if (not isinstance(camera_frame, str) or not camera_frame.strip() or
            not isinstance(lidar_frame, str) or not lidar_frame.strip() or
            not isinstance(matrix, list) or len(matrix) != 12 or
            any(isinstance(x, bool) or not isinstance(x, (int, float)) or
                not math.isfinite(x) for x in matrix)):
        raise ValueError('extrinsics require two frame names and 12 finite matrix entries')
    rotation = [[float(matrix[row * 4 + col]) for col in range(3)] for row in range(3)]
    for row in range(3):
        for other in range(3):
            dot = sum(rotation[row][i] * rotation[other][i] for i in range(3))
            if abs(dot - (1.0 if row == other else 0.0)) > 1e-3:
                raise ValueError('camera_from_lidar rotation must be orthonormal')
    a, b, c = rotation
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1]) -
                   a[1] * (b[0] * c[2] - b[2] * c[0]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(determinant - 1.0) > 1e-3:
        raise ValueError('camera_from_lidar must be a proper rotation')
    return {'camera_frame': camera_frame, 'lidar_frame': lidar_frame,
            'matrix': [float(x) for x in matrix]}


def project_candidate(detection_array, scan, camera, extrinsics):
    """Return reason, representative LiDAR-frame point, count, range spread."""
    if extrinsics is None:
        return 'calibration_missing', None, 0, 0.0
    if (not valid_detection(detection_array) or
            detection_array.header.frame_id != extrinsics['camera_frame'] or
            camera.header.frame_id != extrinsics['camera_frame'] or
            scan.header.frame_id != extrinsics['lidar_frame']):
        return 'frame_or_detection_invalid', None, 0, 0.0
    if (camera.width == 0 or camera.height == 0 or camera.distortion_model != 'plumb_bob' or
            len(camera.k) != 9 or len(camera.d) != 5 or
            not all(math.isfinite(v) for v in list(camera.k) + list(camera.d)) or
            camera.k[0] <= 0 or camera.k[4] <= 0 or abs(camera.k[8] - 1.0) > 1e-6):
        return 'camera_info_invalid', None, 0, 0.0
    if (not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment,
                                            scan.range_min, scan.range_max)) or
            scan.angle_increment == 0 or scan.range_min < 0 or scan.range_max <= scan.range_min):
        return 'scan_geometry_invalid', None, 0, 0.0
    box = detection_array.detections[0].bbox
    if abs(box.center.theta) > 1e-6:
        return 'rotated_box_unsupported', None, 0, 0.0
    left = max(0.0, box.center.position.x - box.size_x / 2)
    right = min(float(camera.width), box.center.position.x + box.size_x / 2)
    top = max(0.0, box.center.position.y - box.size_y / 2)
    bottom = min(float(camera.height), box.center.position.y + box.size_y / 2)
    if left >= right or top >= bottom:
        return 'box_outside_image', None, 0, 0.0

    matrix = extrinsics['matrix']
    k1, k2, p1, p2, k3 = camera.d
    candidates = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not max(scan.range_min, 0.05) <= distance <= min(scan.range_max, 8.0):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        xc = matrix[0] * x + matrix[1] * y + matrix[3]
        yc = matrix[4] * x + matrix[5] * y + matrix[7]
        zc = matrix[8] * x + matrix[9] * y + matrix[11]
        if zc <= 0.05:
            continue
        xn, yn = xc / zc, yc / zc
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        u = camera.k[0] * xd + camera.k[1] * yd + camera.k[2]
        v = camera.k[4] * yd + camera.k[5]
        if math.isfinite(u) and math.isfinite(v) and left <= u <= right and top <= v <= bottom:
            candidates.append((angle % (2 * math.pi), x, y, distance))
    if not candidates:
        return 'no_lidar_return_in_box', None, 0, 0.0

    candidates.sort()
    clusters = [[candidates[0]]]
    for point in candidates[1:]:
        previous = clusters[-1][-1]
        if (point[0] - previous[0] <= 0.12 and
                math.hypot(point[1] - previous[1], point[2] - previous[2]) <= 0.4):
            clusters[-1].append(point)
        else:
            clusters.append([point])
    if len(clusters) > 1:
        first, last = clusters[0][0], clusters[-1][-1]
        if (first[0] + 2 * math.pi - last[0] <= 0.12 and
                math.hypot(first[1] - last[1], first[2] - last[2]) <= 0.4):
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
    if len(clusters) != 1:
        return 'ambiguous_lidar_returns', None, len(candidates), 0.0
    cluster = clusters[0]
    spread = max(point[3] for point in cluster) - min(point[3] for point in cluster)
    if len(cluster) < 3:
        return 'insufficient_lidar_returns', None, len(cluster), spread
    if spread > 0.3:
        return 'ambiguous_lidar_returns', None, len(cluster), spread
    point = (statistics.median(p[1] for p in cluster), statistics.median(p[2] for p in cluster), 0.0)
    return 'candidate_available', point, len(cluster), spread


class TargetProjection(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('target_projection', parameter_overrides=parameter_overrides)
        path = self.declare_parameter('extrinsic_file', '').value
        self.extrinsics = load_extrinsics(path)
        self.timeout = float(self.declare_parameter('observation_timeout_s', 0.25).value)
        self.sync_tolerance = float(self.declare_parameter('sync_tolerance_s', 0.15).value)
        if (not math.isfinite(self.timeout) or self.timeout <= 0 or
                not math.isfinite(self.sync_tolerance) or self.sync_tolerance < 0):
            raise ValueError('observation timeout must be positive and sync tolerance nonnegative')
        self.target = self.scan = self.camera = None
        self.target_received = self.scan_received = None
        self.target_stamp = self.scan_stamp = None
        self.last_ros_ns = None
        self.publisher = self.create_publisher(TargetPositionCandidate, '/swarm/target_position_candidate', 10)
        self.target_sub = self.create_subscription(
            Detection2DArray, '/swarm/target_candidate', self.on_target, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.camera_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.publish, clock=self.steady_clock)

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.target = self.scan = None
            self.target_stamp = self.scan_stamp = None
            self.target_received = self.scan_received = None
        self.last_ros_ns = now
        return now

    def on_target(self, message):
        now = self.now_ns()
        if not message.detections:
            self.target = None
            self.target_received = None
            return
        stamp = stamp_ns(message.header.stamp)
        if (stamp is None or stamp - now > FUTURE_TOLERANCE_NS or now - stamp > self.timeout * 1e9 or
                (self.target_stamp is not None and stamp <= self.target_stamp)):
            return
        self.target_stamp = stamp
        self.target = message
        self.target_received = time.monotonic()

    def on_scan(self, message):
        now = self.now_ns()
        stamp = stamp_ns(message.header.stamp)
        if (stamp is None or stamp - now > FUTURE_TOLERANCE_NS or now - stamp > self.timeout * 1e9 or
                (self.scan_stamp is not None and stamp <= self.scan_stamp)):
            return
        self.scan_stamp = stamp
        self.scan = message
        self.scan_received = time.monotonic()

    def on_camera(self, message):
        self.camera = message

    def publish(self):
        now = self.now_ns()
        message = TargetPositionCandidate()
        message.header.stamp = self.get_clock().now().to_msg()
        if self.extrinsics is None:
            message.reason = 'calibration_missing'
        elif self.target is None:
            message.reason = 'target_missing'
        elif self.scan is None or self.camera is None:
            message.reason = 'scan_or_camera_info_missing'
        elif (self.target_stamp - now > FUTURE_TOLERANCE_NS or self.scan_stamp - now > FUTURE_TOLERANCE_NS or
              (now - self.target_stamp) > self.timeout * 1e9 or
              (now - self.scan_stamp) > self.timeout * 1e9 or
              time.monotonic() - self.target_received > self.timeout or
              time.monotonic() - self.scan_received > self.timeout):
            message.reason = 'observation_stale'
        elif abs(self.target_stamp - self.scan_stamp) > self.sync_tolerance * 1e9:
            message.reason = 'observation_time_mismatch'
        else:
            reason, point, count, spread = project_candidate(self.target, self.scan, self.camera, self.extrinsics)
            message.reason = reason
            message.supporting_points = count
            message.range_spread_m = spread
            message.detection_stamp = self.target.header.stamp
            detection = self.target.detections[0] if self.target.detections else None
            if detection is not None:
                message.target_id = detection.id
                hypotheses = [item.hypothesis for item in detection.results
                              if item.hypothesis.class_id and math.isfinite(item.hypothesis.score)]
                if hypotheses:
                    message.class_id = max(hypotheses, key=lambda item: item.score).class_id
            if point is not None:
                message.valid = True
                message.header = self.scan.header
                message.position.x, message.position.y, message.position.z = point
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetProjection()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

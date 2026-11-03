#!/usr/bin/env python3
"""Adapt N10 scans to slam_toolbox and expose its map to the web dashboard."""
from array import array
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from global_map_core import SlamMapView, planar_transform, reduce_scan


class GlobalMapNode(Node):
    def __init__(self):
        super().__init__('global_map_node')
        self.declare_parameter('http_bind', '0.0.0.0')
        self.declare_parameter('http_port', 8766)
        self.view = SlamMapView()
        self.lock = threading.Lock()
        self.pose = None
        self.pose_at = None
        self.map_transform = None
        self.map_transform_at = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.parameter_client = self.create_client(GetParameters, '/slam_toolbox/get_parameters')
        self.parameter_future = None
        self.parameter_requested_at = 0.0
        self.create_subscription(String, '/fpga/round5/navigation', self.receive_navigation,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.receive_map, map_qos)
        self.scan_publisher = self.create_publisher(LaserScan, '/slam/scan', qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(String, '/global_map_status', map_qos)
        self.timer = self.create_timer(0.2, self.update_status)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path not in ('/global-map.json', '/global-map-meta.json'):
                    self.send_error(404)
                    return
                with owner.lock:
                    now = time.monotonic()
                    view = owner.snapshot_locked(now)
                    if self.path == '/global-map-meta.json':
                        view.pop('cells_rle', None)
                payload = json.dumps(view, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, _format, *_args):
                pass

        bind = self.get_parameter('http_bind').value
        port = self.get_parameter('http_port').value
        self.http = ThreadingHTTPServer((bind, port), Handler)
        self.http.daemon_threads = True
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'SLAM web map: http://{bind}:{port}/global-map.json')

    def snapshot_locked(self, now):
        pose = self.pose if self.pose_at is not None and now - self.pose_at <= 1.0 else None
        view = self.view.snapshot(now, pose)
        age_ms = (self.map_transform['age_ms'] + (now - self.map_transform_at) * 1000
                  if self.map_transform is not None and self.map_transform_at is not None else None)
        if view['status'] == 'building' and age_ms is not None and 0 <= age_ms <= 750:
            view['map_from_odom'] = {k: self.map_transform[k] for k in ('x', 'y', 'yaw')}
            view['map_from_odom_age_ms'] = age_ms
        else:
            view['map_from_odom'] = None
            view['map_from_odom_age_ms'] = None
        return view

    def receive_navigation(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            with self.lock:
                previous = self.view.reset_required
                self.view.navigation(data, time.monotonic())
                if self.view.reset_required and not previous:
                    self.get_logger().error('FPGA coordinate domain changed; restart global SLAM')
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Invalid navigation message ignored')

    def receive_scan(self, message):
        with self.lock:
            if not self.view.can_scan(time.monotonic()):
                return
        reduced = reduce_scan(message.ranges, message.angle_min, message.angle_increment,
                              message.range_min, message.range_max)
        if reduced is None:
            return
        angle_min, angle_increment, ranges = reduced
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + message.header.stamp.nanosec
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or not message.header.frame_id or not -0.05 <= age_s <= 1.0:
            return
        scan = LaserScan()
        scan.header = message.header
        scan.angle_min = angle_min
        scan.angle_increment = angle_increment
        scan.angle_max = angle_min + angle_increment * (len(ranges) - 1)
        scan.time_increment = 0.0  # N10 has no per-beam timestamps.
        scan.scan_time = message.scan_time
        scan.range_min, scan.range_max = message.range_min, message.range_max
        scan.ranges = array('f', ranges)
        with self.lock:
            if not self.view.can_scan(time.monotonic()):
                return
            self.view.sent_scan(time.monotonic(), stamp_ns)
        self.scan_publisher.publish(scan)

    def receive_map(self, msg):
        q = msg.info.origin.orientation
        data = {'frame_id': msg.header.frame_id,
                'stamp_ns': int(msg.header.stamp.sec) * 1_000_000_000 + msg.header.stamp.nanosec,
                'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y},
                'orientation_identity': abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and
                                        abs(q.z) < 1e-6 and abs(abs(q.w) - 1.0) < 1e-6,
                'cells': msg.data}
        with self.lock:
            self.view.receive_map(data, time.monotonic())

    def update_status(self):
        now = time.monotonic()
        if (self.parameter_future is not None and not self.parameter_future.done() and
                now - self.parameter_requested_at > 4.0):
            self.parameter_future.cancel()
            self.parameter_future = None
        if (now - self.parameter_requested_at >= 2.0 and
                (self.parameter_future is None or self.parameter_future.done()) and
                self.parameter_client.service_is_ready()):
            request = GetParameters.Request()
            request.names = ['do_loop_closing']
            self.parameter_future = self.parameter_client.call_async(request)
            self.parameter_future.add_done_callback(self.receive_loop_parameter)
            self.parameter_requested_at = now
        pose = None
        map_transform = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            stamp = int(transform.header.stamp.sec) * 1_000_000_000 + transform.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            q, p = transform.transform.rotation, transform.transform.translation
            if -0.2 <= age <= 1.0:
                yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                if all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                    pose = {'x': p.x, 'y': p.y, 'yaw': yaw}
        except TransformException:
            pass
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', Time())
            stamp = int(transform.header.stamp.sec) * 1_000_000_000 + transform.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            q, p = transform.transform.rotation, transform.transform.translation
            map_transform = planar_transform(p.x, p.y, q.x, q.y, q.z, q.w, age)
        except TransformException:
            pass
        with self.lock:
            self.pose, self.pose_at = pose, time.monotonic() if pose is not None else None
            self.map_transform = map_transform
            self.map_transform_at = time.monotonic() if map_transform is not None else None
            status = self.snapshot_locked(time.monotonic())
        status.pop('cells_rle', None)
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(msg)

    def receive_loop_parameter(self, future):
        try:
            values = future.result().values
            enabled = values[0].bool_value if len(values) == 1 and values[0].type == ParameterType.PARAMETER_BOOL else None
        except Exception:
            enabled = None
        with self.lock:
            self.view.loop_state(enabled, time.monotonic())

    def destroy_node(self):
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=2)
        return super().destroy_node()


def main():
    rclpy.init()
    node = GlobalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

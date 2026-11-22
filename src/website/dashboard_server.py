#!/usr/bin/env python3
"""Serve the existing dashboard using the new robot ROS and video nodes."""

import argparse
import hashlib
import json
import math
import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlsplit
from urllib.request import ProxyHandler, Request, build_opener

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ugv_bridge.srv import MissionStart


PUBLIC = Path(__file__).resolve().parent / 'public'
UPSTREAM = 'http://127.0.0.1:8081'
SWARM_UPSTREAM = 'http://127.0.0.1:8767'
ACG720_UPSTREAM = os.environ.get('ACG720_UPSTREAM', 'http://198.51.100.39:8766')
FLEET_FILE = Path(__file__).resolve().parent.parent / 'swarm_ws/config/robots.json'
GET_ROUTES = {'/status.json', '/mission.json', '/local_map.png', '/video.mjpg', '/frame.jpg'}
POST_ROUTES = {'/manual', '/goal', '/explore', '/stop'}
SWARM_GET_ROUTES = {'/swarm/api/state', '/swarm/api/map'}
SWARM_POST_ROUTES = {'/swarm/api/alignment/reobserve',
                     '/swarm/api/explore/start', '/swarm/api/explore/stop',
                     '/swarm/api/explore/confirm-stopped'}
OPENER = build_opener(ProxyHandler({}))
LOCAL_MAP_MAX_AGE = 1.5
LOCAL_MAP_CELLS = 64
LOCAL_MAP_RESOLUTION = 0.05


def finite(value):
    return value if math.isfinite(value) else None


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or sum(v*v for v in values) < 1e-12:
        raise ValueError('invalid map rotation')
    return math.atan2(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)


def local_scan_pixels(scan):
    size = LOCAL_MAP_CELLS
    center = size // 2
    raster = np.full((size, size), 127, dtype=np.uint8)
    ranges = np.asarray(scan.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if not np.any(valid):
        return raster
    angles = scan.angle_min + np.flatnonzero(valid) * scan.angle_increment
    distance = ranges[valid] / LOCAL_MAP_RESOLUTION
    cells = np.floor(np.column_stack((distance * np.cos(angles), distance * np.sin(angles)))
                     + center).astype(np.int32)
    cells[:, 1] = size - 1 - cells[:, 1]
    rays = np.empty((len(cells), 2, 2), dtype=np.int32)
    rays[:, 0] = (center, size - 1 - center)
    rays[:, 1] = cells
    cv2.polylines(raster, list(rays), False, 255)
    # Place returns last so that crossing rays cannot erase occupied cells.
    inside = cells[((cells >= 0) & (cells < size)).all(axis=1)]
    raster[inside[:, 1], inside[:, 0]] = 0
    return raster


class DashboardState(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        self.lock = threading.Lock()
        self.latest = {}
        self.create_subscription(String, '/acg720/status', self.on_acg720, 1)
        self.navigation_grid = None
        self.navigation_grid_at = 0.0
        self.navigation_grid_stamp = None
        self.navigation_image_cache = None
        self.acg720_scan = None
        self.acg720_scan_png = None
        self.vision_start = self.create_client(MissionStart, '/vision/mission/start')
        self.vision_stop = self.create_client(Trigger, '/vision/mission/stop')
        for name, topic in (
            ('motor', '/fpga/motor'), ('navigation', '/fpga/navigation_status'),
            ('fusion', '/fpga/fusion_health'), ('command', '/fpga/command_status'),
            ('mission', '/robot/mission/status'),
            ('vision_mission', '/vision/mission_status'),
        ):
            self.create_subscription(String, topic, partial(self.on_json, name), 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/acg720/scan', self.on_acg720_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/robot/navigation_map', self.on_navigation_map, 1)

    def on_navigation_map(self, message):
        with self.lock:
            grid = self.navigation_grid
            if (grid is None or grid.header.frame_id != message.header.frame_id or
                    grid.info != message.info or grid.data != message.data):
                self.navigation_grid = message
                self.navigation_image_cache = None
            self.navigation_grid_at = time.monotonic()
            self.navigation_grid_stamp = message.header.stamp

    def navigation_map_image(self, *, cached_only=False):
        with self.lock:
            grid, received_at = self.navigation_grid, self.navigation_grid_at
            stamp_msg = self.navigation_grid_stamp
            cached = self.navigation_image_cache
            mission = self.latest.get('mission')
        if (cached_only and cached is None) or grid is None or grid.header.frame_id != 'map' or (
                time.monotonic() - received_at > LOCAL_MAP_MAX_AGE):
            return None
        stamp_ns = stamp_msg.sec * 1_000_000_000 + stamp_msg.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns)*1e-9
        if stamp_ns <= 0 or not -0.2 <= age <= LOCAL_MAP_MAX_AGE:
            return None
        if cached is None:
            info = grid.info
            if (not info.width or not info.height or info.width*info.height > 16*1024*1024 or
                    len(grid.data) != info.width*info.height or not math.isfinite(info.resolution) or
                    info.resolution <= 0):
                return None
            values = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
            if np.any(values < -1) or np.any(values > 100):
                return None
            pixels = np.ascontiguousarray(np.flipud(np.where(values < 0, 127, 255-values*255//100).astype(np.uint8)))
            ok, encoded = cv2.imencode('.png', pixels)
            if not ok:
                return None
            geometry = ','.join(str(v) for v in (info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation)))
            png = encoded.tobytes()
            digest = hashlib.sha256(geometry.encode())
            digest.update(png)
            cached = (png, geometry, digest.hexdigest())
            with self.lock:
                if self.navigation_grid is grid:
                    self.navigation_image_cache = cached
        active = bool(mission and time.monotonic() - mission[0] < LOCAL_MAP_MAX_AGE and
                      mission[1].get('planning_local_map'))
        png, geometry, digest = cached
        return png, geometry, active, f'{digest}-{int(active)}'

    def put(self, name, value):
        with self.lock:
            self.latest[name] = (time.monotonic(), value)

    def on_json(self, name, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            if name == 'navigation':
                value.pop('session_token', None)
            self.put(name, value)
        except (ValueError, TypeError):
            return

    def on_imu(self, message):
        a, g = message.linear_acceleration, message.angular_velocity
        self.put('imu', {'accel': [finite(a.x), finite(a.y), finite(a.z)],
                         'gyro': [finite(g.x), finite(g.y), finite(g.z)]})

    def on_scan(self, message):
        front = [r for i, r in enumerate(message.ranges)
                 if math.isfinite(r) and message.range_min <= r <= message.range_max
                 and abs(message.angle_min + i * message.angle_increment) <= math.pi / 4]
        valid = sum(1 for r in message.ranges if math.isfinite(r)
                    and message.range_min <= r <= message.range_max)
        self.put('scan', {'front_min_m': min(front) if front else None, 'valid_points': valid})

    def on_acg720_scan(self, scan):
        stamp = scan.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0 or not all(math.isfinite(a) for a in (scan.angle_min, scan.angle_increment)):
            return
        observed_at = time.monotonic() - max(0, self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        version = str(stamp_ns)
        with self.lock:
            self.acg720_scan = (observed_at, version, scan)
            self.acg720_scan_png = None
            self.latest['acg720_local_map'] = (observed_at, {'version': version})

    def acg720_local_map_image(self):
        with self.lock:
            sample, cached = self.acg720_scan, self.acg720_scan_png
        if sample is None or time.monotonic() - sample[0] > LOCAL_MAP_MAX_AGE:
            return None
        _, version, scan = sample
        if cached is None:
            ok, encoded = cv2.imencode('.png', local_scan_pixels(scan))
            if not ok:
                return None
            cached = encoded.tobytes()
            with self.lock:
                if self.acg720_scan is sample:
                    self.acg720_scan_png = cached
        return cached, version

    def on_odom(self, message):
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        self.put('odom', {'x': finite(p.x), 'y': finite(p.y),
                          'yaw': finite(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                   1 - 2 * (q.y * q.y + q.z * q.z))),
                          'vx': finite(v.linear.x), 'vy': finite(v.linear.y),
                          'wz': finite(v.angular.z)})

    def on_acg720(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and data.get('robot_id') == 'acg720':
                self.put('acg720', data)
        except ValueError:
            pass

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            return {name: {'age_ms': round((now - at) * 1000), 'data': value}
                    for name, (at, value) in self.latest.items()}


class Handler(SimpleHTTPRequestHandler):
    state = None
    upstream = UPSTREAM
    swarm_upstream = SWARM_UPSTREAM
    acg720_upstream = ACG720_UPSTREAM

    def do_GET(self):
        url = urlsplit(self.path)
        path = url.path
        if path == '/api/fleet':
            payload = json.dumps({**json.loads(FLEET_FILE.read_text(encoding='utf-8-sig')),
                                  'acg720_ros': self.state.snapshot().get('acg720')},
                                 ensure_ascii=False, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        elif path in ('/acg720/api/status', '/acg720/frame.jpg'):
            self.proxy('GET', False, self.acg720_upstream, self.path[len('/acg720'):])
        elif path == '/acg720/local_map.png':
            image = self.state.acg720_local_map_image()
            if image is None:
                self.send_error(503, 'No fresh ACG720 scan')
            else:
                body, version = image
                self.send_png(body, {'X-BX72-Map-Version': version})
        elif path == '/api/state':
            state = self.state.snapshot()
            image = self.state.navigation_map_image(cached_only=True)
            state['map_version'] = image[3] if image is not None else None
            payload = json.dumps(state, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        elif path == '/map.png':
            if parse_qs(url.query).get('raw') == ['1'] or not self.serve_navigation_map():
                self.proxy('GET', False)
        elif path in GET_ROUTES:
            self.proxy('GET', path == '/video.mjpg')
        elif path in SWARM_GET_ROUTES:
            self.proxy('GET', False, self.swarm_upstream, self.path[len('/swarm'):])
        else:
            super().do_GET()

    def serve_navigation_map(self):
        try:
            image = self.state.navigation_map_image()
        except (ValueError, cv2.error):
            return False
        if image is None:
            return False
        body, geometry, active, version = image
        self.send_png(body, {'X-BX72-Map': geometry, 'X-BX72-Map-Overlay': 'active' if active else 'unavailable',
                             'X-BX72-Map-Version': version})
        return True

    def send_png(self, body, headers):
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            for name, value in headers.items():
                self.send_header(name, value)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        path = urlsplit(self.path).path
        if path in ('/vision/start', '/vision/stop'):
            self.vision_request(path)
        elif path in SWARM_POST_ROUTES or path == '/acg720/api/control':
            try:
                size = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                size = 0
            if not 0 < size <= 16384:
                self.send_error(400, 'Invalid request body')
                return
            prefix = '/acg720' if path.startswith('/acg720/') else '/swarm'
            upstream = self.acg720_upstream if prefix == '/acg720' else self.swarm_upstream
            self.proxy('POST', False, upstream, self.path[len(prefix):], self.rfile.read(size))
        elif path in POST_ROUTES:
            self.proxy('POST', False)
        else:
            self.send_error(404)

    def vision_request(self, path):
        client = self.state.vision_start if path.endswith('/start') else self.state.vision_stop
        if not client.wait_for_service(timeout_sec=0.2):
            self.send_error(503, '视觉任务节点未连接')
            return
        request = MissionStart.Request() if path.endswith('/start') else Trigger.Request()
        if path.endswith('/start'):
            query = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
            for name in ('target_id', 'class_id', 'behavior', 'color'):
                values = query.get(name, [''])
                if len(values) != 1 or len(values[0]) > 128:
                    self.send_error(400, '视觉任务参数无效')
                    return
                setattr(request, name, values[0])
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3):
            self.send_error(504, '视觉任务响应超时，请查看任务状态')
            return
        try:
            result = future.result()
            body = result.message.encode('utf-8')
            self.send_response(200 if result.success else 409)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except Exception:
            self.send_error(503, '视觉任务服务不可用')

    def proxy(self, method, stream, upstream=None, path=None, body=None):
        started = False
        try:
            request = Request((upstream or self.upstream) + (path or self.path), method=method, data=body)
            if body is not None:
                request.add_header('Content-Type', 'application/json')
            try:
                response = OPENER.open(request, timeout=5)
            except HTTPError as error:
                response = error
            with response:
                path = urlsplit(self.path).path
                if not stream:
                    body = response.read()
                self.send_response(response.status)
                self.send_header('Content-Type', response.headers.get('Content-Type', 'application/octet-stream'))
                if path == '/map.png':
                    geometry = response.headers.get('X-BX72-Map')
                    if geometry is not None:
                        self.send_header('X-BX72-Map', geometry)
                    self.send_header('X-BX72-Map-Overlay', 'unavailable')
                if path == '/frame.jpg':
                    frame = response.headers.get('X-BX72-Frame')
                    if frame is not None:
                        self.send_header('X-BX72-Frame', frame)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                if not stream:
                    self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                started = True
                if not stream:
                    self.wfile.write(body)
                else:
                    while True:
                        chunk = response.read1(65536)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, URLError):
            if not started and not self.wfile.closed:
                try:
                    self.send_error(503, 'Jetson video or command endpoint unavailable')
                except OSError:
                    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--upstream', default=UPSTREAM)
    parser.add_argument('--swarm-upstream', default=SWARM_UPSTREAM)
    parser.add_argument('--acg720-upstream', default=ACG720_UPSTREAM)
    args = parser.parse_args()
    rclpy.init()
    node = DashboardState()
    Handler.state = node
    Handler.upstream = args.upstream.rstrip('/')
    Handler.swarm_upstream = args.swarm_upstream.rstrip('/')
    Handler.acg720_upstream = args.acg720_upstream.rstrip('/')
    ThreadingHTTPServer.request_queue_size = 32
    server = ThreadingHTTPServer((args.bind, args.port), partial(Handler, directory=str(PUBLIC)))
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    print(f'BX72 dashboard http://{args.bind}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()

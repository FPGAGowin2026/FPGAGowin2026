"""ACG720 六足的单实例 UDP 接收、ROS 2 发布与集群 HTTP 入口。"""

import argparse
from array import array
from ipaddress import IPv4Address
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import signal
import socket
import sys
import threading
import time
from urllib.parse import urlsplit

from .acg720_receiver import IMU_STALE_SECONDS, UdpCameraReceiver, _image_from_rgb565


ACTIONS = {'stop': 0, 'forward': 1, 'backward': 2, 'turn_left': 3,
           'turn_right': 4, 'strafe_left': 5, 'strafe_right': 6, 'release': 7,
           'stand': 8, 'home': 8, 'tripod': 9, 'ripple': 10}


def scan_bins(points, count=720, self_radius_m=0.):
    # D500 顺时针角度转为 ROS 逆时针角度；缺测保留 NaN，不当作自由空间。
    ranges, intensities = [float('nan')] * count, [0.] * count
    for angle, millimeters, intensity in points:
        distance = millimeters / 1000.
        if not 0.02 <= distance <= 12.:
            continue
        index = round((-angle % 360.) * count / 360.) % count
        if not math.isfinite(ranges[index]) or distance < ranges[index]:
            ranges[index], intensities[index] = distance, float(intensity)
    # Occluded returns are unknown; removing the nearest bin must not reveal a farther return.
    for index, distance in enumerate(ranges):
        if distance <= self_radius_m:
            ranges[index], intensities[index] = float('nan'), 0.
    return ranges, intensities


class Acg720Bridge:
    def __init__(self, bind='0.0.0.0', port=6102, control_source='192.0.2.111',
                 destination='192.0.2.10', control_port=6103, *,
                 control_source_port=0, telemetry_source_port=None):
        self.receiver = UdpCameraReceiver(bind, port, source_address=destination,
                                          source_port=telemetry_source_port)
        self.control_source = control_source
        self.control_source_port = control_source_port
        self.destination = (destination, control_port)
        self.lock = threading.Lock()
        self.sender = None
        self.held_code = None
        self.lease_deadline = 0.
        self.owner = None
        self.error = None
        self.closed = False
        self.commands_sent = 0
        self.last_sent_command = None
        self.last_sent_at = None
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.keepalive, daemon=True)
        self.jpeg_lock = threading.Lock()
        self.jpeg_version, self.jpeg_bytes = -1, None

    def start(self):
        self.receiver.start()
        self.worker.start()

    def send(self, code):
        if self.closed:
            raise ConnectionError('通信已关闭')
        if self.sender is None:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.bind((self.control_source, self.control_source_port))
            except OSError:
                sender.close()
                raise
            self.sender = sender
        self.sender.sendto(b'HEX1' + bytes([code]), self.destination)
        self.commands_sent += 1
        self.last_sent_command, self.last_sent_at = code, time.monotonic()

    def control(self, data):
        if not isinstance(data, dict):
            raise ValueError('请求体必须为对象')
        action = data.get('action')
        if action not in ACTIONS:
            raise ValueError('动作无效')
        pressed = data.get('pressed', True)
        if not isinstance(pressed, bool):
            raise ValueError('pressed 必须为布尔值')
        code = ACTIONS[action]
        released_code = code if 1 <= code <= 6 and not pressed else None
        if released_code is not None:
            code = 0
        lease = data.get('lease_ms', 500)
        if type(lease) is not int or not 100 <= lease <= 5000:
            raise ValueError('lease_ms 必须在 100..5000 范围')
        owner = data.get('client_id', 'manual')
        if not isinstance(owner, str) or not 1 <= len(owner) <= 128:
            raise ValueError('client_id 无效')
        with self.lock:
            lease_active = self.held_code is not None and time.monotonic() < self.lease_deadline
            ignored = released_code is not None and (
                not lease_active or self.owner != owner or self.held_code != released_code)
            if not ignored:
                if code not in (0, 7):
                    if self.receiver.snapshot()['teleop_state'] != 'READY':
                        raise ConnectionError('尚未收到新鲜的 FPGA HXS1 状态')
                    if lease_active and self.owner != owner:
                        raise ValueError('另一客户端正在控制 ACG720')
                try:
                    self.send(code)
                except OSError as exc:
                    self.held_code, self.owner = None, None
                    self.error = str(exc)
                    raise
                self.error = None
                if 1 <= code <= 6:
                    self.held_code, self.owner = code, owner
                    self.lease_deadline = time.monotonic() + lease / 1000.
                elif code < 9:
                    self.held_code, self.owner = None, None
        # HEX1 没有逐命令 ACK；HXS1 是异步设备状态，不能作为此调用的确认。
        return {'accepted': True, 'robot_id': 'acg720', 'command': code,
                'delivery': 'ignored' if ignored else 'sent', 'acknowledged': False, 'status': self.status()}

    def keepalive(self):
        while not self.stop_event.wait(.1):
            with self.lock:
                if self.held_code is None:
                    continue
                expired = (time.monotonic() >= self.lease_deadline or
                           self.receiver.snapshot()['teleop_state'] != 'READY')
                try:
                    self.send(0 if expired else self.held_code)
                except OSError as exc:
                    self.error = str(exc)
                    expired = True
                if expired:
                    self.held_code, self.owner = None, None

    def status(self):
        stats = self.receiver.snapshot()
        imu_age, preview_age = stats['latest_imu_age_s'], stats['preview_age_s']
        with self.lock:
            transport = {
                'commands_sent': self.commands_sent,
                'last_sent_command': self.last_sent_command,
                'last_send_age_s': (None if self.last_sent_at is None else
                                    max(0., time.monotonic() - self.last_sent_at)),
                'control_source': self.control_source,
                'control_source_port': (self.sender.getsockname()[1] if self.sender is not None
                                        and not self.closed else self.control_source_port),
                'control_destination': self.destination[0], 'control_port': self.destination[1],
                'held_command': self.held_code, 'closed': self.closed,
                'error': stats['receiver_error'] or self.error,
            }
        return {
            'robot_id': 'acg720', 'name': 'ACG720 六足', 'robot_type': 'hexapod',
            'host': '198.51.100.39', 'namespace': '/acg720', 'transport': 'HEX1/UDP',
            'fpga_connected': stats['teleop_state'] == 'READY',
            'teleop_state': stats['teleop_state'], 'teleop_age_s': stats['teleop_age_s'],
            'pwm_enabled': stats['teleop_pwm_enabled'],
            'requested_command': stats['teleop_requested_command'],
            'active_gait': stats['teleop_active_gait'], 'frame_index': stats['teleop_frame_index'],
            'ripple_selected': stats['teleop_ripple_selected'],
            'trajectory_ripple': stats['teleop_trajectory_ripple'],
            'homing': stats['teleop_homing'],
            'teleop_status_version': stats['teleop_status_version'],
            'rx_debug': stats['teleop_rx_debug'],
            'imu_ready': imu_age is not None and imu_age <= IMU_STALE_SECONDS and bool(stats['latest_imu_ready']),
            'imu_age_s': imu_age, 'imu_raw': stats['latest_imu_raw'],
            'lidar_state': stats['lidar_state'], 'lidar_age_s': stats['lidar_age_s'],
            'lidar_points': len(stats['lidar_completed_sweep']),
            'camera_ready': preview_age is not None and preview_age <= .75,
            'camera_age_s': preview_age, 'preview_version': stats['preview_version'],
            'packets_received': stats['packets_received'], 'complete_frames': stats['complete_frames'],
            'valid_packets': stats['valid_packets'], 'invalid_packets': stats['invalid_packets'],
            'filtered_packets': stats['filtered_packets'],
            'receive_buffer_bytes': stats['receive_buffer_bytes'],
            'telemetry_source_port': stats['source_port'],
            'telemetry_bind': self.receiver.bind_address, 'telemetry_port': self.receiver.port,
            'navigation_ready': False,
            'navigation_reason': '尚未接入里程计、机身外参、地图与导航',
            **transport,
        }

    def jpeg(self, preview=None):
        version, pixels = self.receiver.preview() if preview is None else preview
        if pixels is None:
            return None
        with self.jpeg_lock:
            if version == self.jpeg_version:
                return self.jpeg_bytes
            output = BytesIO()
            _image_from_rgb565(pixels).save(output, format='JPEG', quality=85)
            encoded = output.getvalue()
            if version > self.jpeg_version:
                self.jpeg_version, self.jpeg_bytes = version, encoded
            return encoded

    def close(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1)
        with self.lock:
            if self.closed:
                return
            if self.held_code is not None:
                try:
                    self.send(0)
                except OSError:
                    pass
            if self.sender is not None:
                self.sender.close()
            self.held_code, self.owner = None, None
            self.closed = True
        self.receiver.stop()


def create_server(bridge, bind, port):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, status, value, content_type='application/json; charset=utf-8'):
            body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/status':
                self.respond(200, bridge.status())
            elif path == '/frame.jpg':
                frame = bridge.jpeg()
                self.respond(200, frame, 'image/jpeg') if frame else self.respond(503, {'error': '等待相机'})
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != '/api/control':
                self.send_error(404)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('请求体长度无效')
                data = json.loads(self.rfile.read(size))
                self.respond(200, bridge.control(data))
            except (ValueError, TypeError) as exc:
                self.respond(400, {'error': str(exc)})
            except OSError as exc:
                self.respond(503, {'error': str(exc)})

        def log_message(self, *_args):
            pass

    return ThreadingHTTPServer((bind, port), Handler)


def create_ros_node(bridge, **kwargs):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CompressedImage, Imu, LaserScan
    from std_msgs.msg import String

    class Sensors(Node):
        def __init__(self):
            super().__init__('fpga_bridge', namespace='acg720', **kwargs)
            self.lidar_self_radius_m = self.declare_parameter('lidar_self_radius_m', 0.).value
            if not math.isfinite(self.lidar_self_radius_m) or self.lidar_self_radius_m < 0:
                raise ValueError('lidar_self_radius_m must be finite and nonnegative')
            self.imu = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
            self.scan = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
            self.camera = self.create_publisher(CompressedImage, 'camera/image/compressed', qos_profile_sensor_data)
            self.alignment_camera = self.create_publisher(
                CompressedImage, 'camera/alignment_image/compressed', qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, 'status', 1)
            self.command_result = self.create_publisher(String, 'command_result', 10)
            self.create_subscription(String, 'command', self.on_command, 1)
            self.last_imu, self.last_sweep, self.last_preview = -1, -1, -1
            self.last_alignment_frame = -1
            self.create_timer(.02, self.publish_sensors)
            self.create_timer(.5, self.publish_alignment_image)
            self.create_timer(1., self.publish_status)

        def publish_alignment_image(self):
            if not self.alignment_camera.get_subscription_count():
                return
            stats = bridge.receiver.snapshot()
            version, pixels = stats['frame_version'], stats['latest_completed_frame']
            if pixels is None or version == self.last_alignment_frame:
                return
            # 对齐只接受完整帧，不能把丢包修补后的预览当作标定图像。
            encoded = BytesIO()
            _image_from_rgb565(pixels).save(encoded, format='PNG', compress_level=1)
            message = CompressedImage()
            message.header.stamp = Time(nanoseconds=stats['latest_completed_frame_stamp_ns']).to_msg()
            message.header.frame_id = 'acg720_camera_optical'
            message.format, message.data = 'rgb8; png compressed rgb8', array('B', encoded.getvalue())
            self.alignment_camera.publish(message)
            self.last_alignment_frame = version

        def on_command(self, message):
            try:
                result = bridge.control(json.loads(message.data))
            except (ValueError, TypeError, OSError) as exc:
                result = {'accepted': False, 'robot_id': 'acg720', 'delivery': 'failed',
                          'acknowledged': False, 'error': str(exc)}
            self.command_result.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))

        def publish_status(self):
            self.status_pub.publish(String(data=json.dumps(bridge.status(), ensure_ascii=False, allow_nan=False)))

        def publish_sensors(self):
            stats = bridge.receiver.snapshot()
            if (stats['imu_version'] != self.last_imu and stats['latest_imu_ready']
                    and stats['latest_imu_age_s'] <= IMU_STALE_SECONDS):
                self.last_imu = stats['imu_version']
                msg = Imu()
                msg.header.stamp = Time(nanoseconds=stats['latest_imu_stamp_ns']).to_msg()
                msg.header.frame_id = 'acg720_imu'
                msg.orientation_covariance[0] = -1.
                raw = stats['latest_imu_raw']
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [math.radians(v * .035) for v in raw[:3]]
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [v * .000244 * 9.80665 for v in raw[3:]]
                self.imu.publish(msg)
            if stats['lidar_sweeps'] != self.last_sweep and stats['lidar_completed_sweep'] and stats['lidar_state'] == 'READY':
                self.last_sweep = stats['lidar_sweeps']
                msg = LaserScan()
                # A complete sweep keeps its first packet receipt time across delayed publishers.
                msg.header.stamp = Time(nanoseconds=stats['lidar_completed_sweep_stamp_ns']).to_msg()
                msg.header.frame_id = 'acg720_lidar'
                msg.angle_min, msg.angle_increment = 0., 2. * math.pi / 720
                msg.angle_max = 719 * msg.angle_increment
                msg.range_min, msg.range_max = .02, 12.
                msg.scan_time = stats['lidar_completed_sweep_time_s']
                # UDP 快照不含每束的主机时钟时间；不伪造逐束去畸变时戳。
                msg.time_increment = 0.
                msg.ranges, msg.intensities = scan_bins(
                    stats['lidar_completed_sweep'], self_radius_m=self.lidar_self_radius_m)
                self.scan.publish(msg)
            if stats['preview_version'] != self.last_preview and self.camera.get_subscription_count():
                frame = bridge.jpeg((stats['preview_version'], stats['latest_preview_frame']))
                if frame is not None:
                    self.last_preview = stats['preview_version']
                    msg = CompressedImage()
                    msg.header.stamp = Time(nanoseconds=stats['latest_preview_stamp_ns']).to_msg()
                    msg.header.frame_id = 'acg720_camera_optical'
                    msg.format, msg.data = 'jpeg', array('B', frame)
                    self.camera.publish(msg)

    return Sensors()


def ipv4_address(value):
    try:
        return str(IPv4Address(value))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('必须为 IPv4 地址') from exc


def port_number(value):
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('端口必须为整数') from exc
    if not 0 <= port <= 65535:
        raise argparse.ArgumentTypeError('端口必须在 0..65535 范围')
    return port


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', type=ipv4_address, default='0.0.0.0', help='遥测接收本机地址')
    parser.add_argument('--port', type=port_number, default=6102, help='遥测接收端口')
    parser.add_argument('--http-bind', type=ipv4_address, default='0.0.0.0')
    parser.add_argument('--http-port', type=port_number, default=8766)
    parser.add_argument('--control-source', type=ipv4_address, default='192.0.2.111')
    parser.add_argument('--control-source-port', type=port_number, default=0, help='0 使用临时源端口')
    parser.add_argument('--control-destination', '--fpga-address', type=ipv4_address, default='192.0.2.10')
    parser.add_argument('--control-port', type=port_number, default=6103)
    parser.add_argument('--telemetry-source-port', type=port_number, default=5000,
                        help='FPGA 遥测源端口；0 关闭源端口筛选')
    parser.add_argument('--http-only', action='store_true', help='系统依赖安装期间仅运行 HTTP/UDP 接入')
    return parser


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if '--ros-args' in argv:
        from rclpy.utilities import remove_ros_args
        app_args = remove_ros_args(args=['acg720_bridge', *argv])[1:]
    else:
        app_args = argv
    parser = build_parser()
    args = parser.parse_args(app_args)
    if not args.control_port:
        parser.error('--control-port 必须为 1..65535')
    bridge, server, worker = None, None, None
    node = None
    ros_initialized = False
    try:
        bridge = Acg720Bridge(args.bind, args.port, args.control_source, args.control_destination,
                             args.control_port, control_source_port=args.control_source_port,
                             telemetry_source_port=args.telemetry_source_port or None)
        server = create_server(bridge, args.http_bind, args.http_port)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        bridge.start()
        worker.start()
        if args.http_only:
            signal.signal(signal.SIGTERM, lambda *_: bridge.stop_event.set())
            bridge.stop_event.wait()
        else:
            import rclpy
            from rclpy.executors import ExternalShutdownException
            rclpy.init(args=argv)
            ros_initialized = True
            node = create_ros_node(bridge)
            try:
                rclpy.spin(node)
            except ExternalShutdownException:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            if worker is not None and worker.is_alive():
                server.shutdown()
                worker.join(timeout=2)
            server.server_close()
        if bridge is not None:
            bridge.close()
        if node is not None:
            node.destroy_node()
        if ros_initialized:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()

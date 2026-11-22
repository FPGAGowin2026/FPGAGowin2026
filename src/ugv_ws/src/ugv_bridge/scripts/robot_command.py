#!/usr/bin/env python3
"""Send local navigation goals and velocity requests to the BX72 executor."""

from binascii import crc_hqx
import json
import math
import queue
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, String


VELOCITY_LEASE_S = 0.15
CORNER_TOLERANCE_MM = 20


def crc16(data):
    return crc_hqx(data, 0xffff)


def packet(kind, sequence, token, payload):
    if len(payload) != 10:
        raise ValueError('BX72 control payload must be 10 bytes')
    body = struct.pack('>4sBBHIQ', b'BX72', 1, kind, 32, sequence, token) + payload
    return body + struct.pack('>H', crc16(body))


def ack_matches(request, reply):
    return (len(reply) == 32 and reply[:5] == request[:5] and
            reply[5] == request[5] + 1 and reply[6:30] == request[6:30] and
            crc16(reply[:30]) == struct.unpack_from('>H', reply, 30)[0])


def encode_task(task_id, domain, boot, x, y, hold_yaw, final_yaw,
                speed, acceleration, tolerance, yaw_tolerance, sent, expiry,
                standalone=False, intermediate=False, confirmation=None, position_only=False):
    q16 = lambda value: int(round(value * 65536))
    values = [q16(x), q16(y), q16(hold_yaw), q16(final_yaw)]
    if not all(-0x80000000 <= value <= 0x7fffffff for value in values):
        raise ValueError('local goal is outside the wire coordinate range')
    body = bytearray(struct.pack('>BBBBIIIiiiiHHHHQQQH6x', 1, 1, 3 if standalone else 2,
                     (1 if standalone else 0) | (2 if position_only else 0) | (4 if intermediate else 0) |
                     (8 if confirmation else 0), task_id, domain, boot,
                     *values, speed, acceleration, tolerance, yaw_tolerance,
                     expiry, 0, sent, 0))
    if confirmation:
        start_x, start_y, until = confirmation
        start = (q16(start_x), q16(start_y))
        if not all(-0x80000000 <= value <= 0x7fffffff for value in start):
            raise ValueError('confirmed start is outside the wire coordinate range')
        struct.pack_into('>ii', body, 48, *start)
        struct.pack_into('>I', body, 64, until)
    return bytes(body)


class NavigationTelemetryStale(TimeoutError):
    pass


class CommandExecutor(SingleThreadedExecutor):
    """Humble drops DDS timestamps before calling a normal Twist subscriber."""

    def __init__(self, manual_subscription):
        super().__init__()
        self.manual_subscription = manual_subscription

    def _take_subscription(self, sub):
        if sub is not self.manual_subscription:
            return super()._take_subscription(sub)
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)


class RobotCommand(Node):
    def __init__(self):
        super().__init__('robot_command')
        self.address = self.declare_parameter('fpga_address', '203.0.113.2').value
        self.source = self.declare_parameter('source_address', '203.0.113.4').value
        self.port = self.declare_parameter('fpga_port', 5000).value
        self.source_port = self.declare_parameter('source_port', 6103).value
        self.max_speed = self.declare_parameter('max_speed_mm_s', 800).value
        self.cruise_speed = self.declare_parameter('cruise_speed_mm_s', 600).value
        self.explore_speed = self.declare_parameter('explore_speed_mm_s', 600).value
        self.accel = self.declare_parameter('max_accel_mm_s2', 200).value
        self.tolerance = self.declare_parameter('goal_tolerance_mm', 100).value
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance_mrad', 100).value
        self.duration = self.declare_parameter('task_duration_s', 120).value
        self.standalone = self.declare_parameter('continue_without_link', False).value
        if not (300 <= self.max_speed <= 800 and 300 <= self.cruise_speed <= 800 and
                300 <= self.explore_speed <= 800 and
                1 <= self.accel <= 1000 and
                20 <= self.tolerance <= 500 and 10 <= self.yaw_tolerance <= 1000 and
                1 <= self.duration <= 120):
            raise ValueError('invalid navigation parameters')
        self.status_pub = self.create_publisher(String, 'fpga/command_status', 10)
        self.create_subscription(String, 'fpga/navigation_status', self.on_status, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'fpga/local_goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, 'fpga/local_reference', self.on_reference, 10)
        self.create_subscription(Path, 'fpga/confirmed_goal', self.on_confirmed_goal, 10)
        self.create_subscription(Path, 'fpga/confirmed_reference', self.on_confirmed_reference, 10)
        self.create_subscription(Path, 'fpga/explore_reference',
                                 lambda msg: self.on_reference_path(msg, corner=False), 10)
        self.create_subscription(Path, 'fpga/corner_reference',
                                 lambda msg: self.on_reference_path(msg, corner=True), 10)
        self.manual_subscription = self.create_subscription(Twist, 'cmd_vel', self.on_manual, 1)
        self.create_subscription(TwistStamped, 'fpga/vision_velocity', self.on_vision_velocity, 1)
        self.create_subscription(TwistStamped, 'fpga/waypoint_velocity', self.on_waypoint_velocity, 1)
        self.create_subscription(Empty, 'fpga/stop', self.on_stop, 10)
        self.lock = threading.Lock()
        self.last_stop_received_ns = 0
        self.last_manual_at = 0.0
        self.last_manual_source_ns = 0
        self.last_manual_stamp_ns = 0
        self.last_vision_stamp_ns = 0
        self.vision_moving = False
        self.stream_generation = 0
        self.authorize_waypoints_on_stop = False
        self.last_waypoint_stamp_ns = 0
        self.waypoint_stop_confirmed = False
        self.device_status = None
        self.device_at = 0.0
        self.pose_yaw = None
        self.pose_at = 0.0
        self.pending = queue.Queue(maxsize=1)
        self.stop_request = threading.Event()
        self.link_recovering = False
        self.closing = threading.Event()
        self.socket = None
        self.socket_opened_at = 0.0
        self.token = 0
        self.session_epoch = None
        self.sequence = 0
        self.active = False
        self.active_task_id = 0
        self.task_submitted_at = 0.0
        self.task_confirmed = False
        self.velocity_until = 0.0
        self.last_velocity_report = None
        self.confirmation_fields = {}
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def report(self, state, detail='', **fields):
        message = String()
        message.data = json.dumps({'state': state, 'detail': detail,
                                   'task_id': self.active_task_id, **self.confirmation_fields,
                                   **fields}, ensure_ascii=False)
        self.status_pub.publish(message)

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict) or any(
                    type(status.get(key)) is not int or not 0 <= status[key] < (1 << bits)
                    for key, bits in (('boot_id', 32), ('odom_domain', 32), ('session_token', 64),
                                      ('task_id', 32), ('last_task_id', 32), ('now_us', 32),
                                      ('status', 8), ('mode', 8))):
                return
            with self.lock:
                confirmed = self.confirmation_fields
                if confirmed:
                    remaining = (confirmed['confirmed_until_us'] - status['now_us']) & 0xffffffff
                    if (confirmed['confirmation_boot_id'] != status['boot_id'] or
                            confirmed['confirmation_odom_domain'] != status['odom_domain'] or
                            not 0 < remaining < 0x80000000):
                        self.confirmation_fields = {}
                self.device_status, self.device_at = status, time.monotonic()
        except (ValueError, TypeError):
            pass

    @staticmethod
    def yaw(q):
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            raise ValueError('non-finite orientation')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 0.5 or norm > 1.5:
            raise ValueError('invalid orientation quaternion')
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def on_odom(self, message):
        try:
            yaw = self.yaw(message.pose.pose.orientation)
            with self.lock:
                self.pose_yaw, self.pose_at = yaw, time.monotonic()
        except ValueError:
            pass

    def on_goal(self, message):
        self.accept_goal(message, False)

    def on_reference(self, message):
        self.accept_goal(message, True)

    def on_confirmed_goal(self, message):
        self.accept_confirmed(message, False)

    def on_confirmed_reference(self, message):
        self.accept_confirmed(message, True)

    def on_reference_path(self, message, corner):
        speed = min(self.cruise_speed, self.explore_speed) if corner else self.explore_speed
        if len(message.poses) == 1:
            self.accept_goal(message.poses[0], not corner, speed=speed, position_only=corner)
        else:
            self.accept_confirmed(message, not corner, speed=speed, position_only=corner)

    def accept_confirmed(self, message, intermediate, speed=None, position_only=False):
        if len(message.poses) != 2 or message.poses[0].header.frame_id != 'odom':
            self.report('rejected', 'invalid confirmed start')
            return
        start = message.poses[0]
        stamp = start.header.stamp.sec * 1000000000 + start.header.stamp.nanosec
        x, y = start.pose.position.x, start.pose.position.y
        if not math.isfinite(x) or not math.isfinite(y) or not stamp:
            self.report('rejected', 'invalid confirmed start')
            return
        self.accept_goal(message.poses[1], intermediate, (x, y, stamp), speed, position_only)

    def accept_goal(self, message, intermediate, confirmation=None, speed=None, position_only=False):
        try:
            if message.header.frame_id != 'odom':
                raise ValueError('local goal must be in the odom frame')
            x, y = message.pose.position.x, message.pose.position.y
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('non-finite local goal')
            speed = self.cruise_speed if speed is None else speed
            pending_stop = self.stop_request.is_set()
            goal_stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            goal = (x, y, self.yaw(message.pose.orientation), intermediate, confirmation,
                    min(speed, self.max_speed), position_only, goal_stamp)
            if goal_stamp <= max(self.last_stop_received_ns, self.last_manual_stamp_ns):
                raise ValueError('goal must be stamped after the pending stop' if pending_stop else
                                 'goal must be stamped after the last stop')
            self.enqueue('goal', goal, allow_while_stopping=pending_stop)
        except ValueError as error:
            self.report('rejected', str(error))

    def on_manual(self, sample):
        message, info = sample
        source_ns = info['source_timestamp']
        age_ns = time.time_ns() - source_ns
        if not 0 <= age_ns < VELOCITY_LEASE_S * 1e9:
            return
        x, y, yaw = message.linear.x, message.linear.y, message.angular.z
        if not all(math.isfinite(v) for v in (x, y, yaw)) or abs(x) > 0.4 or abs(y) > 0.4 or abs(yaw) > 1:
            self.report('rejected', 'manual command exceeds FPGA protocol bounds')
            return
        # The wire accepts 1 rad/s but the RV32 output commit is limited to 0.8.
        values = (round(x * 1000), round(y * 1000), round(max(-0.8, min(0.8, yaw)) * 1000))
        valid_until = time.monotonic() + VELOCITY_LEASE_S - age_ns * 1e-9
        stamp_ns = self.get_clock().now().nanoseconds - age_ns
        with self.lock:
            if (source_ns <= self.last_manual_source_ns or stamp_ns <= self.last_stop_received_ns or
                    time.monotonic() >= valid_until):
                return
            if any(values) and self.link_recovering:
                return
            self.last_manual_source_ns = source_ns
            self.last_manual_stamp_ns = stamp_ns
            self.last_manual_at = valid_until - VELOCITY_LEASE_S
            self.waypoint_stop_confirmed = False
            self.vision_moving = False
            if any(values):
                self.stream_generation += 1
                self.authorize_waypoints_on_stop = False
                # A press during the stop handshake takes ownership immediately.
                # Only a fresh input after that handshake may start moving.
                item = None if self.stop_request.is_set() else ('manual', (values, valid_until, self.stream_generation))
                self.replace_pending(item)
            else:
                self.request_stop_locked(False)
        self.report('manual_takeover', stamp_ns=stamp_ns)

    def on_vision_velocity(self, message):
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1000000000:
            return
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        now = time.monotonic()
        twist = message.twist
        forward, turn = twist.linear.x, twist.angular.z
        if (not 0 <= age < VELOCITY_LEASE_S or
                not (0 <= forward <= 0.4 and -1.0 <= turn <= 1.0 and twist.linear.y == 0)):
            return
        values = (round(forward * 1000), 0, round(max(-0.8, min(0.8, turn)) * 1000))
        with self.lock:
            if (stamp_ns <= max(self.last_stop_received_ns, self.last_vision_stamp_ns) or
                    now - self.last_manual_at < 1.0 or self.stop_request.is_set() or self.link_recovering):
                return
            self.last_vision_stamp_ns = stamp_ns
            if any(values):
                self.vision_moving = True
                self.waypoint_stop_confirmed = False
                self.stream_generation += 1
                self.replace_pending(('vision', (values, now - age + VELOCITY_LEASE_S, self.stream_generation)))
            elif self.vision_moving:
                self.request_stop_locked(False)

    def on_waypoint_velocity(self, message):
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1000000000:
            return
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        received = time.monotonic()
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        twist = message.twist
        forward, turn = twist.linear.x, twist.angular.z
        if (not 0 <= age < VELOCITY_LEASE_S or
                not (0 <= forward <= 0.50 and twist.linear.y == 0 and -0.80 <= turn <= 0.80)):
            return
        values = (round(forward * 1000), 0, round(turn * 1000))
        self.enqueue('waypoint', (values, received - age + VELOCITY_LEASE_S, stamp_ns))

    def on_stop(self, _message):
        with self.lock:
            self.request_stop_locked(_message is not None)

    def request_stop_locked(self, authorize_waypoints):
        self.last_stop_received_ns = self.get_clock().now().nanoseconds
        self.waypoint_stop_confirmed = False
        self.authorize_waypoints_on_stop = authorize_waypoints
        self.vision_moving = False
        self.stream_generation += 1
        self.stop_request.set()
        self.replace_pending(None)

    def replace_pending(self, item):
        try:
            self.pending.get_nowait()
        except queue.Empty:
            pass
        if item is not None:
            self.pending.put_nowait(item)

    def enqueue(self, kind, data, allow_while_stopping=False):
        with self.lock:
            if self.link_recovering or (self.stop_request.is_set() and not allow_while_stopping):
                return
            if kind == 'goal' and time.monotonic() - self.last_manual_at < VELOCITY_LEASE_S:
                return
            if kind == 'waypoint':
                _, valid_until, stamp_ns = data
                now = time.monotonic()
                if (now >= valid_until or stamp_ns <= max(self.last_stop_received_ns, self.last_waypoint_stamp_ns) or
                        not self.waypoint_session_locked(now)):
                    return
            try:
                pending = self.pending.get_nowait()
            except queue.Empty:
                pending = None
            if kind == 'waypoint':
                if pending is not None and pending[0] != 'waypoint':
                    self.pending.put_nowait(pending)
                    return
                self.last_waypoint_stamp_ns = stamp_ns
            else:
                self.vision_moving = False
                self.last_vision_stamp_ns = max(self.last_vision_stamp_ns, data[-1])
                self.stream_generation += 1
                self.waypoint_stop_confirmed = False
            self.pending.put_nowait((kind, data))

    def send(self, kind, payload, retries=2, timeout_s=0.04, deadline=None, valid_until=None,
             waypoint_stamp_ns=None, stream_generation=None, goal_stamp_ns=None, confirmation=None):
        self.sequence = (self.sequence + 1) & 0xffffffff
        request = packet(kind, self.sequence, self.token, payload)
        attempts_made = 0
        for attempt in range(retries):
            if deadline is not None and time.monotonic() >= deadline:
                break
            # FPGA re-ACKs duplicate task fragments, but rejects repeated
            # mode and velocity packets with the same control sequence.
            if attempt and kind in (3, 5):
                self.sequence = (self.sequence + 1) & 0xffffffff
                request = packet(kind, self.sequence, self.token, payload)
            if goal_stamp_ns is not None:
                with self.lock:
                    if (self.stop_request.is_set() or
                            goal_stamp_ns <= max(self.last_manual_stamp_ns, self.last_stop_received_ns)):
                        return False
                    self.socket.send(request)
                    if confirmation is not None:
                        # The final fragment may reach the FPGA even when its ACK is lost.
                        self.confirmation_fields = confirmation
            elif valid_until is not None:
                with self.lock:
                    now = time.monotonic()
                    if stream_generation is not None and (
                            stream_generation != self.stream_generation or now >= valid_until):
                        return False
                    if waypoint_stamp_ns is not None and (
                            self.stop_request.is_set() or now >= valid_until or
                            waypoint_stamp_ns != self.last_waypoint_stamp_ns or
                            waypoint_stamp_ns <= self.last_stop_received_ns or
                            not self.waypoint_session_locked(now)):
                        return
                    if self.stop_request.is_set() or now >= valid_until:
                        raise InterruptedError('manual command expired or was stopped')
                    self.socket.send(request)
                    if waypoint_stamp_ns is not None:
                        # Even without an ACK, this packet may have reached the FPGA.
                        self.velocity_until = valid_until
                        self.last_velocity_report = None
                    elif stream_generation is not None:
                        self.velocity_until = valid_until
            else:
                self.socket.send(request)
            attempts_made += 1
            end = time.monotonic() + timeout_s
            if deadline is not None:
                end = min(end, deadline)
            if valid_until is not None:
                end = min(end, valid_until)
            while time.monotonic() < end:
                try:
                    self.socket.settimeout(max(0.001, end - time.monotonic()))
                    reply = self.socket.recv(512)
                    if ack_matches(request, reply):
                        return True
                except socket.timeout:
                    break
        raise TimeoutError(f'FPGA did not acknowledge control packet (type={kind}, sequence={self.sequence}, attempts={attempts_made})')

    def session_current(self):
        with self.lock:
            return self.session_current_locked(time.monotonic())

    def session_current_locked(self, now):
        current = self.device_status
        return bool(self.socket is not None and current and self.device_at > self.socket_opened_at and
                    now - self.device_at < 0.3 and
                    int(current['session_token']) == self.token and
                    self.session_epoch == (int(current['boot_id']), int(current['odom_domain'])) and
                    all(self.session_epoch))

    def waypoint_session_locked(self, now):
        return (self.waypoint_stop_confirmed and not self.active and
                now - self.last_manual_at >= VELOCITY_LEASE_S and
                self.session_current_locked(now) and self.device_status.get('mode') == 1)

    def close_socket(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.waypoint_stop_confirmed = False

    def close_session(self):
        self.close_socket()
        self.session_epoch = None

    def ensure_session(self, allow_stop=False):
        if self.session_epoch is not None:
            with self.lock:
                current, at = self.device_status, self.device_at
            if (current and time.monotonic() - at < 0.5 and
                    (int(current['session_token']) != self.token or not int(current['boot_id']) or
                     not int(current['odom_domain']) or
                     self.session_epoch != (int(current['boot_id']), int(current['odom_domain'])))):
                self.close_session()
        if self.socket is None:
            # Rebinding the host UDP port does not change the board's session.
            token = self.token
            if self.session_epoch is None:
                with self.lock:
                    previous = int(self.device_status.get('session_token', 0)) if self.device_status else 0
                token = max(time.time_ns(), previous + 1, self.token + 1)
                if token >= 1 << 64:
                    raise ValueError('cannot allocate a new FPGA session')
            command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                command_socket.bind((self.source, self.source_port))
                command_socket.connect((self.address, self.port))
            except OSError:
                command_socket.close()
                raise
            self.socket_opened_at = time.monotonic()
            self.socket, self.token = command_socket, token
        if self.session_epoch is not None:
            return
        # Keep supplying zero packets: starting mid-scan can require almost
        # two 16 * 0.6 s phase sweeps, plus settling and verification.
        end = time.monotonic() + 24.0
        while time.monotonic() < end and not self.closing.is_set():
            if self.stop_request.is_set() and not allow_stop:
                raise InterruptedError('session setup interrupted by stop')
            try:
                self.send(3, bytes(10), retries=1)
            except TimeoutError:
                pass
            with self.lock:
                current, at = self.device_status, self.device_at
            if current and at > self.socket_opened_at and time.monotonic() - at < 0.3:
                device_token = int(current['session_token'])
                if device_token > self.token:
                    self.close_session()
                    raise TimeoutError('FPGA control session changed during setup')
                epoch = (int(current['boot_id']), int(current['odom_domain']))
                if device_token == self.token and all(epoch):
                    self.session_epoch = epoch
                    return
            time.sleep(0.025)
        raise TimeoutError('FPGA session did not appear in telemetry')

    def start_goal(self, goal):
        with self.lock:
            self.waypoint_stop_confirmed = False
        self.ensure_session()
        with self.lock:
            if goal[-1] <= max(self.last_manual_stamp_ns, self.last_stop_received_ns):
                return
            status, at = self.device_status, self.device_at
            yaw, pose_at = self.pose_yaw, self.pose_at
        now = time.monotonic()
        if not status or now - at > 0.5 or yaw is None or now - pose_at > 0.5:
            raise NavigationTelemetryStale('FPGA telemetry or odometry is stale')
        sent = (int(status['now_us']) + int((now - at) * 1000000)) & 0xffffffff
        expiry = (sent + self.duration * 1000000) & 0xffffffff
        confirmation = None
        if goal[4]:
            x, y, stamp = goal[4]
            elapsed = self.get_clock().now().nanoseconds - stamp
            if elapsed < 0 or elapsed >= 10000000000:
                raise ValueError('起点临时区域已过期')
            until = (sent + (10000000000 - elapsed) // 1000) & 0xffffffff
            confirmation = (x, y, until)
        task_id = (int(status['last_task_id']) + 1) & 0xffffffff or 1
        x, y, final_yaw, intermediate, _, speed, position_only, goal_stamp = goal
        # The streaming flag ends local tracking inside the board's 60 mm point radius.
        # A protected corner uses endpoint braking and finishes on position alone.
        tolerance = CORNER_TOLERANCE_MM if position_only else self.tolerance
        body = encode_task(task_id, int(status['odom_domain']), int(status['boot_id']),
                           x, y, yaw, final_yaw, speed, self.accel,
                           tolerance, self.yaw_tolerance, sent, expiry, self.standalone,
                           intermediate, confirmation, position_only)
        replacing = (self.active and self.session_current() and
                     status['task_id'] == self.active_task_id and status['status'] in (2, 3, 5))
        if not replacing:
            if not self.send(5, bytes([1]) + bytes(9), retries=4, timeout_s=0.02, goal_stamp_ns=goal_stamp):
                return
        begin = time.monotonic()
        confirmation_fields = ({
            'confirmation_boot_id': status['boot_id'],
            'confirmation_odom_domain': status['odom_domain'],
            'confirmed_until_us': confirmation[2],
        } if confirmation else None)
        try:
            for index in range(8):
                if not self.send(7, bytes([index]) + body[index*9:(index+1)*9],
                                 retries=4, timeout_s=0.02, deadline=begin + 0.18, goal_stamp_ns=goal_stamp,
                                 confirmation=confirmation_fields if index == 7 else None):
                    return
        except TimeoutError:
            if not replacing or not self.session_current():
                raise
            with self.lock:
                accepted = int(self.device_status['last_task_id']) == task_id
            if not accepted:
                self.report('reference_unconfirmed', 'replacement ACK missing; waiting for FPGA feedback',
                            goal_stamp_ns=goal_stamp)
                return
        self.active = True
        self.active_task_id = task_id
        self.task_submitted_at = time.monotonic()
        self.task_confirmed = False
        self.velocity_until = 0.0
        self.last_velocity_report = None
        self.report('submitted', 'task fragments acknowledged; execution requires FPGA status',
                    goal_stamp_ns=goal_stamp, speed_mm_s=speed)

    def stop(self):
        with self.lock:
            stop_stamp = self.last_stop_received_ns
            if not self.stop_request.is_set():
                self.authorize_waypoints_on_stop = self.waypoint_stop_confirmed
                self.stop_request.set()
            self.waypoint_stop_confirmed = False
        self.ensure_session(allow_stop=True)
        self.velocity_until = 0.0
        self.last_velocity_report = None
        missing_ack = False
        socket_error = None
        for kind in (3, 5):
            try:
                self.send(kind, bytes(10))
            except TimeoutError:
                missing_ack = True
            except OSError as error:
                socket_error = error
        if socket_error is not None:
            raise socket_error
        if missing_ack:
            # An ACK timeout alone cannot show that the device lost its token.
            self.stop_request.set()
            self.report('stop_unconfirmed', 'zero velocity or autonomous-disable ACK missing; retrying stop')
            return False
        if not self.session_current():
            self.stop_request.set()
            self.report('stop_unconfirmed', 'stop packets acknowledged; waiting for current-session telemetry')
            stopped_session = (self.token, self.session_epoch)
            # Both stop packets were received. Only probe until telemetry
            # confirms that session, or a session change requires a new stop.
            while not self.closing.wait(0.03):
                self.ensure_session(allow_stop=True)
                if (self.token, self.session_epoch) != stopped_session:
                    return False
                if self.session_current():
                    break
                try:
                    self.send(1, bytes(10), retries=1, timeout_s=0.02)
                except TimeoutError:
                    return False
            else:
                return False
        with self.lock:
            if self.last_stop_received_ns != stop_stamp or not self.session_current_locked(time.monotonic()):
                self.stop_request.set()
                return False
            self.active = False
            self.link_recovering = False
            # A manual timeout stops motion but does not hand control to navigation.
            self.waypoint_stop_confirmed = self.authorize_waypoints_on_stop
            self.authorize_waypoints_on_stop = False
            self.stop_request.clear()
            sample = self.device_status
        self.report('stop_requested', 'FPGA telemetry must confirm the stopped state',
                    stop_stamp_ns=stop_stamp, session_token=self.token,
                    stop_sample_us=sample['now_us'], stop_boot_id=sample['boot_id'],
                    stop_odom_domain=sample['odom_domain'])
        return True

    def run(self):
        next_probe = 0.0
        while not self.closing.is_set():
            stopping = self.stop_request.is_set()
            try:
                if stopping:
                    self.stop()
                    continue
                try:
                    wait = min(0.02, max(0.0, self.velocity_until-time.monotonic())) if self.velocity_until else 0.02
                    kind, data = self.pending.get(timeout=wait)
                except queue.Empty:
                    kind = None
                if kind == 'goal':
                    try:
                        if not self.stop_request.is_set():
                            self.start_goal(data)
                    finally:
                        with self.lock:
                            if (self.stop_request.is_set() and not self.link_recovering and self.pending.empty() and
                                    data[-1] > max(self.last_stop_received_ns, self.last_manual_stamp_ns)):
                                self.pending.put_nowait((kind, data))
                elif kind == 'waypoint':
                    values, valid_until, stamp_ns = data
                    try:
                        self.send(3, struct.pack('>hhhI', *values, 0), retries=2, timeout_s=0.02,
                                  valid_until=valid_until, waypoint_stamp_ns=stamp_ns)
                    except TimeoutError:
                        if not self.session_current():
                            raise
                        self.get_logger().warning(
                            'FPGA waypoint velocity ACK missing after retry; current-session telemetry is fresh',
                            throttle_duration_sec=5.0)
                elif kind in ('manual', 'vision'):
                    values, valid_until, generation = data
                    if time.monotonic() < valid_until:
                        self.ensure_session()
                        # The user may have released the control during phase training.
                        if not self.stop_request.is_set() and time.monotonic() < valid_until and (kind == 'manual' or (
                                time.monotonic() - self.last_manual_at >= 1.0 and self.session_current())):
                            try:
                                sent = self.send(3, struct.pack('>hhhI', *values, 0),
                                                 timeout_s=0.02, valid_until=valid_until,
                                                 stream_generation=generation)
                            except (TimeoutError, InterruptedError):
                                if self.stop_request.is_set() or self.active or not self.session_current():
                                    raise
                                # A lost ACK does not retract a possibly executed
                                # velocity packet or extend its original lease.
                                state = f'{kind}_unconfirmed'
                                if self.last_velocity_report != state:
                                    self.report(state, 'velocity ACK missing; original input deadline retained')
                                self.velocity_until = valid_until
                                self.last_velocity_report = state
                            else:
                                if sent:
                                    self.active = False
                                    if self.last_velocity_report != kind:
                                        self.report(kind, 'velocity acknowledged; motion requires fresh FPGA feedback')
                                    self.velocity_until = valid_until
                                    self.last_velocity_report = kind
                    # The task owner has already yielded even if the motion lease
                    # expired while waiting for a previous transaction.
                    if (kind == 'manual' and generation == self.stream_generation and
                            time.monotonic() >= valid_until and not self.stop_request.is_set()):
                        self.stop()
                now = time.monotonic()
                if self.velocity_until and now >= self.velocity_until:
                    self.stop()
                elif self.active:
                    with self.lock:
                        status, at = self.device_status, self.device_at
                    if not status or now - at > 0.5:
                        raise NavigationTelemetryStale('FPGA navigation telemetry became stale')
                    if int(status['task_id']) == self.active_task_id:
                        self.task_confirmed = True
                        if int(status['status']) in (4, 7):
                            state = 'arrived' if int(status['status']) == 4 else 'cancelled'
                            self.active = False
                            self.report(state, 'reported by FPGA')
                    if not self.task_confirmed and now - self.task_submitted_at > 2.0:
                        raise TimeoutError('FPGA did not confirm the submitted task')
                # A stopped task still owns its established communication
                # session. Type 1 only refreshes the link; an acknowledged stop
                # has sent zero velocity and an autonomous-disable request.
                if self.socket and not self.velocity_until and not self.stop_request.is_set() and now >= next_probe:
                    try:
                        self.send(1, bytes(10), retries=1, timeout_s=0.02)
                    except TimeoutError:
                        if not self.session_current():
                            raise
                        # Fresh telemetry preserves the session, not an ACK or
                        # proof that this heartbeat refreshed the FPGA lease.
                        self.get_logger().warning('FPGA heartbeat ACK missing; retaining current session with fresh telemetry',
                                                  throttle_duration_sec=5.0)
                    next_probe = time.monotonic() + 0.03
            except OSError as error:
                if isinstance(error, InterruptedError) and self.stop_request.is_set():
                    continue
                self.link_recovering = True
                with self.lock:
                    if not self.stop_request.is_set():
                        self.request_stop_locked(False)
                    else:
                        self.replace_pending(None)
                if stopping and not isinstance(error, TimeoutError):
                    self.close_socket()
                self.report('link_recovering', str(error))
            except (ValueError, struct.error) as error:
                self.on_stop(None)
                self.report('error', str(error))
            if stopping:
                self.closing.wait(.25)

    def destroy_node(self):
        self.closing.set()
        self.worker.join(timeout=1.0)
        self.close_session()
        return super().destroy_node()


def main():
    rclpy.init()
    node = RobotCommand()
    executor = CommandExecutor(node.manual_subscription)
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

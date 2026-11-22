#!/usr/bin/env python3
"""ROS coordinator and HTTP API for optional cooperative mapping participants."""

import argparse
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from queue import Empty, Queue
import signal
import threading
import time
from urllib.parse import parse_qs, urlsplit
from uuid import uuid4

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Empty as EmptyMsg, String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage
from legged_bridge.acg720_calibration import CAMERA_FRAME, load_camera_mount

from swarm_cooperation.april_alignment import (AlignmentUnavailable, ZERO_MOUNT_BODY_FROM_OPTICAL,
                              load_camera_model, map_alignment, model_from_info,
                              observe_pair, transform_matrix)
from swarm_cooperation.grid import (choose_frontier, compose_pose, inverse_pose,
                   merge_transformed_grids, world_pose)

IMAGE_RECONNECT_INTERVAL = 5.
ALIGNMENT_TF_WAIT_SECONDS = 5.
ROBOTS = {
    'car': {'name': '小车', 'map': '/robot/observation_map', 'source': '/map', 'frame': 'map',
             'pose_frame': 'base_link', 'camera_frame': 'camera', 'image': '/camera/image_raw',
            'camera_info': '/camera/camera_info'},
    'acg720': {'name': 'ACG720 六足', 'map': '/acg720/map', 'source': '/acg720/map',
               'frame': 'acg720_map', 'pose_frame': 'acg720_lidar', 'camera_frame': CAMERA_FRAME,
               'image': '/acg720/camera/alignment_image/compressed',
               'camera_info': '/acg720/camera/camera_info'},
}


def participant_ids(value):
    names = tuple(n.strip() for n in value.split(',')) if isinstance(value, str) else tuple(value)
    if not names or len(set(names)) != len(names) or any(n not in ROBOTS for n in names):
        raise ValueError('participants 必须为 car、acg720 的非空不重复列表')
    return names


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)


def read_grid(msg):
    info = msg.info
    p, q = info.origin.position, info.origin.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if (not info.width or not info.height or not 0 < info.resolution < 10 or
            info.width*info.height > 4_000_000 or info.width*info.height != len(msg.data) or
            not all(math.isfinite(v) for v in values) or
            not 0.5 <= math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) <= 1.5 or
            abs(q.x) > 0.001 or abs(q.y) > 0.001):
        return None
    cells = np.asarray(msg.data)
    if cells.min() < -1 or cells.max() > 100:
        return None
    return {'width': info.width, 'height': info.height, 'resolution': info.resolution,
            'origin': (info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation)),
            'frame': msg.header.frame_id.lstrip('/'), 'data': msg.data}


def wire_grid(grid, mode):
    cells = (np.asarray(grid['data'], dtype=np.int8) + 1).tobytes()
    return {'width': grid['width'], 'height': grid['height'], 'resolution': grid['resolution'],
            'origin': grid['origin'], 'frame': grid.get('frame', 'map'), 'mode': mode,
            'cells': base64.b64encode(cells).decode('ascii')}


class Coordinator(Node):
    def __init__(self, args):
        super().__init__('swarm_coordinator')
        self.args = args
        self.participants = participant_ids(getattr(args, 'participants', 'car,acg720'))
        self.anchor_requested = getattr(args, 'anchor', 'auto')
        if self.anchor_requested != 'auto' and self.anchor_requested not in self.participants:
            raise ValueError('anchor 必须为 auto 或已选择的参与者')
        self.anchor = None
        self.anchor_locked = False
        self.transforms = {}
        self.alignment_states = {}
        self.alignment_qualities = {}
        self.alignment_pair = None
        self.alignment_attempt_at = {}
        self.camera_frames = {}
        self.robot_pose = {}
        self.robot_pose_age = {}
        self.robot_pose_stamps = {}
        self.robot_seen_at = {}
        self.lock = threading.RLock()
        self.commands = Queue()
        self.maps = {}
        self.map_at = {}
        self.map_stamps = {}
        self.map_versions = {}
        self.encoded_maps = {}
        self.map_instance = uuid4().hex
        self.map_revision = 0
        self.merged_revision = -1
        self.merged = None
        self.peer_map_key = ()
        self.alignment_status = '等待两台相机画面和相机内参'
        self.alignment_reset_at = 0.
        self.alignment_reset_stamp = 0
        self.last_alignment_reset_reason = ''
        self.images = {name: deque(maxlen=30) for name in self.participants}
        self.image_at = {}
        self.image_topics = {name: ROBOTS[name]['image'] for name in self.participants}
        self.image_subscriptions = {}
        self.image_subscribed_at = {}
        self.map_topics = {name: ROBOTS[name]['map'] for name in self.participants}
        if 'car' in self.map_topics:
            self.map_topics['car'] = getattr(args, 'car_map_topic', self.map_topics['car'])
        # Fusion restarts do not create a new SLAM coordinate frame.
        self.map_source_topics = {name: ROBOTS[name]['source'] for name in self.participants}
        self.map_publishers = {}
        self.camera_models = {}
        self.car_camera_info_file = getattr(args, 'car_camera_info_file', '')
        if self.car_camera_info_file and 'car' in self.participants:
            self.camera_models['car'] = load_camera_model(self.car_camera_info_file)
        self.acg_camera_info_file = getattr(args, 'acg_camera_info_file', '')
        if self.acg_camera_info_file and 'acg720' in self.participants:
            self.camera_models['acg720'] = load_camera_model(self.acg_camera_info_file)
        mount_file = getattr(args, 'acg_camera_mount_file', '')
        self.acg_heading_offset = (load_camera_mount(mount_file)['heading_offset_rad']
                                   if mount_file and 'acg720' in self.participants else None)
        self.tag_size = getattr(args, 'tag_size', 0.025)
        self.tag_spacing = getattr(args, 'tag_spacing', 0.3)
        self.assume_zero_camera_offset = getattr(args, 'assume_zero_camera_offset', False)
        self.alignment_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='apriltag')
        self.alignment_future = None
        self.alignment_context = None
        self.car = None
        self.car_at = 0.
        self.exploring = False
        self.reason = '等待地图、标定与导航组件'
        self.reservations = {}
        self.recent = []
        self.goal_sequence = 0
        self.car_cancel = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        # Each robot keeps its local TF channel; frame names remain globally distinct.
        for name in self.participants:
            if name == 'car':
                continue
            for static, suffix in ((False, 'tf'), (True, 'tf_static')):
                tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=(DurabilityPolicy.TRANSIENT_LOCAL if static else
                                                DurabilityPolicy.VOLATILE))
                self.create_subscription(TFMessage, f'/{name}/{suffix}',
                                         partial(self.on_robot_tf, static=static), tf_qos)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, topic in self.map_topics.items():
            self.create_subscription(OccupancyGrid, topic, partial(self.on_map, name), qos)
        if 'car' in self.participants:
            self.create_subscription(String, '/robot/mission/status', self.on_car, 10)
        if 'acg720' in self.participants:
            self.create_subscription(PoseWithCovarianceStamped, '/acg720/pose',
                                     partial(self.on_robot_pose, 'acg720'), 10)
            self.create_subscription(Odometry, '/acg720/odom',
                                     partial(self.on_robot_odometry, 'acg720'), qos_profile_sensor_data)
            self.create_subscription(String, '/acg720/status',
                                     partial(self.on_robot_status, 'acg720'), 10)
        for name in self.image_topics:
            self.subscribe_image(name)
        for name in self.participants:
            if name not in self.camera_models:
                self.create_subscription(CameraInfo, ROBOTS[name]['camera_info'],
                                         partial(self.on_camera_info, name), qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, '/swarm/car/goal', 10)
        self.stop_pub = self.create_publisher(EmptyMsg, '/robot/mission/stop', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/swarm/map', qos)
        self.peer_map_pub = self.create_publisher(OccupancyGrid, '/swarm/car/peer_map', qos)
        alignment_file = getattr(args, 'alignment_state_file', '')
        if alignment_file:
            with open(alignment_file, encoding='utf-8') as stream:
                self.restore_locked_alignment(json.load(stream))
        self.create_timer(1.0, self.tick)

    def restore_locked_alignment(self, state):
        anchor = state.get('anchor')
        if (state.get('anchor_locked') is not True or anchor not in self.participants or
                set(state.get('participants', ())) != set(self.participants) or
                state.get('frame') != ROBOTS[anchor]['frame'] or
                (self.anchor_requested != 'auto' and self.anchor_requested != anchor)):
            raise ValueError('保存的锁定对齐与当前参与者或共同参考坐标不符')
        transforms, statuses, qualities = {}, {}, {}
        for name in self.participants:
            row = state.get('alignments', {}).get(name, {})
            pose = row.get('transform')
            if (state.get('robots', {}).get(name, {}).get('map_frame') != ROBOTS[name]['frame'] or
                    not isinstance(pose, (list, tuple)) or len(pose) != 3 or
                    not all(type(value) in (int, float) and math.isfinite(value) for value in pose)):
                raise ValueError('保存的地图坐标或对齐变换无效：' + name)
            transforms[name] = tuple(pose)
            statuses[name], qualities[name] = row.get('status', '已恢复锁定对齐'), row.get('details')
        if any(abs(value) > 1e-12 for value in transforms[anchor]):
            raise ValueError('共同参考地图的对齐变换必须为零')
        self.anchor, self.anchor_locked = anchor, True
        self.transforms, self.alignment_states, self.alignment_qualities = transforms, statuses, qualities
        self.alignment_status = state.get('alignment_quality', '已恢复锁定对齐')
        self.pause_alignment_images()

    def destroy_node(self):
        self.alignment_worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def on_robot_tf(self, message, static=False):
        setter = self.tf.set_transform_static if static else self.tf.set_transform
        for transform in message.transforms:
            setter(transform, 'swarm_participant_tf')

    def subscribe_image(self, name):
        previous = self.image_subscriptions.get(name)
        if previous is not None:
            self.destroy_subscription(previous)
        compressed = name == 'acg720'
        image_type = CompressedImage if compressed else Image
        self.image_subscriptions[name] = self.create_subscription(
            image_type, self.image_topics[name], partial(self.on_image, name), qos_profile_sensor_data)
        self.image_subscribed_at[name] = time.monotonic()

    def pause_alignment_images(self):
        for subscription in self.image_subscriptions.values():
            self.destroy_subscription(subscription)
        self.image_subscriptions.clear()

    def refresh_sources(self, now):
        for name, topic in self.map_source_topics.items():
            publishers = frozenset(bytes(info.endpoint_gid) for info in self.get_publishers_info_by_topic(topic))
            if not publishers:
                continue
            previous = self.map_publishers.get(name)
            if previous is not None and previous != publishers:
                before = ','.join(sorted(gid.hex() for gid in previous))
                after = ','.join(sorted(gid.hex() for gid in publishers))
                self.invalidate_map(name, f'{name} 地图源发布者变化：{before} -> {after}')
            self.map_publishers[name] = publishers
        if any(name not in self.transforms for name in self.participants):
            for name in self.image_topics:
                last_activity = max(self.image_at.get(name, 0.), self.image_subscribed_at.get(name, 0.))
                if name not in self.image_subscriptions or now-last_activity >= IMAGE_RECONNECT_INTERVAL:
                    self.subscribe_image(name)

    def on_image(self, name, msg):
        with self.lock:
            now = time.monotonic()
            if now-self.image_at.get(name, 0.) >= 0.09:
                self.images[name].append((msg, now))
                self.image_at[name] = now
                self.robot_seen_at[name] = now
                self.camera_frames[name] = msg.header.frame_id.lstrip('/')

    def on_camera_info(self, name, msg):
        fixed_file = ((name == 'car' and self.car_camera_info_file) or
                      (name == 'acg720' and self.acg_camera_info_file))
        try:
            model = model_from_info(msg)
        except (AlignmentUnavailable, TypeError, ValueError):
            return
        with self.lock:
            if msg.header.frame_id:
                self.camera_frames[name] = msg.header.frame_id.lstrip('/')
            if not fixed_file:
                self.camera_models[name] = model

    def reset_alignment(self, reason='对齐已清除'):
        frame = ROBOTS[self.anchor]['frame'] if self.anchor is not None else 'map'
        if self.alignment_future is not None:
            self.alignment_future.cancel()
            self.alignment_future = self.alignment_context = None
        self.transforms.clear()
        self.anchor = None
        self.anchor_locked = False
        self.alignment_states.clear()
        self.alignment_qualities.clear()
        self.alignment_pair = None
        self.alignment_status = '等待两台相机重新观察同一 AprilTag'
        self.alignment_reset_at = time.monotonic()
        self.alignment_reset_stamp = self.get_clock().now().nanoseconds
        self.last_alignment_reset_reason = reason
        self.merged = None
        self.encoded_maps.pop('common', None)
        self.map_revision += 1
        self.exploring = False
        self.cancel_owned()
        self.recent.clear()
        empty = OccupancyGrid()
        empty.header.frame_id = frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(empty)
        self.publish_peer_map()

    def invalidate_map(self, name, reason):
        # The same frame name may be reused by a restarted map publisher.
        self.maps.pop(name, None)
        self.encoded_maps.pop(name, None)
        self.map_at.pop(name, None)
        self.map_stamps.pop(name, None)
        self.map_versions.pop(name, None)
        self.reset_alignment(reason)

    def ensure_anchor(self):
        if self.anchor_locked or self.alignment_future is not None:
            return
        candidates = self.participants if self.anchor_requested == 'auto' else (self.anchor_requested,)
        available = [n for n in candidates if n in self.maps]
        # Prefer a calibrated camera until the first cross-robot observation locks the frame.
        ready = [n for n in available if n in self.camera_models and self.camera_transform_available(n)]
        preferred = ready or available
        anchor = self.anchor if self.anchor in preferred else next(iter(preferred), None)
        if anchor != self.anchor:
            self.anchor = anchor
            self.transforms.clear()
            if anchor is not None:
                self.transforms[anchor] = (0., 0., 0.)
            self.map_revision += 1

    def camera_transform_available(self, name):
        robot = ROBOTS[name]
        frame = self.camera_frames.get(name, robot['camera_frame'])
        return bool(self.tf.can_transform(robot['frame'], frame, Time()))

    def camera_map_transform(self, name, image):
        at = Time.from_msg(image.header.stamp)
        try:
            tf = self.tf.lookup_transform(ROBOTS[name]['frame'], image.header.frame_id, at)
            return transform_matrix(tf.transform), False
        except TransformException as exc:
            if name == 'car' and self.assume_zero_camera_offset:
                tf = self.tf.lookup_transform('map', 'base_link', at)
                return transform_matrix(tf.transform) @ ZERO_MOUNT_BODY_FROM_OPTICAL, True
            raise AlignmentUnavailable(f"等待{ROBOTS[name]['name']}在图像时间的地图到相机变换") from exc

    def alignment_images(self, name, now):
        ros_now = self.get_clock().now().nanoseconds
        result = []
        for image, received in self.images[name]:
            stamp = Time.from_msg(image.header.stamp).nanoseconds
            if (received > self.alignment_reset_at and now-received <= 2. and
                    stamp > self.alignment_reset_stamp and -0.2 <= (ros_now-stamp)*1e-9 <= 2.):
                result.append((image, stamp))
        return result

    def update_alignment(self, now):
        self.ensure_anchor()
        if self.exploring:
            return
        if self.alignment_future is not None:
            if not self.alignment_future.done():
                return
            future, context = self.alignment_future, self.alignment_context
            reference, target = self.alignment_pair
            try:
                result = future.result()
                first, second, image_delta = context
                reference_camera, first_assumed = self.camera_map_transform(reference, first)
                target_camera, second_assumed = self.camera_map_transform(target, second)
                relative = map_alignment(reference_camera, target_camera, result)
                self.transforms[target] = compose_pose(self.transforms[reference], relative)
                self.anchor_locked = True
                assumed_zero = first_assumed or second_assumed
                details = {'reference': reference, 'target': target, 'tag_ids': result['tags'],
                           'reference_reprojection_px': result['car_reprojection_px'],
                           'target_reprojection_px': result['hex_reprojection_px'],
                           'map_tilt_deg': result['map_tilt_deg'],
                           'image_stamp_difference_s': image_delta,
                           'assumed_zero_car_mount': assumed_zero,
                           'intrinsics_preliminary': {n: self.camera_models[n].get('preliminary', False)
                                                       for n in (reference, target)}}
                self.alignment_qualities[target] = details
                qualifiers = []
                if assumed_zero:
                    qualifiers.append('假定小车相机零安装偏移')
                if any(details['intrinsics_preliminary'].values()):
                    qualifiers.append('内参待验证')
                status = 'AprilTag 自动对齐' + ('，' + '，'.join(qualifiers) if qualifiers else '')
                self.alignment_states[target] = self.alignment_status = status
                if all(n in self.transforms for n in self.participants):
                    self.pause_alignment_images()
                self.map_revision += 1
            except Exception as exc:
                awaiting_tf = (isinstance(exc, TransformException) or
                               isinstance(exc.__cause__, TransformException))
                # Retain the observed images while their delayed map TF catches up.
                if (awaiting_tf and
                        now - self.alignment_attempt_at[self.alignment_pair] < ALIGNMENT_TF_WAIT_SECONDS):
                    self.alignment_states[target] = self.alignment_status = str(exc)
                    return
                self.alignment_states[target] = self.alignment_status = str(exc)
            self.alignment_future = self.alignment_context = None
            return
        eligible = {}
        for name in self.participants:
            label = ROBOTS[name]['name']
            pending = []
            age = self.map_age(name, now)
            # Static grid age is independent of the camera's current map pose.
            if age is None:
                pending.append('等待有效地图')
            if (not self.images[name] or now-self.image_at[name] > 2. or
                    self.image_at[name] <= self.alignment_reset_at):
                pending.append('等待相机画面')
            if name not in self.camera_models:
                pending.append('待标定：缺相机内参')
            if pending:
                if name not in self.transforms:
                    self.alignment_states[name] = f'{label}：' + '；'.join(pending)
                continue
            eligible[name] = self.alignment_images(name, now)
        candidates = [(reference, target) for target in eligible if target not in self.transforms
                      for reference in eligible if reference in self.transforms]
        candidates.sort(key=lambda pair: self.alignment_attempt_at.get(pair, 0.))
        for reference, target in candidates:
            pairs = [(first, second) for first in eligible[reference] for second in eligible[target]
                     if abs(first[1]-second[1]) <= 400_000_000]
            if not pairs:
                self.alignment_states[target] = '等待时间接近且新鲜的两台相机画面'
                continue
            first, second = max(pairs, key=lambda pair: (min(pair[0][1], pair[1][1]),
                                                        -abs(pair[0][1]-pair[1][1])))
            self.alignment_pair = reference, target
            self.alignment_attempt_at[self.alignment_pair] = now
            self.alignment_states[target] = self.alignment_status = '正在识别共同 AprilTag'
            self.alignment_context = (first[0], second[0], abs(first[1]-second[1])*1e-9)
            self.alignment_future = self.alignment_worker.submit(
                observe_pair, first[0], second[0], self.camera_models[reference],
                self.camera_models[target], self.tag_size, self.tag_spacing)
            return
        pending = [self.alignment_states.get(n, f"{ROBOTS[n]['name']}等待共同 AprilTag")
                   for n in self.participants if n not in self.transforms]
        self.alignment_status = '；'.join(pending) if pending else '参考坐标已建立'

    def on_map(self, name, msg):
        grid = read_grid(msg)
        expected_frame = ROBOTS[name]['frame']
        if (not grid and msg.header.frame_id.lstrip('/') == expected_frame and
                not msg.info.width and not msg.info.height and not msg.data):
            with self.lock:
                if name in self.maps or name in self.transforms:
                    self.invalidate_map(name, f'{name} 地图源已清图')
            return
        if grid and grid['frame'] == expected_frame:
            with self.lock:
                self.map_at[name] = time.monotonic()
                self.robot_seen_at[name] = self.map_at[name]
                self.map_stamps[name] = Time.from_msg(msg.header.stamp).nanoseconds
                if self.maps.get(name) != grid:
                    grid['data'] = grid['data'][:]
                    self.maps[name] = grid
                    self.map_revision += 1
                    self.map_versions[name] = f'{self.map_instance}:{self.map_revision}'

    def map_age(self, name, now):
        stamp = self.map_stamps.get(name, 0)
        if name not in self.map_at or stamp <= 0:
            return None
        source_age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if source_age < -0.2:
            return None
        return max(0., source_age, now-self.map_at[name])

    def on_car(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                robot = value.get('robot')
                if robot is not None and (not isinstance(robot, dict) or
                        not all(isinstance(robot.get(k), (float, int)) and math.isfinite(robot[k])
                                for k in ('x', 'y', 'yaw'))):
                    return
                goal = value.get('goal')
                if goal is not None and (not isinstance(goal, (list, tuple)) or len(goal) != 2 or
                        not all(isinstance(v, (float, int)) and math.isfinite(v) for v in goal)):
                    return
                with self.lock:
                    self.car, self.car_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_robot_pose(self, name, msg):
        stamp = msg.header.stamp
        if (msg.header.frame_id.lstrip('/') != ROBOTS[name]['frame'] or
                stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000):
            return
        source = stamp.sec * 1000000000 + stamp.nanosec
        if source > 0:
            with self.lock:
                self.robot_pose_stamps[name] = source

    def on_robot_odometry(self, name, msg):
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        norm = math.sqrt(sum(v*v for v in values[3:]))
        if (msg.header.frame_id.lstrip('/') != f'{name}_odom' or
                msg.child_frame_id.lstrip('/') != ROBOTS[name]['pose_frame'] or
                not all(math.isfinite(v) for v in values) or not .5 <= norm <= 1.5):
            return
        with self.lock:
            self.robot_seen_at[name] = time.monotonic()

    def on_robot_status(self, name, msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(value, dict):
            with self.lock:
                self.robot_seen_at[name] = time.monotonic()

    def update_participant_poses(self):
        for name in self.participants:
            self.robot_pose.pop(name, None)
            self.robot_pose_age.pop(name, None)
            if name == 'car':
                robot = self.car.get('robot') if self.car else None
                if robot:
                    self.robot_pose[name] = (robot['x'], robot['y'], robot['yaw'])
            else:
                # TF can carry future padding or a raw scan stamp after matching was rejected.
                source = self.robot_pose_stamps.get(name)
                if source is None:
                    continue
                accepted_age = (self.get_clock().now().nanoseconds - source) * 1e-9
                self.robot_pose_age[name] = accepted_age
                if not -.2 <= accepted_age <= 1.0:
                    continue
                try:
                    tf = self.tf.lookup_transform(ROBOTS[name]['frame'], ROBOTS[name]['pose_frame'], Time())
                    age = (self.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds) * 1e-9
                    if not -.2 <= age <= 1.0:
                        self.robot_pose_age[name] = age
                        continue
                    self.robot_pose_age[name] = max(accepted_age, age)
                    t = tf.transform.translation
                    pose = t.x, t.y, yaw(tf.transform.rotation)
                    if all(math.isfinite(v) for v in pose):
                        self.robot_pose[name] = pose
                except TransformException:
                    pass

    def readiness(self, now):
        self.ensure_anchor()
        available = [n for n in self.participants if n in self.maps and n in self.transforms]
        if ((len(available) >= 2 or len(self.participants) == 1) and 'car' in available):
            return []
        if 'car' not in self.participants:
            return ['所选机器人使用手动建图，未配置自动探索接口']
        return ['等待 AprilTag 自动对齐：' + self.alignment_status]

    def publish_map(self):
        self.ensure_anchor()
        if self.map_revision == self.merged_revision:
            return
        previous, self.merged = self.merged, None
        self.merged_revision = self.map_revision
        contributors = tuple(n for n in self.participants if n in self.maps and n in self.transforms)
        if len(contributors) >= 2:
            try:
                self.merged = merge_transformed_grids([(self.maps[n], self.transforms[n]) for n in contributors])
                self.merged['frame'] = ROBOTS[self.anchor]['frame']
                self.merged['contributors'] = contributors
            except ValueError as exc:
                self.reason = str(exc)
        if self.merged == previous:
            self.merged = previous
            return
        msg = OccupancyGrid()
        msg.header.frame_id = (self.merged or previous)['frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.merged is not None:
            g = self.merged
            msg.info.resolution, msg.info.width, msg.info.height = g['resolution'], g['width'], g['height']
            msg.info.origin.position.x, msg.info.origin.position.y = g['origin'][:2]
            msg.info.origin.orientation.w = 1.
            msg.data = g['data']
        self.map_pub.publish(msg)

    def publish_peer_map(self):
        if 'car' not in self.participants:
            return
        # The common anchor may be ACG720. Express the peer grid in the car's
        # local map frame and retain its source stamp and cell values.
        peer = next((n for n in self.participants if n != 'car' and
                     n in self.maps and n in self.transforms and 'car' in self.transforms), None)
        peer_to_car = (compose_pose(inverse_pose(self.transforms['car']), self.transforms[peer])
                       if peer is not None else None)
        key = ((self.map_versions[peer], peer_to_car, self.map_stamps[peer])
               if peer is not None else None)
        if key == self.peer_map_key:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        if peer_to_car is not None:
            grid = self.maps[peer]
            x, y, heading = world_pose(grid, peer_to_car)
            msg.header.stamp = Time(nanoseconds=self.map_stamps[peer]).to_msg()
            msg.info.width, msg.info.height, msg.info.resolution = (
                grid['width'], grid['height'], grid['resolution'])
            msg.info.origin.position.x, msg.info.origin.position.y = x, y
            msg.info.origin.orientation.z = math.sin(heading/2.)
            msg.info.origin.orientation.w = math.cos(heading/2.)
            msg.data = grid['data']
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.peer_map_pub.publish(msg)
        self.peer_map_key = key

    def cancel_owned(self):
        reservation = self.reservations.get('car')
        if reservation:
            self.car_cancel = {'local': reservation['local'], 'deadline': time.monotonic()+5.,
                               'sent': False, 'reported': False, 'requested_at': time.monotonic()}
            self.stop_pub.publish(EmptyMsg())
            self.check_car_cancel(time.monotonic())
        self.reservations.clear()

    def check_car_cancel(self, now):
        pending = self.car_cancel
        if not pending:
            return
        if now > pending['deadline'] and not pending['reported']:
            pending['reported'] = True
            self.reason = '小车目标取消未确认，请核对小车任务状态'
        if not self.car or now-self.car_at > 2.:
            return
        if (self.car_at > pending['requested_at'] and
                self.car.get('mode') == 'idle' and self.car.get('goal') is None):
            self.car_cancel = None
            return
        car_goal = self.car.get('goal')
        matching = car_goal and math.dist(car_goal[:2], pending['local']) < 0.2
        if self.car.get('mode') == 'goal' and matching and not self.car.get('owner'):
            self.stop_pub.publish(EmptyMsg())
            pending['sent'] = True
        elif ((matching and self.car.get('mode') in ('arrived', 'blocked')) or
              (self.car.get('mode') in ('goal', 'explore') and
               (not matching or self.car.get('owner')))):
            self.car_cancel = None

    def finish(self, name):
        reservation = self.reservations.pop(name, None)
        if reservation:
            self.recent.append((time.monotonic()+30., reservation['common']))

    def dispatch(self, name, local, common):
        if name != 'car':
            raise ValueError('该机器人仅接入手动建图，没有自动探索接口')
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = local
        msg.pose.orientation.w = 1.
        self.goal_sequence += 1
        self.reservations[name] = {'id': self.goal_sequence, 'local': local, 'common': common,
                                   'at': time.monotonic(), 'acknowledged': False}
        self.goal_pub.publish(msg)

    def tick(self):
        with self.lock:
            now = time.monotonic()
            self.refresh_sources(now)
            self.check_car_cancel(now)
            self.update_alignment(now)
            self.publish_map()
            self.publish_peer_map()
            while True:
                try:
                    command = self.commands.get_nowait()
                except Empty:
                    break
                if command == 'stop':
                    self.exploring = False
                    self.cancel_owned()
                    self.reason = '协同探索已停止'
                elif command == 'start':
                    problems = self.readiness(now)
                    if problems:
                        self.reason = '；'.join(problems)
                    else:
                        self.exploring, self.reason = True, '正在分配小车探索目标'
            if not self.exploring:
                return
            problems = self.readiness(now)
            if problems:
                self.exploring = False
                self.cancel_owned()
                self.reason = '协同探索停止：' + '；'.join(problems)
                return
            reservation = self.reservations.get('car')
            if reservation and self.car:
                car_goal = self.car.get('goal')
                matching = car_goal and math.dist(car_goal[:2], reservation['local']) < 0.2
                if self.car.get('mode') == 'goal' and matching:
                    reservation['acknowledged'] = True
                elif self.car_at > reservation['at'] and (reservation['acknowledged'] or
                                                        now-reservation['at'] > 3.0):
                    self.finish('car')
            self.recent = [(until, goal) for until, goal in self.recent if until > now]
            if self.reservations:
                return
            self.reason = '等待可达探索目标或小车目标接口'
            if ('car' not in self.participants or 'car' not in self.maps or 'car' not in self.transforms or
                    self.car_cancel is not None or not self.car or not self.car.get('robot') or
                    not self.goal_pub.get_subscription_count() or
                    self.car.get('mode') not in ('idle', 'arrived', 'blocked') or self.car.get('owner')):
                return
            pose = self.car['robot']
            peer_sources = [(self.maps[n], self.transforms[n]) for n in self.participants
                            if n != 'car' and n in self.maps and n in self.transforms]
            peer = merge_transformed_grids(peer_sources) if peer_sources else None
            target = choose_frontier(self.maps['car'], (pose['x'], pose['y']), self.args.car_radius,
                                     self.args.sensing, [g for _, g in self.recent], self.args.separation,
                                     self.transforms['car'], self.merged,
                                     allow_start_transition=True, peer_grid=peer, soft_separation=True)
            if target:
                self.dispatch('car', *target)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            self.ensure_anchor()
            self.update_participant_poses()
            transforms = self.transforms
            robots, alignments = {}, {}
            mapping_pending = []
            for name in self.participants:
                camera_tf = self.camera_transform_available(name)
                model_ready = name in self.camera_models
                aligned = self.anchor_locked and name in transforms
                if aligned:
                    status = ('共同坐标参考' if name == self.anchor else
                              self.alignment_states.get(name, 'AprilTag 已对齐'))
                elif not model_ready:
                    status = '待标定：缺相机内参'
                elif not camera_tf:
                    status = '待标定：缺相机到地图的实测安装/定位变换'
                elif name == self.anchor:
                    status = '共同坐标参考' if len(transforms) > 1 else '局部参考坐标，等待跨机 AprilTag 对齐'
                else:
                    status = self.alignment_states.get(name, '等待共同 AprilTag')
                pose = self.robot_pose.get(name)
                common = None
                if pose is not None and aligned:
                    x, y, angle = compose_pose(transforms[name], pose)
                    common = {'x': x, 'y': y, 'yaw': angle}
                age = self.map_age(name, now)
                seen = max(self.robot_seen_at.get(name, 0.), self.map_at.get(name, 0.),
                           self.image_at.get(name, 0.), self.car_at if name == 'car' else 0.)
                has_map = name in self.maps
                auto = (name == 'car' and has_map and name in transforms and
                        bool(self.goal_pub.get_subscription_count()))
                robots[name] = {'name': ROBOTS[name]['name'], 'selected': True,
                                'online': bool(seen and now-seen <= 10.),
                                'map_ready': has_map, 'map_frame': ROBOTS[name]['frame'],
                                'map_session': self.map_session(name),
                                'map_version': self.map_versions.get(name),
                                'map_age_s': round(age, 1) if age is not None else None,
                                'pose_local': pose, 'pose_common': common,
                                'pose_valid': pose is not None,
                                'pose_age_s': self.robot_pose_age.get(name),
                                'pose_reference': ROBOTS[name]['pose_frame'],
                                'heading_offset_rad': self.acg_heading_offset if name == 'acg720' else 0.,
                                'aligned': aligned, 'alignment_status': status,
                                'camera_model_ready': model_ready, 'camera_transform_ready': camera_tf,
                                'camera_intrinsics_preliminary': self.camera_models.get(name, {}).get('preliminary', False),
                                'manual_available': True, 'auto_navigation_available': auto}
                alignments[name] = {'transform': transforms.get(name), 'status': status,
                                    'details': self.alignment_qualities.get(name)}
                if not has_map:
                    mapping_pending.append(ROBOTS[name]['name'] + '等待局部地图')
                if not aligned:
                    mapping_pending.append(ROBOTS[name]['name'] + '：' + status)
            contributors = [n for n in self.participants if n in self.maps and robots[n]['aligned']]
            mapping_status = ('aligned' if len(contributors) >= 2 else
                              'local_only' if self.maps else 'waiting_maps')
            return {'car': self.car if now-self.car_at < 2 else None,
                    'participants': self.participants, 'anchor': self.anchor, 'anchor_locked': self.anchor_locked,
                    'frame': ROBOTS[self.anchor]['frame'] if self.anchor is not None else None,
                    'robots': robots, 'alignments': alignments,
                    'mapping_status': mapping_status, 'mapping_readiness': mapping_pending,
                    'contributors': contributors,
                    'camera_image_ages': {n: round(now-self.image_at[n], 1) if n in self.image_at else None
                                          for n in self.participants},
                    'alignment_image_subscriptions': {n: n in self.image_subscriptions
                                                      for n in self.participants},
                    'map_ages': {n: round(age, 1) if age is not None else None
                                 for n in self.map_at for age in (self.map_age(n, now),)},
                    'alignment_quality': self.alignment_status,
                    'last_alignment_reset_reason': self.last_alignment_reset_reason,
                    'exploring': self.exploring, 'reason': self.reason,
                    'readiness': self.readiness(now), 'reservations': self.reservations}

    def map_session(self, name):
        return ','.join(sorted(gid.hex() for gid in self.map_publishers.get(name, ()))) or None

    def map_snapshot(self, view):
        with self.lock:
            if view in ROBOTS:
                if view not in self.participants or view not in self.maps:
                    return None
                grid, mode = self.maps[view], view + '_local'
                source = view
                metadata = dict(contributors=[view], anchor=None, local=True,
                                map_session=self.map_session(view), map_version=self.map_versions[view])
            elif view not in ('common', 'aligned'):
                return None
            elif self.merged is not None:
                grid, mode = self.merged, 'aligned'
                source = 'common'
                metadata = dict(contributors=grid['contributors'], anchor=self.anchor, local=False)
            else:
                local = next((n for n in self.participants if n in self.maps), None)
                if local is None:
                    return None
                grid, mode = self.maps[local], local + '_only'
                source = local
                metadata = dict(contributors=[local], anchor=None, local=True)
            cached = self.encoded_maps.get(source)
            if cached is not None and cached[0] is grid:
                return dict(cached[1], mode=mode, **metadata)
        encoded = wire_grid(grid, mode)
        with self.lock:
            current = self.merged if source == 'common' else self.maps.get(source)
            if current is grid:
                self.encoded_maps[source] = grid, encoded
        return dict(encoded, **metadata)

    def request_alignment_reobservation(self):
        with self.lock:
            if self.exploring:
                return False
            self.reset_alignment('用户请求重新观察 AprilTag')
            return True

    def confirm_car_stopped(self, data):
        with self.lock:
            if (data.get('confirmed') is not True or self.exploring or not self.car or
                    time.monotonic()-self.car_at > 2 or
                    self.car.get('mode') not in ('idle', 'arrived', 'blocked')):
                raise ValueError('需现场确认小车停止，且任务状态新鲜并处于空闲或结束')
            self.car_cancel = None
            self.reason = '操作员已确认小车停止'


class Handler(BaseHTTPRequestHandler):
    coordinator = None

    def send_json(self, status, value):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path)
        if path.path == '/api/state':
            self.send_json(200, self.coordinator.snapshot())
        elif path.path == '/api/map':
            grid = self.coordinator.map_snapshot(parse_qs(path.query).get('view', ['common'])[0])
            self.send_json(200 if grid else 503, grid or {'error': '地图不可用'})
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 16384:
                raise ValueError('请求体无效')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('请求体必须是对象')
            if path == '/api/explore/confirm-stopped':
                self.coordinator.confirm_car_stopped(body)
                self.send_json(200, {'accepted': True})
            elif path == '/api/alignment/reobserve':
                if body:
                    raise ValueError('重新识别不接受人工位姿，请求体必须为空对象')
                if not self.coordinator.request_alignment_reobservation():
                    self.send_json(409, {'error': '请先停止协同探索'})
                    return
                self.send_json(200, {'accepted': True})
            elif path in ('/api/explore/start', '/api/explore/stop'):
                self.coordinator.commands.put('start' if path.endswith('start') else 'stop')
                self.send_json(202, {'accepted': True})
            else:
                self.send_error(404)
        except (KeyError, TypeError, ValueError, OSError) as exc:
            self.send_json(400, {'error': str(exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--participants', default='car,acg720',
                        help='本会话参与者，逗号分隔：car,acg720；未在线成员不阻断局部建图')
    parser.add_argument('--anchor', default='auto',
                        help='共同参考坐标的机器人；auto 在首次收到地图时锁定一个在线成员')
    parser.add_argument('--car-camera-info-file', default='',
                        help='小车相机标定 YAML；未指定时订阅 /camera/camera_info')
    parser.add_argument('--acg-camera-info-file', default='',
                        help='ACG720 相机实测标定 YAML；未指定时等待 /acg720/camera/camera_info')
    parser.add_argument('--acg-camera-mount-file', default='',
                        help='ACG720 安装标定 YAML，读取地图显示用的机头偏角')
    parser.add_argument('--car-map-topic', default=ROBOTS['car']['map'],
                        help='小车地图输入，只读建图时使用 /map')
    parser.add_argument('--allow-preliminary-intrinsics', action='store_true',
                        help='按机主要求使用初步小车内参进行现场试验，仍保留状态提示')
    parser.add_argument('--tag-size', type=float, default=0.025)
    parser.add_argument('--tag-spacing', type=float, default=0.3)
    parser.add_argument('--assume-zero-camera-offset', action='store_true',
                        help='静止测试时假定小车相机安装偏移和安装角为零')
    parser.add_argument('--alignment-state-file', default='',
                        help='恢复此前已锁定的共同坐标，不恢复实时位置或任务')
    for name in ('car-radius', 'sensing', 'separation',
                 'max-position-variance', 'max-yaw-variance'):
        parser.add_argument('--' + name, type=float, default=0.)
    args = parser.parse_args(remove_ros_args()[1:])
    try:
        args.participants = participant_ids(args.participants)
        if args.anchor != 'auto' and args.anchor not in args.participants:
            raise ValueError('anchor 必须为 auto 或已选择的参与者')
    except ValueError as exc:
        parser.error(str(exc))
    if (not math.isfinite(args.tag_size) or args.tag_size <= 0 or
            not math.isfinite(args.tag_spacing) or args.tag_spacing < 0):
        parser.error('AprilGrid 标签尺寸必须为正，间距比必须为非负数')
    if any(not math.isfinite(getattr(args, name)) or getattr(args, name) < 0 for name in
           ('car_radius', 'sensing', 'separation',
            'max_position_variance', 'max_yaw_variance')):
        parser.error('校准参数必须是有限的非负数')
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Coordinator(args)
    executor = SingleThreadedExecutor(context=node.context)
    Handler.coordinator = node
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'协同接口 http://{args.bind}:{args.port}', flush=True)
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.reset_alignment()
        node.cancel_owned()
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

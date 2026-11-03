#!/usr/bin/env python3
"""ROS-only synthetic replay. Run on Jetson in an isolated domain, never on the robot domain."""
from array import array
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


SENSOR_OFFSET_X_M = 0.13


SEGMENTS = [((-5, -5), (5, -5)), ((5, -5), (5, 5)), ((5, 5), (-5, 5)),
            ((-5, 5), (-5, -5)), ((-4.0, 0.5), (-3.2, 0.5)),
            ((1.8, 3.5), (3.4, 3.5)), ((3.6, -4.0), (3.6, -1.8))]


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


def range_to_wall(x, y, angle):
    dx, dy = math.cos(angle), math.sin(angle)
    nearest = float('nan')
    for (ax, ay), (bx, by) in SEGMENTS:
        sx, sy = bx - ax, by - ay
        den = cross(dx, dy, sx, sy)
        if abs(den) < 1e-9:
            continue
        distance = cross(ax - x, ay - y, sx, sy) / den
        along = cross(ax - x, ay - y, dx, dy) / den
        if distance > 0.01 and 0 <= along <= 1 and (math.isnan(nearest) or distance < nearest):
            nearest = distance
    return nearest


def square_path(progress):
    # 20 m square followed by the first edge again to make revisit overlap visible.
    distance = min(progress, 1.10) * 20.0
    corners = [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5),
               (-2.5, -2.5), (2.5, -2.5)]
    edge = min(int(distance / 5), 4)
    fraction = (distance - 5 * edge) / 5
    a, b = corners[edge], corners[edge + 1]
    return a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])


def transform(parent, child, stamp, x, y):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.transform.translation.x, msg.transform.translation.y = x, y
    msg.transform.rotation.w = 1.0
    return msg


def main():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 190 <= domain <= 220:
        raise RuntimeError('Set an isolated ROS_DOMAIN_ID between 190 and 220')
    config = Path(sys.argv[1]).resolve()
    adapter = Path(sys.argv[2]).resolve()
    log = Path(sys.argv[3]).resolve()
    port = int(sys.argv[4]) if len(sys.argv) > 4 else 18766
    loop_closing = sys.argv[5] if len(sys.argv) > 5 else 'true'
    duration = float(sys.argv[6]) if len(sys.argv) > 6 else 47.0
    if loop_closing not in ('true', 'false'):
        raise ValueError('loop_closing must be true or false')
    processes = []
    with log.open('w', encoding='utf-8') as output:
        try:
            slam = '/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node'
            processes.append(subprocess.Popen([slam, '--ros-args', '--params-file', str(config),
                                               '-p', 'debug_logging:=true',
                                               '-p', f'do_loop_closing:={loop_closing}'],
                                              stdout=output, stderr=output))
            processes.append(subprocess.Popen([sys.executable, str(adapter), '--ros-args',
                                               '--params-file', str(config),
                                               '-p', 'http_bind:=127.0.0.1',
                                               '-p', f'http_port:={port}'], stdout=output, stderr=output))
            rclpy.init()
            node = Node('synthetic_slam_replay')
            nav_pub = node.create_publisher(String, '/fpga/round5/navigation', qos_profile_sensor_data)
            scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            dynamic = TransformBroadcaster(node)
            static = StaticTransformBroadcaster(node)
            static.sendTransform(transform('base_link', 'n10_sensor', node.get_clock().now().to_msg(), SENSOR_OFFSET_X_M, 0.0))
            start = time.monotonic()
            tick = 0
            progress_samples = []
            next_sample = 2.0
            while time.monotonic() - start < duration:
                if any(proc.poll() is not None for proc in processes):
                    raise RuntimeError('SLAM or adapter process exited early; inspect log')
                elapsed = time.monotonic() - start
                progress = max(0.0, (elapsed - 3.0) / 35.0)
                x, y = square_path(progress)
                # Simulated odometry drift; scans are generated from the true pose.
                odom_x = x + min(progress, 1.0) * 0.3
                stamp = node.get_clock().now().to_msg()
                dynamic.sendTransform(transform('odom', 'base_link', stamp, odom_x, y))
                nav = String()
                nav.data = json.dumps({'protocol': 'nav_grid_v1', 'boot_id': 1, 'odom_domain': 1,
                                       'now_us': (tick * 200000) & 0xffffffff, 'pose_valid': True})
                nav_pub.publish(nav)
                msg = LaserScan()
                msg.header.stamp, msg.header.frame_id = stamp, 'n10_sensor'
                msg.angle_min, msg.angle_increment = 0.0, math.pi / 18000
                msg.angle_max = msg.angle_increment * 35999
                msg.range_min, msg.range_max = 0.01, 65.534
                msg.scan_time, msg.time_increment = 0.2, 0.0
                ranges = array('f', [float('nan')]) * 36000
                for index in range(720):
                    ranges[index * 50] = range_to_wall(x + SENSOR_OFFSET_X_M, y, index * math.pi / 360)
                msg.ranges = ranges
                scan_pub.publish(msg)
                tick += 1
                rclpy.spin_once(node, timeout_sec=0.01)
                if elapsed >= next_sample:
                    next_sample = elapsed + 2.0
                    try:
                        with urlopen(f'http://127.0.0.1:{port}/global-map-meta.json', timeout=0.4) as response:
                            sample = json.load(response)
                        if sample.get('available') and sample.get('status') == 'building':
                            counts = sample.get('counts', {})
                            progress_samples.append({'revision': sample.get('revision'),
                                                     'known': counts.get('free', 0) + counts.get('occupied', 0)})
                    except (OSError, ValueError):
                        pass
                time.sleep(0.19)
            with urlopen(f'http://127.0.0.1:{port}/global-map.json', timeout=3) as response:
                result = json.load(response)
            with urlopen(f'http://127.0.0.1:{port}/global-map-meta.json', timeout=3) as response:
                metadata = json.load(response)
            print(json.dumps({'ticks': tick, 'available': result.get('available'),
                              'status': result.get('status'), 'counts': result.get('counts'),
                              'revision': result.get('revision'),
                              'progress_samples': progress_samples,
                              'loop_closure_enabled': result.get('loop_closure_enabled'),
                              'map_from_odom': result.get('map_from_odom'),
                              'map_from_odom_age_ms': result.get('map_from_odom_age_ms'),
                              'pose': result.get('pose')}, ensure_ascii=False))
            if not result.get('available') or result.get('counts', {}).get('occupied', 0) < 10:
                raise RuntimeError('SLAM did not produce a populated global map')
            if result.get('loop_closure_enabled') != (loop_closing == 'true'):
                raise RuntimeError('SLAM parameter status does not match the actual loop setting')
            if not isinstance(result.get('map_from_odom'), dict):
                raise RuntimeError('fresh map-to-odom transform was not exposed')
            if ('cells_rle' in metadata or metadata.get('domain') != result.get('domain') or
                    not isinstance(metadata.get('map_from_odom'), dict)):
                raise RuntimeError('lightweight metadata is missing a fresh transform or contains the grid')
            if duration >= 12 and (len(progress_samples) < 2 or
                                   progress_samples[-1]['known'] <= progress_samples[0]['known']):
                raise RuntimeError('global map did not grow during the synthetic drive')
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
            for proc in reversed(processes):
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
            for proc in reversed(processes):
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


if __name__ == '__main__':
    main()

"""Publish measured ACG720 camera calibration; never invent missing geometry."""

import math
from pathlib import Path

import yaml


CAMERA_FRAME = 'acg720_camera_optical'
LIDAR_FRAME = 'acg720_lidar'
PREVIEW_TOPIC = '/acg720/camera/image/compressed'


def _mapping_file(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a YAML mapping')
    return data


def _numbers(values, length, name):
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f'{name}: expected {length} numbers')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError(f'{name}: values must be finite numbers')
    return [float(value) for value in values]


def load_camera_info(path):
    """Read the standard ROS CameraInfo YAML produced by camera calibration."""
    data = _mapping_file(path)
    if data.get('image_width') != 800 or data.get('image_height') != 480:
        raise ValueError('ACG720 calibration must match the 800x480 FPGA image stream')
    matrices = {}
    for key, rows, columns in (('camera_matrix', 3, 3), ('rectification_matrix', 3, 3),
                               ('projection_matrix', 3, 4)):
        matrix = data.get(key, {})
        if not isinstance(matrix, dict) or matrix.get('rows') != rows or matrix.get('cols') != columns:
            raise ValueError(f'{key}: expected a {rows}x{columns} matrix')
        matrices[key] = _numbers(matrix.get('data'), rows * columns, key)
    distortion = data.get('distortion_coefficients', {})
    model = data.get('distortion_model')
    coefficient_count = {'plumb_bob': 5, 'rational_polynomial': 8, 'equidistant': 4}.get(model)
    if (coefficient_count is None or not isinstance(distortion, dict)
            or distortion.get('rows') != 1 or distortion.get('cols') != coefficient_count):
        raise ValueError('distortion_model and distortion_coefficients dimensions do not match')
    matrices['distortion_coefficients'] = _numbers(
        distortion.get('data'), coefficient_count, 'distortion_coefficients')
    intrinsic = matrices['camera_matrix']
    if intrinsic[0] <= 0 or intrinsic[4] <= 0 or abs(intrinsic[8] - 1.) > 1e-9:
        raise ValueError('camera_matrix must contain positive focal lengths and K[8]=1')
    preliminary = data.get('calibration_preliminary', True)
    if not isinstance(preliminary, bool):
        raise ValueError('calibration_preliminary must be a boolean')
    return {**matrices, 'image_width': 800, 'image_height': 480,
            'distortion_model': model, 'calibration_preliminary': preliminary}


def load_camera_mount(path):
    """Read T_lidar_camera_optical: translation in metres, fixed-axis RPY in radians."""
    data = _mapping_file(path)
    if data.get('parent_frame') != LIDAR_FRAME or data.get('child_frame') != CAMERA_FRAME:
        raise ValueError(f'camera mount must describe {LIDAR_FRAME} -> {CAMERA_FRAME}')
    vectors = {}
    for key, axes in (('translation', ('x', 'y', 'z')),
                      ('rotation_rpy', ('roll', 'pitch', 'yaw'))):
        vector = data.get(key, {})
        if not isinstance(vector, dict):
            raise ValueError(f'{key}: expected {", ".join(axes)}')
        vectors[key] = _numbers([vector.get(axis) for axis in axes], 3, key)
    roll, pitch, yaw = (value / 2 for value in vectors['rotation_rpy'])
    sr, cr, sp, cp, sy, cy = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
    quaternion = [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                  cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]
    bearing = data.get('robot_head_bearing_in_lidar_deg')
    return {'parent_frame': LIDAR_FRAME, 'child_frame': CAMERA_FRAME,
            'translation': vectors['translation'], 'quaternion': quaternion,
            'heading_offset_rad': None if bearing is None else
            math.radians(_numbers([bearing], 1, 'robot_head_bearing_in_lidar_deg')[0])}


def create_calibration_node(**kwargs):
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from tf2_ros import StaticTransformBroadcaster

    class Calibration(Node):
        def __init__(self):
            super().__init__('camera_calibration', namespace='acg720', **kwargs)
            info_file = self.declare_parameter('camera_info_file', '').value
            mount_file = self.declare_parameter('camera_mount_file', '').value
            self.info = None
            if info_file:
                data = load_camera_info(info_file)
                self.info = CameraInfo(width=data['image_width'], height=data['image_height'],
                                       distortion_model=data['distortion_model'],
                                       d=data['distortion_coefficients'], k=data['camera_matrix'],
                                       r=data['rectification_matrix'], p=data['projection_matrix'])
                self.publisher = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
                # Only the real frame header is needed; avoid forcing lossless alignment images.
                self.create_subscription(CompressedImage, PREVIEW_TOPIC,
                                         self.on_image, qos_profile_sensor_data)
                if data['calibration_preliminary']:
                    self.get_logger().warning('Camera calibration is preliminary; retain its YAML in the alignment configuration')
            else:
                self.get_logger().info('No camera_info_file: camera intrinsics remain unconfigured')
            if mount_file:
                mount = load_camera_mount(mount_file)
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = mount['parent_frame']
                transform.child_frame_id = mount['child_frame']
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = mount['translation']
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = mount['quaternion']
                self.mount_broadcaster = StaticTransformBroadcaster(self)
                self.mount_broadcaster.sendTransform(transform)
            else:
                self.get_logger().info('No camera_mount_file: lidar-to-camera transform remains unconfigured')

        def on_image(self, message):
            self.info.header = message.header
            self.publisher.publish(self.info)

    return Calibration()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = create_calibration_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

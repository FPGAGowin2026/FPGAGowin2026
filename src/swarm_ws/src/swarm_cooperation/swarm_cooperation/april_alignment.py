"""AprilGrid observations and transforms for cooperative map alignment."""

import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from . import _april
from ._april import AlignmentUnavailable, board_points, board_pose, map_alignment


def camera_model(width, height, matrix, distortion, distortion_model):
    matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if (width <= 0 or height <= 0 or matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or
            not np.isfinite(matrix).all() or not np.isfinite(distortion).all()):
        raise AlignmentUnavailable('相机内参无效')
    if distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
        raise AlignmentUnavailable('相机畸变模型暂不支持: ' + distortion_model)
    if distortion.size not in (0, 4, 5, 8, 12, 14):
        raise AlignmentUnavailable('相机畸变参数长度无效')
    return {'width': int(width), 'height': int(height), 'matrix': matrix,
            'distortion': distortion}


def model_from_info(info):
    return camera_model(info.width, info.height, info.k, info.d, info.distortion_model)


def load_camera_model(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    model = camera_model(data['image_width'], data['image_height'],
                         data['camera_matrix']['data'],
                         data.get('distortion_coefficients', {}).get('data', []),
                         data.get('distortion_model', ''))
    model['preliminary'] = bool(data.get('calibration_preliminary', False))
    return model


def detect_tags(image):
    if _april.native_detector:
        return _april.detect_tags(image)

    # OpenCV installed through Python can provide ArUco even when the C++ SDK lacks it.
    import cv2
    aruco = cv2.aruco
    if hasattr(aruco, 'ArucoDetector'):
        params = aruco.DetectorParameters()
        params.markerBorderBits = 2
        detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11), params)
        detect = detector.detectMarkers
    else:
        dictionary = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        params = aruco.DetectorParameters_create()
        params.markerBorderBits = 2

        def detect(gray):
            return aruco.detectMarkers(gray, dictionary, parameters=params)

    return _april.detect_tags(image, detect)


def observe_pair(car_image, hex_image, car_model, hex_model, tag_size, tag_spacing):
    bridge = CvBridge()
    images = []
    for message, model in ((car_image, car_model), (hex_image, hex_model)):
        if isinstance(message, CompressedImage):
            image = bridge.compressed_imgmsg_to_cv2(message, desired_encoding='bgr8')
        else:
            image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        if image.shape[:2] != (model['height'], model['width']):
            raise AlignmentUnavailable('图像分辨率与相机内参不匹配')
        images.append(image)
    return _april.poses_from_tags(detect_tags(images[0]), detect_tags(images[1]),
                                  car_model, hex_model, tag_size, tag_spacing)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return _april.transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


# ROS body axes: x forward, y left, z up; optical axes: x right, y down, z forward.
ZERO_MOUNT_BODY_FROM_OPTICAL = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                                         [0., -1., 0., 0.], [0., 0., 0., 1.]])

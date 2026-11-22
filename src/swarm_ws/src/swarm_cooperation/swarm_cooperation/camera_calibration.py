"""AprilGrid observations and OpenCV fitting shared by camera calibration tools."""

import numpy as np

from . import _april
from ._april import distinct_views, evaluate, fit, points
from .april_alignment import board_points, detect_tags


def observe_grid(image, tag_size, tag_spacing, filename=''):
    """Extract a planar observation without needing an initial camera matrix."""
    return _april.observe_grid(detect_tags(image), tag_size, tag_spacing, filename)


def camera_info_document(size, matrix, distortion, name):
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    return {'image_width': size[0], 'image_height': size[1], 'camera_name': name,
            'distortion_model': 'plumb_bob', 'calibration_preliminary': True,
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.ravel().tolist()},
            'distortion_coefficients': {'rows': 1, 'cols': int(distortion.size),
                                        'data': distortion.ravel().tolist()},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': [float(fx), 0., float(cx), 0., 0., float(fy),
                                           float(cy), 0., 0., 0., 1., 0.]}}

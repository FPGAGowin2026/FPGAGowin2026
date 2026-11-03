"""Shared timestamp and 2D observation helpers for the coordination nodes."""

import math

FUTURE_TOLERANCE_NS = 50_000_000


def stamp_ns(stamp):
    if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
        return None
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    return value if value > 0 else None


def valid_detection(message):
    if not message.header.frame_id or len(message.detections) != 1:
        return False
    detection = message.detections[0]
    if (not detection.id or detection.header.frame_id != message.header.frame_id or
            stamp_ns(detection.header.stamp) != stamp_ns(message.header.stamp)):
        return False
    box = detection.bbox
    numbers = (box.center.position.x, box.center.position.y, box.center.theta,
               box.size_x, box.size_y)
    return all(math.isfinite(value) for value in numbers) and box.size_x > 0 and box.size_y > 0

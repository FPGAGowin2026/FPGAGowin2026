"""Pure helpers for the SLAM scan adapter and read-only map view."""
from itertools import groupby
import math
import uuid


def u32(value):
    return type(value) is int and 0 <= value <= 0xffffffff


def delta(later, earlier):
    value = (later - earlier) & 0xffffffff
    return value if value < 0x80000000 else value - 0x100000000


def reduce_scan(ranges, angle_min, angle_increment, range_min, range_max):
    """按实际角度规约扫描；未观测方向保持 NaN，不扩张传感器视场。"""
    if (not ranges or not math.isfinite(angle_min) or
            not math.isfinite(angle_increment) or (angle_increment == 0 and len(ranges) > 1) or
            not math.isfinite(range_min) or not math.isfinite(range_max) or
            range_min < 0 or range_max <= range_min):
        return None
    source_step = abs(angle_increment)
    span = (len(ranges) - 1) * source_step
    if not math.isfinite(span) or span > 2 * math.pi + 1e-6:
        return None
    source_end = angle_min + (len(ranges) - 1) * angle_increment
    start = min(angle_min, source_end)
    # 细分辨率按 0.5° 保留最近回波；较粗扫描保留原分辨率。
    step = max(math.pi / 360, source_step)
    if 0 < span < step:
        step = source_step
    count = int(math.floor(span / step + 1e-9)) + 1
    bins = [math.nan] * count
    wraps = span >= 2 * math.pi - step - 1e-6
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = angle_min + index * angle_increment
        slot = int(math.floor((angle - start) / step + 0.5 + 1e-12))
        if slot == count and wraps:
            slot = 0
        if not 0 <= slot < count:
            continue
        # 掩掉车体实际遮挡的后半圈，按输出槽检查避免边界取整泄漏。
        output_angle = (start + slot * step) % (2 * math.pi)
        if math.pi / 2 - 1e-9 <= output_angle <= 3 * math.pi / 2 + 1e-9:
            continue
        if math.isnan(bins[slot]) or value < bins[slot]:
            bins[slot] = value
    return (start, step, bins) if any(math.isfinite(value) for value in bins) else None


def planar_transform(x, y, qx, qy, qz, qw, age_s):
    """Validate a fresh planar map <- odom TF before exposing it to the web."""
    values = (x, y, qx, qy, qz, qw, age_s)
    if (not all(math.isfinite(v) for v in values) or not -0.25 <= age_s <= 0.75 or
            abs(qx) > 1e-3 or abs(qy) > 1e-3 or
            not 0.99 <= qx*qx + qy*qy + qz*qz + qw*qw <= 1.01):
        return None
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
    return {'x': x, 'y': y, 'yaw': yaw, 'age_ms': max(0.0, age_s * 1000)}


class SlamMapView:
    """Keep map and sensor freshness separate from the SLAM algorithm."""
    MAX_SIDE = 2048
    MAX_CELLS = 2_000_000

    def __init__(self):
        self.generation = uuid.uuid4().hex
        self.domain = None
        self.nav = None
        self.nav_at = None
        self.scan_at = None
        self.first_scan_stamp = None
        self.map_at = None
        self.map_stamp = None
        self.map = None
        self.revision = 0
        self.reset_required = False
        self.loop_enabled = None
        self.loop_checked_at = None
        self.reason = '等待 FPGA 导航和 SLAM 地图'

    def navigation(self, data, now):
        # 建图只依赖位置有效性及坐标域，不依赖设备内使用哪一种局部规划器。
        # 这些协议由网桥同一遥测解码器校验并提供相同的域、时钟和位姿字段。
        if (data.get('protocol') not in ('nav_grid_v1', 'nav_polar_v2', 'nav_full_v1', 'velocity_only_v1') or
                data.get('rejected') or
                data.get('rx_rejection') or not all(u32(data.get(k)) for k in
                    ('boot_id', 'odom_domain', 'now_us')) or
                not data['boot_id'] or not data['odom_domain']):
            return
        domain = (data['boot_id'], data['odom_domain'])
        if self.domain is not None and self.domain != domain:
            # A pose graph cannot safely continue across an unrelated odometry frame.
            self.reset_required = True
            self.map = None
            self.reason = 'FPGA 坐标域已变化，请重启全局建图'
            return
        if self.nav is not None and delta(data['now_us'], self.nav['now_us']) < 0:
            return
        self.domain, self.nav, self.nav_at = domain, data, now

    def can_scan(self, now):
        return bool(not self.reset_required and self.nav is not None and
                    self.nav.get('pose_valid') is True and now - self.nav_at <= 0.75)

    def sent_scan(self, now, stamp_ns):
        self.scan_at = now
        if self.first_scan_stamp is None:
            self.first_scan_stamp = stamp_ns

    def loop_state(self, enabled, now):
        self.loop_enabled = enabled if type(enabled) is bool else None
        self.loop_checked_at = now

    def receive_map(self, data, now):
        if (self.reset_required or self.first_scan_stamp is None or
                data.get('frame_id') != 'map' or not isinstance(data.get('stamp_ns'), int) or
                data['stamp_ns'] < self.first_scan_stamp or
                (self.map_stamp is not None and data['stamp_ns'] <= self.map_stamp)):
            return False
        width, height, resolution = data.get('width'), data.get('height'), data.get('resolution')
        origin, cells = data.get('origin'), data.get('cells')
        if (type(width) is not int or type(height) is not int or
                not (0 < width <= self.MAX_SIDE and 0 < height <= self.MAX_SIDE) or
                width * height > self.MAX_CELLS):
            self.map = None
            self.reason = 'SLAM 地图超过网页显示范围'
            return False
        if (not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0 or
                not isinstance(origin, dict) or not all(isinstance(origin.get(k), (int, float)) and
                    math.isfinite(origin[k]) for k in ('x', 'y')) or
                data.get('orientation_identity') is not True or not hasattr(cells, '__len__') or
                len(cells) != width * height or any(v < -1 or v > 100 for v in cells)):
            return False
        values = bytearray(0 if v < 0 else 2 if v >= 65 else 1 for v in cells)
        rle = []
        for value, run in groupby(values):
            rle.extend((value, sum(1 for _ in run)))
        self.revision += 1
        self.map_at = now
        self.map_stamp = data['stamp_ns']
        self.map = {'width': width, 'height': height, 'resolution_m': float(resolution),
                    'origin': {'x': origin['x'], 'y': origin['y']}, 'cells_rle': rle,
                    'counts': {'unknown': values.count(0), 'free': values.count(1),
                               'occupied': values.count(2)}}
        return True

    def snapshot(self, now, pose=None):
        result = {'v': 1, 'type': 'global_mapping', 'protocol': 'slam_toolbox_view_v1',
                  'source': 'slam_toolbox', 'available': self.map is not None,
                  'domain': ':'.join(map(str, self.domain)) if self.domain else None,
                  'generation': self.generation, 'revision': self.revision,
                  'loop_closure_enabled': self.loop_enabled if self.loop_checked_at is not None and
                                          now - self.loop_checked_at <= 4.0 else None}
        if self.map is None:
            result.update(status='paused', reason=self.reason)
            return result
        fresh = (self.can_scan(now) and self.scan_at is not None and now - self.scan_at <= 1.5 and
                 self.map_at is not None and now - self.map_at <= 8.0)
        result.update(self.map)
        loop_note = {True: '回环检测已启用', False: '回环检测未启用', None: '回环状态待确认'}[
            result['loop_closure_enabled']]
        result.update(status='building' if fresh else 'paused',
                      reason=f'SLAM 建图中 · {loop_note}' if fresh else 'SLAM 数据停更 · 显示历史地图',
                      pose=pose if fresh else None, map_age_ms=round((now - self.map_at) * 1000))
        return result

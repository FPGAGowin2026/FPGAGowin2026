import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from global_map_core import SlamMapView, planar_transform, reduce_scan


def nav(now=1_000_000, boot=1, domain=1, valid=True):
    return dict(protocol='nav_grid_v1', boot_id=boot, odom_domain=domain,
                now_us=now, pose_valid=valid)


def grid(cells=(-1, 0, 100, 42), stamp=101, width=2, height=2):
    return dict(frame_id='map', stamp_ns=stamp, width=width, height=height,
                resolution=0.05, origin={'x': -2.0, 'y': -1.0},
                orientation_identity=True, cells=cells)


def unpack(view):
    out = []
    for value, count in zip(view['cells_rle'][::2], view['cells_rle'][1::2]):
        out.extend([value] * count)
    return out


def test_scan_reduction_keeps_nearest_return_wrap_and_unknown_angles():
    ranges = [float('nan')] * 36000
    for index in range(32):
        ranges[index * 50] = 2.0
    ranges[24] = 1.5
    ranges[25] = 1.0
    ranges[35999] = 0.5
    start, step, values = reduce_scan(ranges, 0.0, math.pi / 18000, 0.01, 65.534)
    assert start == 0.0 and step == math.pi / 360
    assert len(values) == 720
    assert values[0] == 0.5
    assert values[1] == 1.0
    assert math.isnan(values[32])
    assert sum(math.isfinite(v) for v in values) == 32


def test_scan_reduction_preserves_partial_view_sparse_returns_and_direction():
    source = [float('nan')] * 91
    source[0], source[45], source[90] = 1.0, 2.0, 3.0
    start, step, values = reduce_scan(source, -math.pi / 4, math.pi / 180, 0.01, 10)
    assert math.isclose(start, -math.pi / 4)
    assert math.isclose(start + step * (len(values) - 1), math.pi / 4)
    assert len(values) == 91 and values[0] == 1.0 and values[45] == 2.0 and values[90] == 3.0
    assert sum(math.isfinite(v) for v in values) == 3
    reverse = reduce_scan(source[::-1], math.pi / 4, -math.pi / 180, 0.01, 10)
    assert reverse[0] == start and reverse[1] == step
    assert all((math.isnan(a) and math.isnan(b)) or a == b for a, b in zip(values, reverse[2]))

    # A different origin and lower resolution are legal for a full scan.
    full = [float('nan')] * 360
    full[180] = 0.5
    start, step, values = reduce_scan(full, -math.pi, math.pi / 180, 0.01, 10)
    assert math.isclose(start, -math.pi) and math.isclose(step, math.pi / 180)
    assert len(values) == 360 and values[180] == 0.5


def test_scan_reduction_rejects_invalid_geometry_or_no_usable_returns():
    assert reduce_scan([], 0, 1, 0.01, 10) is None
    assert reduce_scan([1.0, 2.0], 0, 0, 0.01, 10) is None
    assert reduce_scan([1.0] * 10, 0, float('nan'), 0.01, 10) is None
    assert reduce_scan([1.0] * 362, 0, math.pi / 180, 0.01, 10) is None
    assert reduce_scan([float('nan'), 11.0], 0, 0.1, 0.01, 10) is None
    assert reduce_scan([1.0], 0, 0, 0.01, 10)[2] == [1.0]


def test_slam_scan_uses_fpga_front_view_without_erasing_real_near_returns():
    ranges = [float('nan')] * 36000
    for index in range(0, 36000, 50):
        ranges[index] = 2.0
    # 现场后半圈 79～158 mm 回波不能变成车心附近的全局障碍。
    for index in (8999, 9000, 12975, 18526, 22954, 27000, 27001):
        ranges[index] = 0.079
    ranges[0] = 0.150
    ranges[4500] = 0.149
    ranges[31500] = 0.151
    _, _, values = reduce_scan(ranges, 0, math.pi / 18000, 0.01, 8.0)
    assert all(math.isnan(values[i]) for i in range(180, 541))
    # 15 cm 边界附近真实前向回波全部保留，停车仍由原 RTL 处理。
    assert values[0] == 0.150
    assert values[90] == 0.149
    assert values[630] == 0.151
    assert ranges[22954] == 0.079  # 原始 /scan 不改写，仍可用于诊断。


def test_planar_map_to_odom_requires_valid_rotation_and_recent_tf():
    result = planar_transform(1.5, -2.0, 0, 0, math.sin(math.pi/4), math.cos(math.pi/4), 0.2)
    assert result['x'] == 1.5 and result['y'] == -2.0
    assert abs(result['yaw'] - math.pi/2) < 1e-9
    assert result['age_ms'] == 200
    assert planar_transform(0, 0, 0, 0, 0, 1, 0.751) is None
    assert planar_transform(0, 0, 0, 0, 0, 1, -0.251) is None
    assert planar_transform(0, 0, 0.2, 0, 0, 1, 0) is None
    assert planar_transform(float('nan'), 0, 0, 0, 0, 1, 0) is None


def test_map_waits_for_current_scan_and_replaces_whole_grid_after_correction():
    view = SlamMapView()
    view.navigation(nav(), 0)
    assert view.can_scan(0.01)
    assert not view.receive_map(grid(), 0.02)
    view.sent_scan(0.03, 100)
    view.loop_state(True, 0.03)
    assert not view.receive_map(grid(stamp=99), 0.04)
    assert view.receive_map(grid(), 0.04)
    first = view.snapshot(0.05, {'x': 1, 'y': 2, 'yaw': 0})
    assert unpack(first) == [0, 1, 2, 1]
    assert first['counts'] == {'unknown': 1, 'free': 2, 'occupied': 1}
    assert first['status'] == 'building'
    assert first['loop_closure_enabled'] is True
    assert view.receive_map(grid((100, -1, 0, 0), stamp=102), 0.06)
    second = view.snapshot(0.07)
    assert unpack(second) == [2, 0, 1, 1]
    assert second['revision'] == first['revision'] + 1
    assert second['protocol'] == 'slam_toolbox_view_v1'
    assert not view.receive_map(grid(stamp=102), 0.08)  # Repeated old maps cannot renew freshness.
    assert view.snapshot(2)['status'] == 'paused'
    assert view.snapshot(2, {'x': 1})['pose'] is None
    assert view.snapshot(5)['loop_closure_enabled'] is None


def test_loop_status_is_reported_from_slam_parameter_and_expires():
    view = SlamMapView()
    view.navigation(nav(), 0)
    view.sent_scan(0.01, 100)
    assert view.receive_map(grid(), 0.02)
    view.loop_state(False, 0.03)
    assert view.snapshot(0.04)['loop_closure_enabled'] is False
    assert '未启用' in view.snapshot(0.04)['reason']
    view.loop_state(True, 0.05)
    assert view.snapshot(0.06)['loop_closure_enabled'] is True
    assert view.snapshot(4.1)['loop_closure_enabled'] is None


def test_boot_or_odometry_change_latches_restart_and_stops_new_scans():
    view = SlamMapView()
    view.navigation(nav(), 0)
    view.sent_scan(0.01, 100)
    assert view.receive_map(grid(), 0.02)
    view.navigation(nav(1_001_000, boot=2), 0.03)
    assert view.reset_required
    assert not view.can_scan(0.03)
    assert not view.snapshot(0.03)['available']
    view.navigation(nav(1_002_000), 0.04)
    assert not view.can_scan(0.04)
    assert not view.receive_map(grid(stamp=102), 0.04)


def test_scan_freshness_depends_on_odom_domain_not_local_planner_type():
    for protocol in ('nav_grid_v1', 'nav_polar_v2', 'nav_full_v1', 'velocity_only_v1'):
        view = SlamMapView()
        view.navigation({**nav(), 'protocol': protocol}, 0)
        assert view.can_scan(0.1)
        view.navigation({**nav(1_001_000, domain=2), 'protocol': protocol}, 0.2)
        assert view.reset_required
        assert not view.can_scan(0.2)

    view = SlamMapView()
    view.navigation({**nav(), 'protocol': 'unknown'}, 0)
    assert not view.can_scan(0.1)
    view.navigation({**nav(), 'rx_rejection': True}, 0.2)
    assert not view.can_scan(0.2)


def test_nav_clock_wrap_freshness_and_invalid_pose():
    view = SlamMapView()
    view.navigation(nav(0xfffffff0), 0)
    view.navigation(nav(20), 0.1)
    assert view.can_scan(0.1)
    view.navigation(nav(10, valid=False), 0.2)
    assert view.can_scan(0.2)  # old frame did not invalidate the new state
    view.navigation(nav(30, valid=False), 0.3)
    assert not view.can_scan(0.3)
    view.navigation(nav(40), 0.4)
    assert not view.can_scan(1.2)


def test_invalid_or_oversized_map_is_not_rendered():
    view = SlamMapView()
    view.navigation(nav(), 0)
    view.sent_scan(0.01, 100)
    for bad in ({'orientation_identity': False}, {'cells': [-2, 0, 0, 0]},
                {'frame_id': 'odom'}, {'width': 2_049}, {'cells': [1]}):
        assert not view.receive_map({**grid(), **bad}, 0.02)
    assert not view.snapshot(0.03)['available']

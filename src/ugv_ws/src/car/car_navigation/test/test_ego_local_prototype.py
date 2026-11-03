"""仅离线验证二维轨迹原型；不调用 ROS、网桥或电机。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ego_local_prototype import Grid2D, Limits, plan


def make_grid(barrier=False, closed=False):
    width, height, resolution = 60, 40, 0.1
    cells = [1] * (width * height)
    if barrier:
        for y in range(0 if closed else 11, height if closed else 28):
            for x in range(20, 30):
                cells[y * width + x] = 2
    return Grid2D(width, height, resolution, 0.0, 0.0, cells)


class OfflinePrototypeTests(unittest.TestCase):
    def setUp(self):
        self.limits = replace(Limits(), footprint_radius_m=0.12)

    def test_open_space_produces_timed_trajectory(self):
        result = plan(make_grid(), [(0.8, 1.5), (4.5, 1.5)], limits=self.limits)
        self.assertEqual(result.status, 'ok', result.reason)
        self.assertEqual(result.collision_samples, 0)
        self.assertAlmostEqual(result.samples[0].x, 0.8)
        self.assertAlmostEqual(result.samples[-1].x, 4.5)
        self.assertGreater(result.samples[-1].t, 0)
        self.assertAlmostEqual(result.samples[0].vx, 0.0)
        self.assertAlmostEqual(result.samples[-1].vx, 0.0)

    def test_collision_uses_existing_guide_to_go_around(self):
        grid = make_grid(barrier=True)
        guide = [(0.8, 1.5), (1.6, 3.3), (3.4, 3.3), (4.5, 1.5)]
        result = plan(grid, guide, limits=self.limits, timeout_s=10.0)
        self.assertEqual(result.status, 'ok', result.reason)
        self.assertGreater(result.collision_samples, 0)
        self.assertTrue(all(grid.free(p.x, p.y, self.limits.footprint_radius_m)
                            for p in result.samples))
        self.assertGreater(max(p.y for p in result.samples), 2.9)

    def test_blocked_guide_and_timeout_return_no_trajectory(self):
        result = plan(make_grid(barrier=True, closed=True), [(0.8, 1.5), (4.5, 1.5)],
                      limits=self.limits)
        self.assertEqual(result.status, 'no_path')
        self.assertEqual(result.samples, ())
        timed_out = plan(make_grid(), [(0.8, 1.5), (4.5, 1.5)],
                         limits=self.limits, timeout_s=0)
        self.assertEqual(timed_out.status, 'timeout')
        self.assertEqual(timed_out.samples, ())

    def test_unknown_is_explorable_by_default_without_rewriting_grid(self):
        grid = make_grid()
        for y in range(grid.height):
            for x in range(20, 30):
                grid.cells[y * grid.width + x] = 0
        guide = [(0.8, 1.5), (4.5, 1.5)]
        result = plan(grid, guide, limits=self.limits)
        self.assertEqual(result.status, 'ok', result.reason)
        self.assertEqual(grid.cells[15 * grid.width + 25], 0)
        strict = plan(grid, guide, limits=self.limits, allow_unknown=False)
        self.assertEqual(strict.status, 'no_path')
        self.assertEqual(strict.samples, ())

    def test_time_scaling_respects_sampled_mecanum_limits(self):
        result = plan(make_grid(), [(0.8, 1.5), (4.5, 2.0)], yaw_goal=math.pi / 2,
                      limits=self.limits)
        self.assertEqual(result.status, 'ok', result.reason)
        for sample in result.samples:
            self.assertLessEqual(max(map(abs, sample.wheel_speeds)), self.limits.wheel_speed_m_s)
            self.assertLessEqual(math.hypot(sample.vx, sample.vy), self.limits.body_speed_m_s)
            self.assertLessEqual(abs(sample.wz), self.limits.yaw_speed_rad_s)
        for a, b in zip(result.samples, result.samples[1:]):
            dt = b.t - a.t
            self.assertGreater(dt, 0)
            self.assertLessEqual(max(abs(v - w) / dt for v, w in zip(a.wheel_speeds, b.wheel_speeds)),
                                 self.limits.wheel_accel_m_s2 * 1.01)


if __name__ == '__main__':
    unittest.main()

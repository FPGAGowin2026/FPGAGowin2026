"""二维 EGO 思路离线原型，不连接真实地图、ROS 控制或电机。

调用方提供完整、单一坐标系的占据栅格和已有 A* 引导路线；这里不建图、不搜索路线。
占据值沿用项目约定：0 未知、1 空闲、2 占据。默认允许规划经过未知格，
可选阻止未知格；地图边界外和已知占据始终不可通行，不更改输入栅格。
长度单位为米，时间为秒，角度为弧度。输出 vx/vy 为车体坐标系速度。

借鉴 EGO-Planner 的局部样条优化与碰撞后按需引导思想；这里使用引导线吸引
近似其碰撞项，不是对论文锚点/梯度或其求解器的完整复现。
论文：https://arxiv.org/abs/2008.08835
碰撞和动力学仅按离散采样检查，无传感器时效、旋转矩形扫掠或制动尾段保障，
不能用于真实运动控制。
"""

from dataclasses import dataclass
import math
import time
from typing import Sequence


@dataclass(frozen=True)
class Grid2D:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    cells: Sequence[int]

    def __post_init__(self):
        if (not isinstance(self.width, int) or not isinstance(self.height, int) or
                self.width <= 0 or self.height <= 0 or
                not all(math.isfinite(v) for v in (self.resolution, self.origin_x, self.origin_y)) or
                self.resolution <= 0 or len(self.cells) != self.width * self.height or
                any(v not in (0, 1, 2) for v in self.cells)):
            raise ValueError('invalid occupancy grid')

    def free(self, x, y, radius, allow_unknown=True):
        """以圆形包络查询占据；可选择未知格策略，栅格边界外不可通过。"""
        r, ox, oy = self.resolution, self.origin_x, self.origin_y
        if (x - radius < ox or y - radius < oy or
                x + radius >= ox + self.width * r or y + radius >= oy + self.height * r):
            return False
        x0, x1 = math.floor((x - radius - ox) / r), math.floor((x + radius - ox) / r)
        y0, y1 = math.floor((y - radius - oy) / r), math.floor((y + radius - oy) / r)
        for iy in range(y0, y1 + 1):
            for ix in range(x0, x1 + 1):
                value = self.cells[iy * self.width + ix]
                if value == 1 or (allow_unknown and value == 0):
                    continue
                left, bottom = ox + ix * r, oy + iy * r
                dx = x - min(max(x, left), left + r)
                dy = y - min(max(y, bottom), bottom + r)
                if dx * dx + dy * dy <= radius * radius:
                    return False
        return True


@dataclass(frozen=True)
class Limits:
    # wheel_lever_m 为麦轮角速度到轮缘线速度的几何系数，而非轮半径。
    wheel_lever_m: float = 0.178
    wheel_speed_m_s: float = 0.5
    wheel_accel_m_s2: float = 0.5
    body_speed_m_s: float = 0.4
    body_accel_m_s2: float = 0.5
    yaw_speed_rad_s: float = 0.8
    yaw_accel_rad_s2: float = 1.5
    footprint_radius_m: float = 0.231
    max_duration_s: float = 120.0


@dataclass(frozen=True)
class Sample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    wheel_speeds: tuple


@dataclass(frozen=True)
class Result:
    status: str
    reason: str
    samples: tuple = ()
    iterations: int = 0
    collision_samples: int = 0


def _knots(n, degree=3):
    return ([0.0] * (degree + 1) +
            [i / (n - degree) for i in range(1, n - degree)] +
            [1.0] * (degree + 1))


def _basis(u, knots, degree):
    if u >= 1.0:
        return [0.0] * (len(knots) - degree - 2) + [1.0]
    values = [1.0 if knots[i] <= u < knots[i + 1] else 0.0
              for i in range(len(knots) - 1)]
    for p in range(1, degree + 1):
        next_values = []
        for i in range(len(values) - 1):
            left = knots[i + p] - knots[i]
            right = knots[i + p + 1] - knots[i + 1]
            next_values.append(((u - knots[i]) * values[i] / left if left else 0.0) +
                               ((knots[i + p + 1] - u) * values[i + 1] / right if right else 0.0))
        values = next_values
    return values


def _evaluate(points, knots, degree, u):
    basis = _basis(u, knots, degree)
    return (sum(w * p[0] for w, p in zip(basis, points)),
            sum(w * p[1] for w, p in zip(basis, points)))


def _derivative(points, knots, degree):
    result = []
    for i in range(len(points) - 1):
        denominator = knots[i + degree + 1] - knots[i + 1]
        factor = degree / denominator if denominator else 0.0
        result.append(((points[i + 1][0] - points[i][0]) * factor,
                       (points[i + 1][1] - points[i][1]) * factor))
    return result, knots[1:-1], degree - 1


def _guide_progress(guide):
    distances = [0.0]
    for a, b in zip(guide, guide[1:]):
        distances.append(distances[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return distances


def _guide_point(guide, distances, u):
    target = u * distances[-1]
    for i in range(1, len(guide)):
        if target <= distances[i]:
            length = distances[i] - distances[i - 1]
            f = (target - distances[i - 1]) / length if length else 0.0
            return (guide[i - 1][0] + f * (guide[i][0] - guide[i - 1][0]),
                    guide[i - 1][1] + f * (guide[i][1] - guide[i - 1][1]))
    return guide[-1]


def _wheels(vx, vy, wz, lever):
    return (vx - vy - lever * wz, vx + vy + lever * wz,
            vx + vy - lever * wz, vx - vy + lever * wz)


def _smoothstep(s):
    # 零初末速度及加速度；同一进度用于平面位置和独立航向。
    return (s**3 * (10.0 + s * (-15.0 + 6.0 * s)),
            30.0 * s * s * (1.0 - s)**2,
            60.0 * s * (1.0 - s) * (1.0 - 2.0 * s))


def _sample_count(points, knots, resolution):
    derivative, _, _ = _derivative(points, knots, 3)
    max_derivative = max(math.hypot(x, y) for x, y in derivative)
    # 五次时间曲线的最大 du/ds 为 1.875，位置均匀采样和计时采样均覆盖。
    return max(40, math.ceil(1.875 * max_derivative / (resolution / 3.0)) + 1)


def plan(grid, guide, *, yaw_start=0.0, yaw_goal=None, limits=Limits(),
         allow_unknown=True, timeout_s=2.0, max_iterations=80):
    """离线规划。guide 是调用方给出的 (x, y) 米坐标序列。

    返回 Result；仅 status=='ok' 时 samples 非空，其他状态没有可执行输出。
    四轮顺序沿用本工程轮式模型，轮缘速度分别为
    (vx-vy-k*wz, vx+vy+k*wz, vx+vy-k*wz, vx-vy+k*wz)。
    """
    started = time.monotonic()
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        return Result('timeout', '规划时间预算已用尽')
    deadline = started + timeout_s
    if yaw_goal is None:
        yaw_goal = yaw_start
    limit_values = tuple(vars(limits).values())
    if (not isinstance(grid, Grid2D) or not isinstance(allow_unknown, bool) or
            not isinstance(max_iterations, int) or
            max_iterations < 0 or not all(math.isfinite(v) and v > 0 for v in limit_values) or
            not math.isfinite(yaw_start) or not math.isfinite(yaw_goal)):
        return Result('invalid_input', '参数无效')
    try:
        path = [(float(p[0]), float(p[1])) for p in guide]
    except (TypeError, ValueError, IndexError):
        return Result('invalid_input', '引导路线无效')
    if len(path) < 2 or not all(math.isfinite(v) for p in path for v in p):
        return Result('invalid_input', '引导路线至少需要两个有限点')
    lengths = _guide_progress(path)
    if lengths[-1] <= 1e-9:
        return Result('invalid_input', '引导路线长度为零')

    # 先独立采样验证已有 A* 引导线，不能假设跨地图更新后仍有效。
    for a, b in zip(path, path[1:]):
        steps = max(1, math.ceil(math.dist(a, b) / (grid.resolution / 3.0)))
        for j in range(steps + 1):
            if time.monotonic() >= deadline:
                return Result('timeout', '引导线检查超时')
            t = j / steps
            if not grid.free(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                             limits.footprint_radius_m, allow_unknown):
                return Result('no_path', '引导路线与占据、不可通过区域或边界冲突')

    count = max(8, min(20, math.ceil(lengths[-1] / 0.3) + 3))
    points = [(path[0][0] + (path[-1][0] - path[0][0]) * i / (count - 1),
               path[0][1] + (path[-1][1] - path[0][1]) * i / (count - 1))
              for i in range(count)]
    knots = _knots(count)
    collision_samples = 0
    for iteration in range(max_iterations + 1):
        samples = _sample_count(points, knots, grid.resolution)
        if samples > 2048:
            return Result('no_path', '轨迹采样量超过离线原型上限', iterations=iteration,
                          collision_samples=collision_samples)
        forces = [[0.0, 0.0, 0.0] for _ in points]
        hits = 0
        for i in range(samples):
            if time.monotonic() >= deadline:
                return Result('timeout', '样条优化超时', iterations=iteration,
                              collision_samples=collision_samples)
            u = i / (samples - 1)
            basis = _basis(u, knots, 3)
            x = sum(w * p[0] for w, p in zip(basis, points))
            y = sum(w * p[1] for w, p in zip(basis, points))
            if grid.free(x, y, limits.footprint_radius_m, allow_unknown):
                continue
            hits += 1
            target = _guide_point(path, lengths, u)
            for j in range(1, count - 1):
                forces[j][0] += basis[j] * (target[0] - x)
                forces[j][1] += basis[j] * (target[1] - y)
                forces[j][2] += basis[j]
        collision_samples += hits
        if not hits:
            break
        if iteration == max_iterations:
            return Result('no_path', '样条未能在迭代预算内避开障碍', iterations=iteration,
                          collision_samples=collision_samples)
        # 碰撞采样才生成引导梯度；邻点平滑项避免局部尖折。
        previous = points
        points = list(previous)
        for j in range(1, count - 1):
            fx, fy, weight = forces[j]
            smooth_x = (previous[j - 1][0] + previous[j + 1][0]) / 2.0 - previous[j][0]
            smooth_y = (previous[j - 1][1] + previous[j + 1][1]) / 2.0 - previous[j][1]
            points[j] = (previous[j][0] + (0.45 * fx / weight if weight else 0.0) + 0.03 * smooth_x,
                         previous[j][1] + (0.45 * fy / weight if weight else 0.0) + 0.03 * smooth_y)

    first, first_knots, first_degree = _derivative(points, knots, 3)
    second, second_knots, second_degree = _derivative(first, first_knots, first_degree)
    yaw_delta = math.atan2(math.sin(yaw_goal - yaw_start), math.cos(yaw_goal - yaw_start))
    normalized = []
    speed_factor = accel_factor = 0.0
    for i in range(samples):
        if time.monotonic() >= deadline:
            return Result('timeout', '时间分配超时', iterations=iteration,
                          collision_samples=collision_samples)
        s = i / (samples - 1)
        u, du, ddu = _smoothstep(s)
        x, y = _evaluate(points, knots, 3, u)
        if not grid.free(x, y, limits.footprint_radius_m, allow_unknown):
            return Result('no_path', '时间分配后的轨迹采样发生碰撞', iterations=iteration,
                          collision_samples=collision_samples)
        dx, dy = _evaluate(first, first_knots, first_degree, u)
        ddx, ddy = _evaluate(second, second_knots, second_degree, u)
        yaw = yaw_start + yaw_delta * u
        c, sn = math.cos(yaw), math.sin(yaw)
        vx_world, vy_world = dx * du, dy * du
        ax_world, ay_world = ddx * du * du + dx * ddu, ddy * du * du + dy * ddu
        vx, vy = c * vx_world + sn * vy_world, -sn * vx_world + c * vy_world
        wz, az = yaw_delta * du, yaw_delta * ddu
        ax = c * ax_world + sn * ay_world + wz * vy
        ay = -sn * ax_world + c * ay_world - wz * vx
        wheels = _wheels(vx, vy, wz, limits.wheel_lever_m)
        wheel_acc = _wheels(ax, ay, az, limits.wheel_lever_m)
        speed_factor = max(speed_factor, max(map(abs, wheels)) / limits.wheel_speed_m_s,
                           math.hypot(vx, vy) / limits.body_speed_m_s,
                           abs(wz) / limits.yaw_speed_rad_s)
        accel_factor = max(accel_factor, max(map(abs, wheel_acc)) / limits.wheel_accel_m_s2,
                           math.hypot(ax, ay) / limits.body_accel_m_s2,
                           abs(az) / limits.yaw_accel_rad_s2)
        normalized.append((s, x, y, yaw, vx, vy, wz, wheels))
    duration = max(0.001, speed_factor, math.sqrt(accel_factor)) * 1.02
    if duration > limits.max_duration_s:
        return Result('no_path', '限速后的轨迹超出最大时长', iterations=iteration,
                      collision_samples=collision_samples)
    output = tuple(Sample(s * duration, x, y, math.atan2(math.sin(yaw), math.cos(yaw)),
                          vx / duration, vy / duration, wz / duration,
                          tuple(w / duration for w in wheels))
                   for s, x, y, yaw, vx, vy, wz, wheels in normalized)
    return Result('ok', '离线采样通过', output, iteration, collision_samples)

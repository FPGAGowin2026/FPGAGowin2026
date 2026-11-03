# 二维局部轨迹优化离线原型

`scripts/ego_local_prototype.py` 是独立离线研究原型，没有启动入口、ROS 订阅或电机输出。它参考 [EGO-Planner](https://arxiv.org/abs/2008.08835) 按需碰撞信息和样条优化的思想；碰撞点朝调用方引导线移动是简化近似，并非论文中障碍物表面锚点、梯度和 L-BFGS 的完整实现。

## 接口

`Grid2D(width, height, resolution, origin_x, origin_y, cells)` 接收行优先栅格；`0` 未知、`1` 空闲、`2` 占据。分辨率、原点和路径坐标使用米。`plan(grid, guide, yaw_start=0, yaw_goal=None, limits=Limits(), allow_unknown=True, timeout_s=2, max_iterations=80)` 接收调用方已有的 A* 路线，路线包含起点和终点；不自行建图或再运行 A*。默认允许探索未知格，`allow_unknown=False` 将未知格判为不可通过；输入栅格始终保持未知值。已知占据和地图边界外仍不可通过。航向单位为弧度，输入栅格和路线必须处于同一坐标系。

`result.status` 为 `ok`、`no_path`、`timeout` 或 `invalid_input`。仅 `ok` 时包含 `samples`；每个样本有相对时间 `t`、地图坐标 `x/y`、航向 `yaw`、车体坐标速度 `vx/vy`、角速度 `wz` 和 `wheel_speeds`。速度单位为 m/s，角速度为 rad/s。四轮线速度与现有工程约定一致：`vx-vy-k*wz`、`vx+vy+k*wz`、`vx+vy-k*wz`、`vx-vy+k*wz`；默认 `k=0.178 m`，即车体角速到轮缘速度的几何系数，不是轮半径。`Limits` 的速度、加速度和圆形包络参数可显式覆盖。

在 `car_navigation` 目录执行时，例如：

```python
from scripts.ego_local_prototype import Grid2D, plan

grid = Grid2D(30, 30, 0.1, 0.0, 0.0, [1] * 900)
result = plan(grid, [(0.5, 1.5), (2.4, 1.5)], yaw_goal=0.5)
if result.status == 'ok':
    print(result.samples[-1])  # 仅用于离线检查
else:
    print(result.status, result.reason)
```

从 `car_navigation` 目录运行必要的离线回归：

```bash
python3 -m unittest discover -s test -p test_ego_local_prototype.py -v
```

## 边界

原型使用圆形包络和有限采样；时间缩放只约束采样点上的机身速度、角速度、轮速与加速度。允许探索未知格不表示该区域已观测或可安全通行。它没有实现矩形车体连续扫掠、制动尾段、雷达盲区观测策略、地图和位姿新鲜度、动态障碍预测、在线反馈跟踪、控制权限或硬件停车保护。不能将输出连接到真实车辆，也不能将离线通过解释为轨迹或车辆安全保证。接入前需使用同一份新鲜局部图、现有车体检查和看门狗，并独立验证 Jetson 耗时及实车运动。

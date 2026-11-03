# Jetson 局部导航

`jetson_local_navigation` 是兼容速度型 FPGA 固件的局部执行器：只有 FPGA 报告 `velocity_only_v1` 时才建图和计算候选速度。FPGA 报告自身局部导航协议时，该节点清理已有会话并停止局部计算，避免双规划器争用。全局 A* 和航点调度由 Jetson 的 `jetson_global_navigation` 负责，网页只提交并显示目标；具体执行器由网桥按 FPGA 能力选择。兼容执行器复用 FPGA 的局部 A* 搜索，在唯一的 Jetson 局部栅格上寻找引导点，再由实时车体与制动扫掠验证候选速度。不能把旧位流的地面航点结果当作当前组合链路的验证。

全局搜索先使用 191 mm 半径；没有路线时再尝试 135 mm，以提出窄通道路线。较小半径的结果只是路径建议，车体能否通过及能否转向仍由局部执行器的实时矩形车体和制动扫掠决定。

## 接口

节点使用单线程执行器，因为栅格和移植的控制状态是单实例、不可并发调用。启动后每 100ms 发布空闲或任务状态；具备会话后每 200ms 发布局部图快照。地图就绪由快照标志表示，需要有效位姿和完整雷达扫描，不要求先下发任务。

* 输入：`/fpga/round5/navigation`、`/fpga/round5/fusion`、`/odom/filtered`、`/scan` 和 `/fpga/bidirectional_status`。局部节点直接使用与融合同一实时遥测链路的导航状态，避免持久化状态队列的旧消息导致时钟与位姿历史错位；外部人工或视觉控制来源接管时撤销旧任务的固定起点许可。
* 命令：`/jetson/navigation/command`，类型 `ugv_bridge/srv/NavigationCommand`。目标坐标为 odom 米，角度为弧度；位置容差支持 1–500 mm，按请求值判定到达。`action=3, task_id=0` 无条件取消旧任务，显式取消使用递增事务号。接收成功只表示接受，结果以状态为准。
* 输出：`/jetson/navigation/cmd_vel` 为带网桥 ID 和授权 epoch 的 `AuthorizedTwist`，车体系线速度 m/s、角速度 rad/s；每 20ms 更新，取消时发送零速。网桥负责撤销授权、转发速度与停车。
* `/jetson/navigation/state` 使用现有状态码和事务水位；终态持续发布，`last_task_id` 保留最新事务。`/jetson/navigation/local_grid` 使用 `jetson_grid_snapshot_v1`，`/jetson/navigation/local_plan` 使用 `jetson_local_segment_v1`；两者的 `source` 都为 `jetson`，原 FPGA 状态话题独立保留。

默认坐标系 `odom`、`base_link`、`n10_sensor`，对应参数 `odom_frame_id`、`base_frame_id`、`lidar_frame_id`。当前共享栅格只实现雷达相对车体 `x=0.13m, y=0, yaw=0`；`lidar_x_m`、`lidar_y_m`、`lidar_yaw_rad` 必须与之匹配，不能通过改参数假装更换安装位置。节点在 FPGA 明确报告融合位姿失效时清除缓存位姿并输出零速；FPGA 聚合位姿有效性包含融合、IMU 和轮速条件，Jetson 不声称独立诊断各传感器。优先使用新协议的 `guard_stop` 反馈，旧 `guard_error` 编码可能被其他错误覆盖，仅供兼容。

ROS 服务校验外部请求后，通过进程内的类型化接口提交航点；该接口保留事务编号、有效期和地图重置水位，不再模拟旧 FPGA 任务分片。

## 时间和坐标的限制

网桥的 `/scan` 是整圈结束后发送的扫描级数据，没有逐束时标。节点用扫描起始时刻附近的融合位姿投影整圈，最多接收 500ms 前开始的扫描；保留原始扫描时标，不把接收时间冒充采样时间。运动中这会造成畸变，且与旧 FPGA 逐束历史位姿及 120ms 限龄并不等价。整圈耗时、链路延迟和车辆转速需要在目标机核对；如果输入过旧，原导航控制的 500ms 雷达时效限制会阻止运动。

雷达扫描匹配接受时，原始遥测没有同一时刻的修正前后位姿对。节点根据上一融合状态和车体速度外推近似修正前位姿，再对局部地图锚点和历史位姿做重定位；漏掉多次修正或间隔过大则清图并取消活动任务。该外推不是原 FPGA 的精确同一时刻重定位，需要统一验证。启动、重启或 odom 域变化时会重置本地图，不把旧坐标系的任务和地图继续用于新域。

新协议 kind8 的 `guard_stop` 与原 FPGA CPU 的反馈相同：为避免启动闭锁造成反馈环，在 hard-kill 有效及其释放后短暂流水期内屏蔽该反馈。此位为假不表示 hard-kill 等硬件闭锁已解除，电机仍受未屏蔽的硬件停车链控制；kind6 的原始 guard 状态保持独立。有效的 guard 反馈会使 Jetson 输出零速并保留任务，反馈解除后按地图和位姿重新规划。该反馈仍可能随指令或雷达状态变化；启动和障碍前的实际恢复时序需结合硬件验证。

## 构建与验证入口

完成整个工作区代码后，在 ROS 2 环境的 `ugv_ws` 下执行。构建时要求同级 `FPGA_ws` 中的 `boards/BX72_138K_robot/src/firmware/lite/local_path.c` 和 `local_path.h`，因为两个执行器编译同一份搜索实现：

```bash
colcon build --packages-up-to car_navigation
colcon test --packages-select car_navigation --event-handlers console_direct+
colcon test-result --verbose
```

`test_jetson_local_navigation.c` 迁移原有局部导航回归，并补充网桥撤权后的强制取消检查。`test_jetson_local_node.py` 启动真实局部导航节点，以合成的会话、融合位姿和完整扫描检查地图、授权速度、显式取消和停止；它不连接实际网桥或车辆。`scripts/ego_local_prototype.py` 是离线二维样条候选研究，不接在线控制，接口和限制见 [离线原型说明](ego_local_prototype.md)。

# Jetson 全局规划与航点调度

`jetson_global_navigation_node.py` 在 Jetson 上运行全局 A* 和航点调度；网页负责提交目标及显示状态，不承担路径搜索或分段事务。节点使用现有全局路线搜索器、局部地图解码和航点状态机；内部 Node 子进程负责计算，ROS 适配层负责执行器接口。`node` 运行时要求 Node.js 22 或更高版本。

## 接口

- `/jetson/global_navigation/command`：`ugv_bridge/srv/GlobalNavigation`。`request_json` 为对象，`action` 为 `start`、`stop` 或 `cancel`；`start` 还包含 `frame`（`map` 或 `odom`）、`domain`、`generation`（`map` 地图代次，`odom` 为 `null`）、`goals`（`[{x,y}, ...]`，米），以及 `max_speed_mm_s`、`max_accel_mm_s2`、`tolerance_mm`、`yaw_tolerance_mrad`、`duration_s`。单次最多 64 个航点。
- 服务成功只表示目标已接收或已请求停止，不表示设备已开始运动、已停止或已到达。完整状态由 `/jetson/global_navigation/state` 的 JSON String 提供；`route` 为规划路线，不是外部测得的实际行驶轨迹。路线超过 256 点或状态 UTF-8 超过 24 KiB 时明确报告超出容量，不发送截断后冒充完整的路线。
- `/navigation/executor/command`、`/navigation/executor/state` 和 `/navigation/executor/local_grid` 是网桥提供的统一执行器接口。FPGA 模式下由 FPGA 完成局部规划和运动控制；兼容现有的 Jetson 局部执行器。目标与设备状态的 `boot_id`、`odom_domain`、任务编号必须一致。网桥的任务分片 ACK 只说明传输入队；调度器通过同一任务编号的活动、拒绝或终态判断执行情况。ACK 后 2 秒仍没有对应状态时请求停止并报告执行结果未知，停止本身也须等待设备确认。
- `/fpga/round5/fusion` 提供融合位姿；全局地图从 Jetson 本地 `http://127.0.0.1:8766/global-map.json` 读取。坐标域、新鲜度或栅格不可用时不会继续派发下一段。

网页提交后的任务由 Jetson 独立执行，网页关闭不会自动取消；显式停止或取消仍由执行器处理。当前接口不支持 `allow_disconnect=false`，若显式提供该值会拒绝并说明，不会静默忽略。

## 坐标与碰撞检查

`map` 目标保留原始全局地图坐标，每段派发时用最新 `map←odom` 转为 odom。若运行中变换使已派发的段终点相对原 map 段终点发生明显漂移，调度器请求取消、等待执行器确认终态，再用新变换重新规划；`odom` 目标保持原始 odom 语义。两种坐标的航点均在设备报告到达后，用对应坐标系下的最新估计位姿复核目标容差。全局导航最多一次只持有一个局部事务，不能以网络 ACK 代替设备到达。

全局路线和受阻绕行均以 191 mm 圆形余量检查已知占据，来自 260×250 mm 车体的外接半径加 10 mm；局部执行器另行检查矩形车体与制动扫掠。未知区域仍是未知，可作为全局探索路线的候选，但实际运动须由局部新鲜观测判断。尺寸、轮径、雷达外参和实际制动距离仍需实测标定，该参数不是对实车安全间距的证明。

SLAM 和局部栅格都受观测方向、扫描时间与位姿精度限制。状态中的融合轨迹是里程估计，不是外部真值；实际到点与避障效果需单独实车验证。

# 小车 FPGA 接入

本包对应 Jetson 实际使用的 `ugv_bridge`。遥测、命令、视频独立运行；不再安装另一套任务规划脚本。地图航点、探索及协同目标统一交给 `car_navigation`。

| 可执行文件 | 作用 |
| --- | --- |
| `ugv_bridge_node` | 接收 UDP 6104；发布 `/scan`、`/odom`、IMU、FPGA 状态和局部地图 |
| `robot_command` | 接收局部目标、人工速度、Nav2 航点速度和停车请求，维护 FPGA 控制会话 |
| `robot_video_node` | 接收 UDP 6102；发布相机、检测显示及 HTTP 8081 视频/地图/任务接口 |

`srv/MissionStart.srv` 是网页与 `car_mission` 仍使用的视觉任务接口，保留包名和字段。旧局部控制使用的消息和服务已随旧实现删除。

在 ROS 2 Humble 环境构建：

```bash
colcon build --packages-up-to car_bringup --executor sequential
source install/setup.bash
```

完整启动配置见 `car_bringup/config/bringup.yaml`。任务空闲启动不会建立运动会话；已有网关或视频实例时应使用 `car_navigation/mission_services.launch.py`，避免重复绑定端口。

运行中，遥测网关读取 UDP 套接字的异步错误并记录原因。描述符失效时，接收线程清除未完成的扫描与地图分片，在原端口重新绑定，随后继续发布遥测。

RX 质量训练每轮扫描 16 个相位，每相位 0.6 秒。从扫描中途开始供包时，可能需要近两轮 19.2 秒再加稳定和复核时间，因此会话建立的总供包窗口为 24 秒，期间只发送零速包。单包 ACK 等待、手动输入 150 ms 截止和任务分片 180 ms 窗口不随之延长；训练期间过期的手动输入不会补发。

相机、扫描戳来自接收与 FPGA 时钟映射，不等同于精确曝光时间或逐束激光采样时间。命令 ACK 只表示请求被接收，动作和停车结果以新鲜设备反馈为准。

合法且新鲜的手动输入（含零速）被命令桥接纳时，`manual_takeover` 通知导航和视觉让出控制权。手动速度按 DDS 发布源时间执行 150 ms 有效期。

`robot_command` 只有收到速度包的严格 ACK 才报告 `manual`，按住续传不重复报告。缺 ACK 时报告 `manual_unconfirmed`，保留该次输入原有的 150 ms 截止时间；后续真实 ACK 会恢复 `manual` 状态。历史命令错误不能代替当前链路反馈。

`/fpga/waypoint_velocity` 接收通过 Collision Monitor 的带时戳速度，使用现有 FPGA 流式速度包。配套 BX72 位流接受 0–0.50 m/s 前进和 ±0.80 rad/s 转向，按源时间执行 150 ms 有效期。首包在 20 ms 内未收到 ACK 时，只对仍有效且最新的速度用新序号补发一次。补发仍缺 ACK 且当前会话遥测新鲜时，网关输出限频警告，速度沿用原截止时间。手动接管和停车优先。仍使用旧版位流时，将 `waypoint_nav.yaml` 的 `vx_max` 设为 `0.40`。

类型 1 心跳只刷新链路，不切换运动模式。每次探测最多等待 20 ms，缺 ACK 时仅在遥测新鲜且会话令牌、boot 和 odom domain 都与已建立会话相符时保留连接；这不代表该次心跳已到达。速度、模式和任务包仍保留完整 CRC、序号、令牌与负载镜像校验。

停车分别发送零速度和禁用自主模式。缺 ACK 时报告 `stop_unconfirmed` 并重试停车。两类 ACK 收齐后，若当前会话遥测仍过期，网关继续禁止非零输出，用心跳维持链路并等待新遥测。会话变化或心跳超时时重新执行停车握手。当前会话遥测新鲜后报告 `stop_requested`，实际停车由新鲜 FPGA 执行反馈确认。

命令链路出现套接字错误、超时或遥测过期时报告 `link_recovering`，清除待发指令并持续尝试零速停车握手。短时 ACK 丢失优先重试现有会话，收到不同的 FPGA 会话标识或套接字失效时重新建立会话。导航只在停车确认、定位有效且原 FPGA 执行会话与里程计域一致时重算原任务。

Jetson 默认构建 TensorRT 检测节点。仅在没有 TensorRT 的开发机验证 Python 任务与接口时，可传入 `--cmake-args -DBUILD_TENSORRT_VISION=OFF`；这种构建不能验证实际推理节点。

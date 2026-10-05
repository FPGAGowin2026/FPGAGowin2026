> 当前状态与候选身份见[项目状态](project_status_20261005.md)；板级配置和原始证据见[BX72 基线记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md)。

# Jetson 与 FPGA 数据契约

更新：2026-10-05。当前主线为 BX72＋Jetson，实际 ROS 包名为 `fpga_gateway`。当前源码已包含三轴速度命令与 WASD 网关；本文将现有协议、后续扩展和历史差速协议分别记录。

## 当前 BX72 协议入口

| 通道 | 端口与状态 | 实际定义 |
| --- | --- | --- |
| RTP/UDP 视频与状态 | Jetson 6102；已有视频共存实板验收 | [板级视频说明](../FPGA_ws/boards/BX72_138K_robot/docs/video_jetson.md) |
| 双向诊断 | FPGA 5000 下行、Jetson 6103 应答；type=1/2 诊断；type=3/4 为速度请求/入队回执 | [BX72_BIDIRECTIONAL](../ros_ws/src/robot/fpga_gateway/BX72_BIDIRECTIONAL.md) |
| BX72 v1 五类遥测 | Jetson 6104；IMU、电机/里程计、N10、融合、健康五类；远端有短窗口板测 | [板级线格式](../FPGA_ws/boards/BX72_138K_robot/docs/jetson_udp_telemetry.md)、[ROS 接入](../ros_ws/src/robot/fpga_gateway/BX72_SENSORS.md) |
| 四轮运动命令与反馈 | FPGA 5000 / Jetson 6103；32 字节 BX72 v1 type=3/4，已有源码与远端悬空板测记录 | [遥控操作与协议](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)及[开发计划](后续开发计划.md) |

BX72 v1 分别记录物理 CH0～CH3、逻辑 A/B/C/D、目标/实测轮速与融合状态；轮速绝对尺度和定位精度仍需实测。ROS Header 当前为接收时间，自定义 timestamp_us 为 FPGA 采集提交时间；未完成时钟映射。传感器正确性、时间映射与里程计发布需分别验收。

当前速度协议采用大端 32 字节报文，包含会话 token、序号、vx/vy（mm/s）、wz（mrad/s）与 CRC16；会话以零速包建立，重复/乱序及旧会话被拒绝。FPGA 使用 300 ms 本地看门狗；type=4 回执表示入队，不表示机械动作已完成。详细偏移、范围与桥接行为以[遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)和 RTL 为准。旧差速 C++ 网关不能直接发送此协议。测速尺度、落地负载、ROS /cmd_vel 接入及更完整的数据年龄/故障语义仍待验证或开发。

## 已实现的三轴速度线格式

以下是当前 FPGA 速度报文，不是 6104 的传感器遥测头。固定总长 32 字节，大端序：

| 偏移 | 字节数 | 内容 |
| --- | --- | --- |
| 0 | 4 | ASCII BX72 |
| 4 / 5 | 各 1 | version=1；type=3 请求或 type=4 入队回执 |
| 6 | 2 | 总长 32 |
| 8 | 4 | uint32 序号，允许模 2^32 回绕 |
| 12 | 8 | uint64 会话 token，Jetson 启动 time.time_ns() |
| 20 / 22 / 24 | 各 2 | int16 vx/vy（mm/s）、wz（mrad/s） |
| 26 | 4 | 保留字段，必须为 0 |
| 30 | 2 | CRC16-CCITT-FALSE，初值 FFFF，覆盖前 30 字节 |

vx/vy 合法范围为 ±300 mm/s，wz 为 ±1000 mrad/s。固定来源为 Jetson 192.168.0.4:6103，目标 FPGA 192.168.0.2:5000；网络/FCS、端点、应用 CRC、范围和保留字段通过后才提交。新会话通过更大 token 的零速包建立，同会话拒绝重复/倒序；RX 复位后重新建立。

本机约 50 Hz 发当前请求，Jetson 约 20 Hz 转发；PC 超过 250 ms 无新包时网关置零，回执超过 100 ms 未匹配时置零并重新握手，超过 200 ms 未收到有效回执时标记不在线。FPGA 非零速度看门狗约 300 ms，超时清使能、输出 00；链路掉线也停止。超时后须零速解锁，非零包不能直接恢复。UART `0` 保持停车优先级。以上是程序门限，机械停转和滑行距离须另测。

遥控网关需独占 6102/6103/6104，不能与 ROS 接收器/探测器同时绑定这些端口。ROS `/cmd_vel` 接入和控制来源切换仍待实现/验证。真实命令和完整停车条件见[遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)。

## 规划中的模块契约

| 对象 | 必需语义 |
|---|---|
| 运动请求 | 车体系 vx/vy/wz、单位、来源、会话/启动标识、序号、有效期及新鲜度依据 |
| 本体状态 | 四轮轮速与轮序、车体速度、输入来源、估计对齐时间、有效性、数据年龄、质量 |
| 环境约束 | 相对车体外形的方向间隙、方向覆盖、观测时间、有效性、连续性、TTC 及可信度 |
| 安全结果 | 请求与最终允许指令、安全模式、原因码、并发故障、输出许可、最近接受序号 |
| LiDAR 上传 | 扫描/分包标识、角度与距离、采样时序依据、校验和缺失状态 |

坐标采用 x 前、y 左、z 上，wz 为绕 z 正向；协议统一明确单位与字节序。接收、采样、估计时间分别表达，未知值携带无效原因。上表新增环境约束/故障结果等完整线格式尚未冻结；当前速度包和五类遥测已经有各自定义，不得直接扩字段并视为兼容。

## 后续指令许可与故障语义要求

- 重复、乱序、过期、错误或非授权来源指令均不得刷新运动许可。序号更新与刚收到不等于生成时间新鲜。
- 启动/会话标识区分重启前后报文。有效期必须有可判定时间依据；可采用 FPGA 本地有界许可窗口，窗口不得被延迟旧包延长。当前实现为会话/序号检查和本地 300 ms 租约；更完整生成时间新鲜度证明仍待扩展。
- Jetson 桥接检查上游请求新鲜度，不能给冻结的旧速度反复更换序号；心跳不能替代运动许可。
- 无效观测不能当作零速度或无障碍；无有限 TTC 与 TTC 未知分别编码。
- 解除急停或锁存故障后，需满足恢复条件、重新使能和新鲜命令，不自动恢复缓存运动。
- 本地 Watchdog、仲裁和输出门控仅依赖本地有效信息，不依赖跨机时间映射或上传成功。

## 传输约束

链路采用 RGMII、MAC、IPv4、UDP 与自定义二进制协议。完整报文通过规定校验后才原子提交；明确 MAC FCS、IP/UDP 及应用完整性校验的责任。固定端点可采用静态地址与必要 ARP，不扩展为通用网络栈。

控制/状态与 LiDAR 上传使用有界队列及明确优先级；上传拥塞不得阻塞本地保护。明确 MTU、分包、丢包、队列满和不支持报文处理。实时请求优先最新有效值，频率与超时需依据延迟预算确定。

## 历史记录使用边界

下文保留原差速协议及黄金向量，用于追溯已有代码；它只有线速度/角速度、左右轮和三路测距，不包含四轮麦轮所需完整语义。历史频率、端口、状态位及恢复规则不是新方案冻结指标；测试与构建入口也可能已失效。不得据此宣称已具备新方案的数据年龄、会话或动态包络能力。

---

## 历史差速协议记录

以下是现有差速代码对应的历史协议，不是四轮麦轮规划的线格式。所有多字节整数均为网络字节序（大端）；不直接传输 C/C++ 结构体内存。

### 报文格式

每个 UDP 数据报恰好包含一个报文：

| 字段 | 类型 | 字节数 | 说明 |
|---|---:|---:|---|
| magic | uint32 | 4 | 固定为 `0x46504741`（ASCII `FPGA`） |
| version | uint8 | 1 | 固定为 `1` |
| message_type | uint8 | 1 | `1` 控制，`2` 遥测，`3` 状态 |
| payload_length | uint16 | 2 | payload 的实际字节数 |
| sequence | uint32 | 4 | 发送方递增序号 |
| timestamp_us | uint64 | 8 | 发送方微秒时间戳 |
| payload | bytes | 可变 | 由 `message_type` 决定 |
| crc16 | uint16 | 2 | CRC16-CCITT-FALSE，覆盖头和 payload |

固定头长 20 字节，CRC 长 2 字节。接收端必须检查 magic、version、已知类型、总长度与 `payload_length` 一致性，以及 CRC；任一检查失败即丢弃。控制报文还应丢弃重复或倒序序号；`0xffffffff` 后的 `0` 视为更新序号。

CRC16-CCITT-FALSE 参数：多项式 `0x1021`、初值 `0xffff`、不反射输入/输出、最终异或值 `0x0000`。CRC 字段本身采用网络字节序，且不参与 CRC 计算。

用于 FPGA RTL 对照的控制报文黄金向量：`sequence=0x01020304`、`timestamp_us=0x0102030405060708`、`linear_mm_s=-1000`、`angular_mrad_s=2500`、`flags=1` 时，完整报文为：

```text
46 50 47 41 01 01 00 09 01 02 03 04 01 02 03 04
05 06 07 08 FF FF FC 18 00 00 09 C4 01 85 AA
```

### 消息类型

#### Control（类型 1，payload 9 字节）

| 字段 | 类型 | 单位 | 说明 |
|---|---:|---|---|
| linear_mm_s | int32 | mm/s | 线速度命令 |
| angular_mrad_s | int32 | mrad/s | 角速度命令 |
| flags | uint8 | - | bit0：急停请求 |

#### Telemetry（类型 2，payload 50 字节）

字段顺序如下，所有 `int32` 和 `uint32` 为 4 字节，所有 `uint16` 为 2 字节：

| 字段 | 类型 | 单位 |
|---|---:|---|
| left_ticks, right_ticks | int32 | 编码器计数 |
| left_velocity_mm_s, right_velocity_mm_s | int32 | mm/s |
| accel_x_mm_s2, accel_y_mm_s2, accel_z_mm_s2 | int32 | mm/s² |
| gyro_x_mrad_s, gyro_y_mrad_s, gyro_z_mrad_s | int32 | mrad/s |
| range_front_mm, range_left_mm, range_right_mm | uint16 | mm |
| status_bits | uint32 | 位状态 |

`status_bits` 在机器人网络 RTL 中冻结如下；位 `14..31` 保留为 `0`：

| 位 | 名称 | 置位条件 |
|---:|---|---|
| 0 | active_mode | `1` Jetson 协同模式，`0` FPGA 独立模式 |
| 1 | motion_enabled | 最终逻辑运动输出已使能 |
| 2 | physical_estop | 物理急停输入有效 |
| 3 | software_estop | 最近合法 Control 的软件急停有效 |
| 4 | safety_stop | 测距安全层要求停车 |
| 5 | control_command_active | Control 看门狗内存在有效 Jetson 命令 |
| 6 | ranges_valid | 三路测距当前均有效 |
| 7 | path_active | FPGA 独立路径正在运行 |
| 8 | path_finished | FPGA 独立路径已完成 |
| 9 | avoidance_active | 局部绕障状态机有效 |
| 10 | switch_pending | 模式切换等待停车或零输出保护 |
| 11 | critical_fault | 严重故障输入有效 |
| 12 | rx_control_fifo_full | 网络域到系统域 Control FIFO 满，经系统域同步 |
| 13 | tx_frame_fifo_full | 系统域到网络域返回报文 FIFO 满 |

#### Status（类型 3，payload 16 字节）

| 字段 | 类型 | 说明 |
|---|---:|---|
| mode | uint8 | `0` FPGA 独立模式，`1` Jetson 协同模式 |
| motion_enabled | uint8 | 运动输出是否允许 |
| estop | uint8 | 物理或逻辑急停状态 |
| safety_stop | uint8 | FPGA 安全停车状态 |
| fault_bits | uint32 | 故障位 |
| uptime_ms | uint32 | FPGA 上电毫秒数 |
| last_control_sequence | uint32 | 最近一次已接受的控制序号 |

`fault_bits` 在机器人网络 RTL 中冻结如下；位 `6..31` 保留为 `0`。锁存位只用于报告和安全诊断，不授权主机自动恢复：

| 位 | 名称 | 语义与清除条件 |
|---:|---|---|
| 0 | critical_fault | 严重故障输入的当前电平 |
| 1 | control_timeout | Control 看门狗曾超时；下一合法 Control 或复位清除 |
| 2 | imu_sample_error | IMU 原子样本曾不完整；复位清除 |
| 3 | ranges_invalid | 三路测距任一路当前无效 |
| 4 | rx_overflow | Control CDC FIFO 或辅助报文提交曾溢出；复位清除 |
| 5 | tx_backpressure | Telemetry/Status CDC FIFO 曾满；复位清除 |

初始端口分配：Jetson 到 FPGA 控制端口 `5000`，FPGA 到 Jetson 遥测/状态端口 `5001`。控制与遥测目标频率均为 100 Hz，状态目标频率为 10 Hz；图像使用独立端口和 FIFO，不得阻塞控制链路。

当前 RTL 的 Telemetry 和 Status 分别按系统时钟参数调度为 100 Hz 与 10 Hz；接受首个合法 Control 后还会各挂起一帧，便于尽快建立返回路径。挂起标志会在拥塞时合并周期事件，不累积无界队列；Status 在两者同时挂起时优先编码。当前 IPv4/UDP 发送端使用 UDP checksum `0`，接收端接受 `0` 或校验正确的非零 checksum，均符合 IPv4 UDP 规则。

#### 最小以太网控制面

- Control 只接受目标 MAC、目标 IPv4 和 UDP 端口均匹配的 IPv4 IHL=5、非分片报文，并检查以太网 FCS、IPv4 header checksum、UDP 长度与可选 UDP checksum。
- ARP responder 只处理以本机 IPv4 为目标的 Ethernet/IPv4 ARP Request。帧字段和发送边界参考官方第 52 章资料；项目只实现固定 responder，不扩展为通用 ARP cache。
- ICMP 只处理发往本机、IHL=5、非分片且 checksum 正确的 Echo Request，最多回显 64 字节 ICMP 报文。该最小 Echo Reply 是项目 RTL，不属于官方第 58 章 Ping 例程。

### 主机侧安全行为

- `fpga_gateway_node` 无论目标速度是否为零，都按 100 Hz 发送控制报文，供 FPGA 判断通信心跳。
- 超过 `command_timeout_ms` 未收到新的 `/cmd_vel` 时，网关发送零速度。
- 软件急停置位时立即清除缓存速度；解除急停后必须收到新的 `/cmd_vel` 才能恢复非零命令。
- `link_timeout_ms` 只控制 ROS 诊断中的在线判定。FPGA 的 PHY、报文心跳和 PWM 看门狗仍是独立的最终安全层。

### 实现与验证入口

C++17 逐字段编解码位于 `ros_ws/src/robot/fpga_gateway`，由 `ros_ws` 仓库统一维护；RTL 位于独立 `FPGA_ws` 仓库。本节线格式和黄金向量保留供旧 C++ 网关对照；早期差速工程命令不再列为当前入口。板级历史验证见[共享验证记录](validation_history.md)，现行主机入口见[网关 README](../ros_ws/src/robot/fpga_gateway/README.md)。

系统在物理急停、严重故障或离开 Jetson 模式时清除缓存速度；条件解除后须收到新的合法 Control 才能恢复协同运动。Control 在 FCS 和网络层检查完成后才提交，返回报文在应用编码完成后才提交；两个时钟域使用深度 4 的整记录异步 FIFO，复位异步置位，并分别在网络域和系统域两拍同步释放。

RTL 仿真和离线 wrapper 的内部 STA 不证明真实 PHY 断链、传感器单位换算或物理 PWM 关闭。实板联调必须重新验证这些行为。

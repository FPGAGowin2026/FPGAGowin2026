# FPGA 与 Jetson UDP 二进制协议

本协议用于 ACG720 FPGA 与 Jetson Orin Nano 之间的控制和状态通信。所有多字节整数均为网络字节序（大端）；不直接传输 C/C++ 结构体内存。

## 报文格式

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

## 消息类型

### Control（类型 1，payload 9 字节）

| 字段 | 类型 | 单位 | 说明 |
|---|---:|---|---|
| linear_mm_s | int32 | mm/s | 线速度命令 |
| angular_mrad_s | int32 | mrad/s | 角速度命令 |
| flags | uint8 | - | bit0：急停请求 |

### Telemetry（类型 2，payload 50 字节）

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

### Status（类型 3，payload 16 字节）

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

### 最小以太网控制面

- Control 只接受目标 MAC、目标 IPv4 和 UDP 端口均匹配的 IPv4 IHL=5、非分片报文，并检查以太网 FCS、IPv4 header checksum、UDP 长度与可选 UDP checksum。
- ARP responder 只处理以本机 IPv4 为目标的 Ethernet/IPv4 ARP Request。帧字段和发送边界参考官方第 52 章资料；项目只实现固定 responder，不扩展为通用 ARP cache。
- ICMP 只处理发往本机、IHL=5、非分片且 checksum 正确的 Echo Request，最多回显 64 字节 ICMP 报文。该最小 Echo Reply 是项目 RTL，不属于官方第 58 章 Ping 例程。

## 主机侧安全行为

- `fpga_gateway_node` 无论目标速度是否为零，都按 100 Hz 发送控制报文，供 FPGA 判断通信心跳。
- 超过 `command_timeout_ms` 未收到新的 `/cmd_vel` 时，网关发送零速度。
- 软件急停置位时立即清除缓存速度；解除急停后必须收到新的 `/cmd_vel` 才能恢复非零命令。
- `link_timeout_ms` 只控制 ROS 诊断中的在线判定。FPGA 的 PHY、报文心跳和 PWM 看门狗仍是独立的最终安全层。

## 实现与验证入口

C++17 逐字段编解码位于 `ros_ws/src/robot/fpga_gateway`，由 `ros_ws` 仓库统一维护；RTL 位于独立 `FPGA_ws` 仓库。协议黄金向量、模块与网络端到端测试结果统一见 [handoff](./handoff.md#51-rtl-仿真证据快照)，主机测试记录见该文第 5 节。

系统在物理急停、严重故障或离开 Jetson 模式时清除缓存速度；条件解除后须收到新的合法 Control 才能恢复协同运动。Control 在 FCS 和网络层检查完成后才提交，返回报文在应用编码完成后才提交；两个时钟域使用深度 4 的整记录异步 FIFO，复位异步置位，并分别在网络域和系统域两拍同步释放。

RTL 仿真和离线 wrapper 的内部 STA 不证明真实 PHY 断链、传感器单位换算或物理 PWM 关闭。实板联调必须重新验证这些行为。

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

`status_bits` 的位分配尚未冻结，当前主机模拟器发送 `0`；必须在 FPGA 传感器有效性和看门狗状态定义完成后再固化。

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

`fault_bits` 的位分配同样尚未冻结。未知位应保留并上报，不用于主机侧自动恢复。

初始端口分配：Jetson 到 FPGA 控制端口 `5000`，FPGA 到 Jetson 遥测/状态端口 `5001`。控制与遥测目标频率均为 100 Hz，状态目标频率为 10 Hz；图像使用独立端口和 FIFO，不得阻塞控制链路。

## 主机侧安全行为

- `fpga_gateway_node` 无论目标速度是否为零，都按 100 Hz 发送控制报文，供 FPGA 判断通信心跳。
- 超过 `command_timeout_ms` 未收到新的 `/cmd_vel` 时，网关发送零速度。
- 软件急停置位时立即清除缓存速度；解除急停后必须收到新的 `/cmd_vel` 才能恢复非零命令。
- `link_timeout_ms` 只控制 ROS 诊断中的在线判定。FPGA 的 PHY、报文心跳和 PWM 看门狗仍是独立的最终安全层。

## 当前实现与验证状态

- C++17 实现在 `ros_ws/src/robot/fpga_gateway`，逐字段编解码，不直接复制结构体内存。
- 4 项 GTest 覆盖黄金向量、三类报文往返、CRC/头部/长度拒绝和序号回绕，当前全部通过。
- `fake_fpga_node` 与 `fpga_gateway_node` 的本机 UDP 闭环达到约 100 Hz，并已验证速度命令、超时归零、软件急停和断链诊断。
- 尚未与 ACG720 实板通信；当前结果不能证明 RGMII 时序、FPGA CRC 实现、PHY 断链看门狗或 PWM 关闭。

# Jetson 通信与 ROS 接入

更新：2026-10-05。保留当前实现、操作与验证边界；已移除早期退役记录。

[返回项目总览](README.md)

[ROS 工作区与遥控](#ros-工作区与遥控) · [数据契约](#数据契约) · [BX72 双向 UDP 联调](#bx72-双向-udp-联调) · [BX72 传感器接入](#bx72-传感器接入) · [ACG720 传感器接入](#acg720-传感器接入)

## ROS 工作区与遥控

运行目标为 Jetson/Linux、ROS 2 Humble、C++17。源码位于 `ros_ws/src/robot/fpga_gateway`；launch 和测试保留 Python，Python 参考实现不安装为运行入口。当前 C++ 构建/回归与部署缺口见 [handoff](handoff.md#证据与交付缺口)。

### 构建与启动

新增构建依赖为 OpenCV（core、imgcodecs）和 nlohmann_json。可通过工作区的 rosdep 安装依赖，也可在 Ubuntu/Jetson 上安装 `libopencv-dev` 与 `nlohmann-json3-dev`。

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup
source install/setup.bash

# PC UDP 遥控、FPGA 速度 ACK、视频 JPEG 转发、轮速状态
ros2 launch robot_bringup bx72_teleop.launch.py \
  bind_fpga:=192.168.0.4 fpga_ip:=192.168.0.2

# 或直接运行，命令行选项保留原脚本名称
ros2 run fpga_gateway jetson_teleop_bridge \
  --bind-fpga 192.168.0.4 --fpga-ip 192.168.0.2 --pc-port 6200
```

只读记录器独立启动：

```bash
ros2 launch robot_bringup bx72_recording.launch.py \
  bind:=192.168.0.4 output:=/tmp/bx72-data duration:=30

# 或
ros2 run fpga_gateway jetson_telemetry_receiver \
  --bind 192.168.0.4 --source 192.168.0.2 --duration 30 --output /tmp/bx72-data
```

两种程序均支持 `--help`。程序是由 ROS 包安装与 launch 管理的 UDP 服务，不订阅 `/cmd_vel`，不发布 ROS 话题。PC 客户端沿用原 UDP JSON/BXJV 协议。

### 端口与报文

| 端口 | 默认地址/用途 |
| --- | --- |
| 6200 | Jetson `0.0.0.0`，PC 控制输入和状态/JPEG 输出 |
| 5000 | FPGA `192.168.0.2`，三轴速度请求及 FPGA 数据源端口 |
| 6103 | Jetson `192.168.0.4`，速度应答 |
| 6102 | Jetson `192.168.0.4`，800×480、大端 RGB565 RTP 视频 |
| 6104 | Jetson `192.168.0.4`，BX72 v1 IMU/motor/lidar/fusion/health 遥测 |

遥控网关独占 6102/6103/6104。启动前停止占用这些端口的 `bx72_sensors.launch.py`、`bx72_bidirectional.launch.py`、probe 或记录器。记录器独占 6102/6104，可与仅占用 6103 的诊断入口共存。

PC 控制 JSON 示例（UTF-8，一包一条；速度单位 mm/s、mrad/s）：

```json
{"v":1,"type":"control","session":"pc-session-1","seq":1,"vx":0,"vy":0,"wz":0}
```

同一 PC IP/端口和 session 按递增的 32 位序号发送。建立/恢复租约先发零速度，再发送运动命令；序号支持回绕。速度上限为 `|vx|/|vy|≤300`、`|wz|≤1000`。控制中断 250 ms 清零；FPGA 应答超时 100 ms 撤销租约，超过 200 ms 未收到应答则取消 ready。零速度 ACK 后才能下发运动速度。正常请求周期 50 ms、状态周期 100 ms；停车不等待旧运动 ACK。退出时发送三条递增序号零速度请求。

FPGA 控制为 `BX72` v1、类型 3、32 字节，大端字段顺序为 magic/version/type/length/sequence/session_token/vx/vy/wz/reserved/CRC16。应答类型为 4，其余字段必须与当前请求匹配，CRC16-CCITT 初值 `0xffff`。此格式与历史 `FPGA` 差速协议相互独立。网关 token 使用启动时 Unix 纳秒时间。

视频只提交长度正确、帧内序号连续的完整帧；JPEG 使用 `BXJV`、32 位 frame_id、16 位分片编号/总数及最多 1200 字节载荷。JPEG 最大 512 KiB。C++ 重组器忽略已交付帧的重复分片。

### 记录文件与整合范围

记录器保留 `telemetry.jsonl`、`video_frames.jsonl`、带大端长度前缀的 `telemetry_packets.bin`、`summary.json`、`latest.json`、首帧/最新帧 RGB565 与 PPM 和 `latest_frame.json`。图像转换和磁盘写入使用单独线程，最多保留一个待写帧，最新图像更新间隔为一秒；周期摘要每五秒写入，退出时刷新。

记录目录中的 `bridge.pid` 不表示当前进程，不能据此停止服务。

### C++ 运行入口

| 功能 | 当前入口 |
| --- | --- |
| 双向诊断 | `bx72_udp_node`、`bx72_udp_probe`、`bx72_udp_acceptance` |
| BX72 v1 只读 ROS | `bx72_round5_node`，实现于 `src/bx72_sensor_nodes.cpp` |
| ACG720 BXT1 只读 ROS/验收 | `bx72_sensor_node`、`bx72_sensor_acceptance`；解码库 `bxt1_protocol` |
| PC 遥控与视频转发 | `jetson_teleop_bridge` |
| 遥测/视频记录 | `jetson_telemetry_receiver`，BX72 v1 解码共用 `bx72_protocol` |

各入口沿用对应参数、话题与输出格式，CLI 使用 `ros2 run fpga_gateway <入口>`。诊断仅发送 type=1 请求；ACG720 `video-*` 模式只接收。ROS 接收入口不发布 odom/TF，也不订阅运动命令。部署使用新安装目录，避免旧 `.py` 入口残留。

## 数据契约

更新：2026-10-05。当前主线为 BX72＋Jetson，实际 ROS 包名为 `fpga_gateway`。当前源码已包含三轴速度命令与 WASD 网关；本文维护现行 BX72/BXT1 协议与后续扩展边界。

### 当前 BX72 协议入口

| 通道 | 端口与状态 | 实际定义 |
| --- | --- | --- |
| RTP/UDP 视频与状态 | Jetson 6102；已有视频共存实板验收 | [板级视频说明](../FPGA_ws/boards/BX72_138K_robot/docs/video_jetson.md) |
| 双向诊断 | FPGA 5000 下行、Jetson 6103 应答；type=1/2 诊断；type=3/4 为速度请求/入队回执 | [BX72_BIDIRECTIONAL](#bx72-双向-udp-联调) |
| BX72 v1 五类遥测 | Jetson 6104；IMU、电机/里程计、N10、融合、健康五类；远端有短窗口板测 | [板级线格式](../FPGA_ws/boards/BX72_138K_robot/docs/jetson_udp_telemetry.md)、[ROS 接入](#bx72-传感器接入) |
| 四轮运动命令与反馈 | FPGA 5000 / Jetson 6103；32 字节 BX72 v1 type=3/4，已有源码与远端悬空板测记录 | [遥控操作与协议](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)及[开发计划](handoff.md#后续开发计划) |

BX72 v1 分别记录物理 CH0～CH3、逻辑 A/B/C/D、目标/实测轮速与融合状态；轮速绝对尺度和定位精度仍需实测。ROS Header 当前为接收时间，自定义 timestamp_us 为 FPGA 采集提交时间；未完成时钟映射。传感器正确性、时间映射与里程计发布需分别验收。

当前速度协议采用大端 32 字节报文，包含会话 token、序号、vx/vy（mm/s）、wz（mrad/s）与 CRC16；会话以零速包建立，重复/乱序及旧会话被拒绝。FPGA 使用 300 ms 本地看门狗；type=4 回执表示入队，不表示机械动作已完成。详细偏移、范围与桥接行为以[遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)和 RTL 为准。旧差速 C++ 网关不能直接发送此协议。测速尺度、落地负载、ROS /cmd_vel 接入及更完整的数据年龄/故障语义仍待验证或开发。

### 已实现的三轴速度线格式

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

### 规划中的模块契约

| 对象 | 必需语义 |
|---|---|
| 运动请求 | 车体系 vx/vy/wz、单位、来源、会话/启动标识、序号、有效期及新鲜度依据 |
| 本体状态 | 四轮轮速与轮序、车体速度、输入来源、估计对齐时间、有效性、数据年龄、质量 |
| 环境约束 | 相对车体外形的方向间隙、方向覆盖、观测时间、有效性、连续性、TTC 及可信度 |
| 安全结果 | 请求与最终允许指令、安全模式、原因码、并发故障、输出许可、最近接受序号 |
| LiDAR 上传 | 扫描/分包标识、角度与距离、采样时序依据、校验和缺失状态 |

坐标采用 x 前、y 左、z 上，wz 为绕 z 正向；协议统一明确单位与字节序。接收、采样、估计时间分别表达，未知值携带无效原因。上表新增环境约束/故障结果等完整线格式尚未冻结；当前速度包和五类遥测已经有各自定义，不得直接扩字段并视为兼容。

### 后续指令许可与故障语义要求

- 重复、乱序、过期、错误或非授权来源指令均不得刷新运动许可。序号更新与刚收到不等于生成时间新鲜。
- 启动/会话标识区分重启前后报文。有效期必须有可判定时间依据；可采用 FPGA 本地有界许可窗口，窗口不得被延迟旧包延长。当前实现为会话/序号检查和本地 300 ms 租约；更完整生成时间新鲜度证明仍待扩展。
- Jetson 桥接检查上游请求新鲜度，不能给冻结的旧速度反复更换序号；心跳不能替代运动许可。
- 无效观测不能当作零速度或无障碍；无有限 TTC 与 TTC 未知分别编码。
- 解除急停或锁存故障后，需满足恢复条件、重新使能和新鲜命令，不自动恢复缓存运动。
- 本地 Watchdog、仲裁和输出门控仅依赖本地有效信息，不依赖跨机时间映射或上传成功。

### 传输约束

链路采用 RGMII、MAC、IPv4、UDP 与自定义二进制协议。完整报文通过规定校验后才原子提交；明确 MAC FCS、IP/UDP 及应用完整性校验的责任。固定端点可采用静态地址与必要 ARP，不扩展为通用网络栈。

控制/状态与 LiDAR 上传使用有界队列及明确优先级；上传拥塞不得阻塞本地保护。明确 MTU、分包、丢包、队列满和不支持报文处理。实时请求优先最新有效值，频率与超时需依据延迟预算确定。

## BX72 双向 UDP 联调

当前工具与 ROS 节点已改为 C++17，运行命令已更新；本次未编译或测试。当前入口见 [C++ 运行入口](#c-运行入口)。

诊断对象为 `FPGA_ws/boards/BX72_138K_robot`。FPGA 接收 Jetson 的诊断请求，校验后回显序号、token 和 10 字节数据；Jetson 只有收到匹配应答才报告双向在线。此协议不携带运动指令，不能用旧差速 `/cmd_vel` 报文驱动当前 BX72 四轮底盘。

### 默认端点

| 用途 | 源 | 目的 |
| --- | --- | --- |
| 现有视频/状态 | FPGA `192.168.0.2:5000` | Jetson `192.168.0.4:6102` |
| 新诊断请求 | Jetson `192.168.0.4:6103` | FPGA `192.168.0.2:5000` |
| 新诊断应答 | FPGA `192.168.0.2:5000` | Jetson `192.168.0.4:6103` |

FPGA 返回帧沿用视频的广播目的 MAC，IP 指向 Jetson。FPGA RX 检查目标 MAC/IP/端口以及请求源 IP/端口。硬件参数默认固定，主机参数必须与其一致；仅修改 launch 参数不能修改 FPGA。FPGA 没有 ARP/ping responder，Jetson 发单播前必须设置静态邻居。默认 FPGA MAC 为 `00:0a:35:01:fe:c0`。

### 32 字节报文

所有多字节整数均为大端。此格式独立于旧 `FPGA` 前缀差速协议和 `ACG6` 视频状态。

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 4 | ASCII `BX72` |
| 4 | 1 | version=`1` |
| 5 | 1 | type：`1` 请求，`2` 应答 |
| 6 | 2 | 总长度=`32` |
| 8 | 4 | sequence：请求序号，应答原样回显 |
| 12 | 8 | token：每次请求的随机值，应答原样回显 |
| 20 | 10 | data：任意回显数据 |
| 30 | 2 | CRC16-CCITT-FALSE，覆盖前 30 字节；初值 `ffff`，多项式 `1021`，不反射，无最终异或 |

FPGA 应答修改 type 并重新计算应用 CRC。Ethernet FCS、IPv4 checksum、UDP 长度和非零 UDP checksum 必须合法；IPv4 UDP checksum 为零时允许接收。仅支持 IPv4 IHL=5、非分片 UDP。队列满时丢弃新请求，主机通过超时发现。

黄金请求：sequence=`01020304`，token=`0102030405060708`，data=`00..09`：

```text
42 58 37 32 01 01 00 20 01 02 03 04 01 02 03 04
05 06 07 08 00 01 02 03 04 05 06 07 08 09 56 E5
```

### 实板顺序

1. 先按[项目状态](handoff.md#项目状态与交接)与[板级 README](../FPGA_ws/boards/BX72_138K_robot/README.md)核对候选、构建及部署。加载候选后须独立复验。SRAM 下载断电失效。
2. 按已有视频文档完成 Jetson 有线地址配置。确认连接 FPGA 的接口是 `enP8p1s0`，然后增加静态邻居：

```bash
sudo ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 nud permanent dev enP8p1s0
ip neigh show dev enP8p1s0
```

3. 在 Jetson 构建并加载当前 ROS 工作区后运行 C++ 探测工具。工具本身使用原生 UDP，不需要启动 ROS 节点：

```bash
ros2 run fpga_gateway bx72_udp_probe \
  --bind-ip 192.168.0.4 --duration 30
```

默认 10 Hz，每次只保留一个请求，300 ms 超时。应看到 TX/ACK 一起增加并出现 `PASS`；超时或发送失败会返回非零退出码。可同时运行原视频接收，确认完整帧率与序号连续性。独立工具与 ROS 节点使用同一应答端口，只能选择一个运行。

4. ROS 2 集成（在 Jetson 的 `ros_ws` 目录）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup
source install/setup.bash
ros2 launch robot_bringup bx72_bidirectional.launch.py bind_ip:=192.168.0.4
```

在另一个已加载环境的终端查看：

```bash
ros2 topic echo /fpga/bidirectional_status
```

`bidirectional=True` 表示近期收到正确回显；`sent` 仅表示本机成功发送，不能证明 FPGA 收到。拔线或停止应答后状态应变为 ERROR；恢复后新的正确应答恢复 OK。

### 视频共存与拔线恢复验收

停止其他占用 UDP `6102/6103` 的视频预览或 ROS 接收节点。先设置静态邻居，再运行C++ 验收工具，它自己接收并检查 800×480 RGB565 视频，无需图形窗口。

```bash
sudo ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 nud permanent dev enP8p1s0
ros2 run fpga_gateway bx72_udp_acceptance \
  --mode coexist --bind-ip 192.168.0.4 --duration 30 --output /tmp/bx72_coexist.json
```

工具先记录 5 秒视频基线，再连续发送 30 秒诊断请求。要求请求全部匹配应答，超时/无效包/发送失败为零；视频有完整帧且没有不完整帧或序号缺口，完整帧率不少于基线的 85%。输出 JSON 和 `PASS/FAIL`，不能用单纯收到视频包代替完整帧验收。

```bash
ros2 run fpga_gateway bx72_udp_acceptance \
  --mode recovery --bind-ip 192.168.0.4 --duration 90 --output /tmp/bx72_recovery.json
```

按提示拔开 FPGA 网线，等待“已确认断链”后接回。程序要求应答与视频都停止，然后都恢复；恢复后再检查独立的 5 秒稳定窗口。断链期间的预期超时保留在总计数中，不作为稳定阶段失败。若链路重建清掉了静态邻居，在另一终端重新执行上述 `ip neigh replace`；验收不要求重新烧录或重启 FPGA。物理拔线动作由操作者执行，本机模拟暂停流量不能作为实板拔线结果。

## BX72 传感器接入

协议为 ASCII BX72、版本 1、32 位微秒时间戳，包含 IMU、电机、N10、融合和健康五类。完整字段与轮位标定见 [板级接口](../FPGA_ws/boards/BX72_138K_robot/docs/jetson_udp_telemetry.md)，源码/位流边界见 [统一记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md)。

### C++ 接收

在 Jetson 构建并加载本 ROS 工作区后，执行 C++ 记录器：

```bash
ros2 run fpga_gateway jetson_telemetry_receiver --output runs/round5
```

接收器同时占用视频 6102 和遥测 6104，保存原始包、telemetry.jsonl、latest.json、summary.json 与图像。独立启动 ROS 前停止此接收器；不能让两个进程争用 6104。

### ROS 2 Humble

同步当前 fpga_gateway 和 robot_bringup（含 msg 和 C++ 源码）到实际 ROS 工作区，执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select fpga_gateway robot_bringup
source install/setup.bash
ros2 launch robot_bringup bx72_sensors.launch.py bind_ip:=192.168.0.4
```

默认使用 C++ `bx72_round5_node`，仅接收 6104；可选 C++ 诊断节点只发送已有回显请求。没有运动订阅或运动指令。

| 话题 | 语义 |
|---|---|
| /imu/data_raw、/fpga/imu_raw | 传感器坐标原始六轴与名义 SI 换算；姿态未知，不虚构已标定协方差 |
| /fpga/n10_packets | 原始 16 点组，不是完整扫描；本类没有 UART framing 计数，framing_errors_available=false |
| /fpga/encoder_counts | 物理 CH0..3 累计计数；非法跳变掩码从逻辑轮序转换到物理通道 |
| /fpga/round5/imu、motor、lidar、fusion、health | std_msgs/String JSON，完整保留实际解码器的全部字段；包括目标/实测轮速及融合状态 |
| /fpga/sensor_status | DiagnosticArray，接收量、数据年龄和协议错误；新窗口/进程用于区分累计历史与当前错误 |

ROS Header 是主机接收时间。timestamp_us 是 FPGA 包快照时间，32 位自然回绕（约 71.6 分钟）；没有跨机时钟映射。EncoderCounts.interval_us 是相邻收到电机包的时间差，首包为 0；丢包时不是固定采样周期。ready 表示收到了完整电机快照，不是独立编码器 ready 硬件位。

原始计数没有空间坐标；轮速绝对尺度仍待每圈计数和有效轮径标定。融合 JSON 仅用于调试，本入口不发布 odom/TF 或 LaserScan。IMU 轴向、零偏、N10 距离与安装外参需物理核对。

BX72 使用 BX72 v1 解码器，ACG720 使用 BXT1 解码器；两者不能互换。部署前按 [handoff](handoff.md) 核对位流与接收器。

## ACG720 传感器接入

2026-10-05；适用于 `FPGA_ws/boards/ACG720_60K_robot`。线协议为 BXT1 v1，详见 [完整字段定义](../FPGA_ws/boards/ACG720_60K_robot/docs/sensor_telemetry.md) 和 [本板验证记录](../FPGA_ws/boards/ACG720_60K_robot/docs/sensor_telemetry_validation.md)。复用 `bxt1_protocol`、`bx72_sensor_node` 和现有三个自定义消息，文件/消息中的历史 BX72 名称不限制板型；不另造一套解码器。

ACG720 网络目前只有 TX，向 Jetson `.4` 发送源 `.2:5000` 的视频/状态 6102 和传感器 6104，没有 6103 诊断、ARP 或 ping responder。默认 IP/MAC 与 BX72 相同，只启用当前测试板；切换板型不代表测量轴、轮位和标定可复用。ACG720 的 TFT 左侧相机/右侧雷达显示保留，网络视频仍为完整相机图像。

### Jetson C++ 验收

停止占用 6102/6104 的程序。在 Jetson 构建当前 ROS 工作区并加载 `install/setup.bash` 后执行；以下命令仅接收 UDP，不发送命令，结果 JSON 与原始 JSONL 均保留。此次只改写工具，未执行以下验收命令。

```bash
ros2 run fpga_gateway bx72_sensor_acceptance --mode telemetry --duration 10 \
  --bind-ip 192.168.0.4 --output acg720_telemetry.json
ros2 run fpga_gateway bx72_sensor_acceptance --mode video-telemetry --duration 30 \
  --bind-ip 192.168.0.4 --minimum-fps 26 --output acg720_coexist.json
ros2 run fpga_gateway bx72_sensor_acceptance --mode video-recovery --duration 90 \
  --bind-ip 192.168.0.4 --minimum-fps 26 --recovery-method automatic \
  --output acg720_recovery.json
```

恢复测试按提示拔线，确认遥测和视频都停止，再接回；禁止在 automatic 测试期间刷新邻居。若需要人工刷新，另跑并设置 `--recovery-method manual-neighbor`，不能宣称自动恢复。工具无法检测其他终端中的邻居操作。

默认频率门限为 IMU 100～140 Hz、编码器 45～55 Hz、健康 0.7～1.3 Hz、N10 有效 16 点分组 50～410 Hz；N10 实际频率尚须实测，不是扫描帧率。视频至少 26 fps 且不低于当前板基线的 85%；门限需随本板实测配置核对，不能引用其他板卡的历史帧率。稳定阶段检查序号、CRC、时间、采集器错误、源丢弃、完整视频帧和过渡排空；接收端源 IP/端口必须匹配。

`transport_passed=true` 只证明该窗口的传输检查通过，工具始终输出 `sensor_correctness_proven=false`。另存无电机动作的物理记录：静止重力模长/陀螺偏置、手动旋转的轴与符号、已知距离物体的 N10 测距、关闭电机电源后逐 CH 手转正反计数和完整圈数。N10 零距离保留为无有效测距，不作为近距离障碍；16 点包不是完整扫描。

### Jetson ROS 2

在工作区构建并启动本板专用 launch；C++ 验收结束后再启动，避免争用端口。

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fpga_gateway robot_bringup
source install/setup.bash
ros2 launch robot_bringup acg720_sensors.launch.py bind_ip:=192.168.0.4
```

该 launch 只启动传感器节点，不启动 `bx72_udp_node` 或旧 `fpga_gateway_node`，没有 `/cmd_vel` 等运动订阅，也不发送 UDP。节点名 `acg720_sensor_node`，状态前缀 `acg720_60k`。话题仍为 `/imu/data_raw`、`/fpga/imu_raw`、`/fpga/n10_packets`、`/fpga/encoder_counts`、`/fpga/sensor_status`；不要与 BX72 节点同时占用端口/同名话题。

IMU 转换至 m/s²、rad/s，使用 `acg720_imu660rc_sensor` 芯片轴坐标；无安装变换、姿态或标定协方差。N10 使用 `acg720_n10_sensor` 原解析器角度，未知外参不生成 `/scan`；编码器始终为物理 CH0～3，空 frame_id，没有 ticks/rev 或轮位假设。ROS Header 为接收时刻，FPGA 名义微秒采集提交时间单独保留；尚无跨机器同步映射。

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /fpga/n10_packets --once
ros2 topic echo /fpga/encoder_counts --once
ros2 topic echo /fpga/sensor_status --once
ros2 node info /acg720_sensor_node
```

C++ 构建/测试、真实传感器及 Jetson 跨进程发布订阅均须单独验收；仅有 launch 启动或同进程测试不代表独立 ROS 图通过。当前交付与验证边界见 [handoff](handoff.md#证据与交付缺口)。

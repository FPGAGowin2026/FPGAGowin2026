# 项目交接文档

最后更新：2026-08-23

## 1. 接手顺序

开始工作前依次阅读：

1. [`AGENTS.md`](../AGENTS.md)：项目处于研究原型/工程验证模式，优先正确性、可调试性和开发速度。
2. [`赛题.md`](./赛题.md)：选题 3 原始要求和评分权重。
3. [`系统总体方案.md`](./系统总体方案.md)：当前唯一总体方案，本文不替代它。
4. [`项目完成度报告.md`](./项目完成度报告.md)：按最终现场演示门槛估算当前完成度、剩余工作和关键路径。

## 2. 项目目标

在 2026 年 11 月前完成可稳定现场演示的具身机器人：

- ACG720 独立完成摄像头/姿态显示、差速底盘闭环、测距减速停车、局部绕障和预设路径执行。
- Jetson Orin Nano 完成 ROS 2、SLAM、Nav2、视觉 AI 和智能任务。
- FPGA 始终保留物理急停、限幅、减速、停车和看门狗等最终安全裁决。

## 3. 已确认基线

| 项目 | 当前决定 |
|---|---|
| FPGA 板卡 | 小梅哥 ACG720 |
| FPGA 器件 | `GW5AT-LV138PG484AC1` |
| 边缘计算 | Jetson Orin Nano（现有文档记录组委会已确认可用） |
| Jetson 软件 | JetPack 6.2.x、Ubuntu 22.04、ROS 2 Humble |
| 底盘 | 两轮差速 + AB 相编码器 + 万向轮 |
| 视觉 | OV5640 DVP 采集，DDR3 帧缓存，HDMI 显示 |
| 感知 | IMU、2D 激光雷达、前/左/右三路 TOF 或超声波 |
| FPGA—Jetson | 千兆以太网 UDP，UART 仅调试 |
| FPGA 任务管理 | 首版使用纯 RTL 运动段队列，不引入 PicoRV32 |

实际电机、驱动器、IMU、测距模块、激光雷达型号及引脚还未在仓库中固化，实施前必须核对电平、电流、编码器分辨率和板卡约束。

## 4. 当前最新决策：模式隔离

任一时刻只允许一个上层运动指令源。

FPGA 独立模式：

```text
物理急停
> 严重故障
> FPGA 障碍物安全限制
> FPGA 绕障控制
> FPGA 独立路径控制
> 默认停车
```

Jetson 协同模式：

```text
物理急停
> 严重故障
> FPGA 障碍物安全限制
> Jetson 运动命令
> 默认停车
```

实施时必须保证：

- 独立模式忽略 Jetson 运动命令。
- 协同模式禁用 FPGA 独立路径和主动绕障指令源。
- 协同模式中 Nav2 选择绕行方向；FPGA 只限幅、减速或置零。
- 模式只能在复位或电机已停止时切换。
- 切换时先清零运动目标，再启用新模式的唯一指令源。
- 物理急停、严重故障和安全停车不受模式选择影响。

## 5. 当前仓库状态

- 已建立 `ros_ws/src/robot` 和 `ros_ws/src/external`；`external` 保存不写入系统目录的 ROS 2 Humble 二进制依赖覆盖层，通过 `Scripts/install_ros_dependencies.sh` 安装，并由 `Scripts/setup_ros_dependencies.bash` 激活。该目录整体忽略，不上传到 Git。
- `fpga_gateway` 已作为独立私有仓库发布，包含 C++17 固定二进制协议、POSIX UDP 网关、主机侧 `fake_fpga_node` 和 4 项 GTest。
- `robot_bringup` 已作为独立私有仓库发布，包含仿真与硬件两套 launch、集中 YAML 参数，以及一项自动化 ROS 主机闭环 launch 测试。
- 父仓库新增 `robot_navigation`、`vision_bridge` 和 `mission_control` 三个 C++17/`rclcpp` 原型包：分别提供 EKF/SLAM/Nav2、红色单目标观测，以及“到搜索区—搜索—对准—接近—停车”状态机。Python 只保留给 ROS 2 launch；原型稳定并需要独立发布时再拆仓库。
- 当前网关发布版本增加了轮式里程计与 IMU 的非零协方差参数，`robot_bringup` YAML 提供对应初值；这些值只用于主机融合起步，必须用真实静止、直行和定点旋转数据重新标定。
- [`fpga_rtl`](https://github.com/FPGAGowin2026/fpga_rtl) 已作为独立私有仓库发布，包含 Verilog-2001 模式仲裁器和一项自检 testbench；运行 `Scripts/sync_fpga_workspace.sh` 可将其首次克隆或快进更新到 `FPGA_ws/`。
- 父仓库不重复保存三个已发布独立仓库的源码；运行 `Scripts/sync_ros_packages.sh` 可将两个 ROS 2 通信包首次克隆或快进更新到 `ros_ws/src/robot/`。三个高级功能原型包当前由父仓库直接保存。
- 协议源文件为 [`udp_protocol.md`](./udp_protocol.md)，当前只定义控制、综合遥测和状态三类报文，不含参数或轨迹段上传。
- `FPGA_ws` 当前是 `fpga_rtl` 的独立检出。从父仓库干净克隆后通过同步脚本获取该仓库，Verilog-2001 编译和 15 项模式隔离与安全置零自检均通过；尚未接入 ACG720 顶层工程。
- `build/`、`install/`、`log/`、Python 缓存、`src/external/` 和两个独立通信包目录已由 `ros_ws/.gitignore` 排除，不进入父仓库提交。
- 已从 ACG720 官方第 58 章工程导入 RGMII/GMII、IPv4/UDP 收发和 UDP 回环 RTL 到 `FPGA_ws/src/ethernet/`，并新增 `FPGA_ws/sim/tb_eth_udp_loopback_rgmii.sv` 自检 testbench。导入只规范了文本格式，未改变官方 RTL 行为。
- 已有记录表明 72 字节有效帧回环检查通过，覆盖端点交换、8 字节载荷、填充和 FCS；坏 FCS 帧会被标记为错误但仍触发发送，拒绝路径尚未闭环。该 testbench 使用行为级 PLL/FIFO 模型和官方 GW5A DDR 原语，只是 RTL 仿真证据。本轮环境缺少 `iverilog`/`vvp`，未能现场重复这些结果。
- 当前 `fpga_gateway` 与 `robot_bringup` 发布版本联合验证通过：4/4 协议 GTest 和一项自动化 launch 测试均通过。launch 测试覆盖约 100 Hz 遥测、速度命令、200 ms 命令超时归零、软件急停、急停释放后的安全保持与恢复、模拟 FPGA 进程退出后的断链诊断。
- launch 测试使用独立 ROS domain 和 Fast DDS 同机共享内存传输，只用于隔离主机测试环境，不是实机部署配置，也不证明以太网链路行为。
- 2026-08-23 已用独立 `/tmp` 构建目录复验主机侧 5 个 ROS 2 包：全部构建成功，4 项协议 GTest、1 项 UDP launch 测试和 3 项 C++ 算法 GTest 共 8 个直接用例通过。`colcon test-result` 因同时汇总 CTest 包装结果和底层 XML 显示 `13 tests, 0 errors, 0 failures`，不能把它解释为 13 个独立测试用例。
- 同轮完整主机仿真现场复现 `/odometry/filtered`、`120×80 @ 0.05 m/pix` 地图、Nav2 从 `(0, 0)` 到 `(1, 0)` 成功，以及任务状态 `WAITING_NAV → NAVIGATING → SEARCHING → TRACKING → COMPLETE`；SIGINT 后所有进程正常退出。
- `robot_localization`、`slam_toolbox`、Nav2 及其运行时闭包已安装到 `ros_ws/src/external/ros_humble_overlay`，不依赖系统级安装；构建和运行前必须先加载 `Scripts/setup_ros_dependencies.bash`。视觉只验证了合成相机上的红色阈值基线，尚未接入 TensorRT、真实相机或训练模型。
- 当前已有独立模式仲裁器和官方 UDP 回环 RTL 仿真底座，但尚无本项目应用层协议解析、CRC16、序号检查和遥测 RTL，也没有完整 FPGA 顶层、板卡约束、电机或传感器 RTL；尚未运行综合、布线、FPGA 实板 UDP、电机、真实传感器、实车 SLAM/Nav2 或视觉 AI 验证。HDL 与主机侧仿真结果不得表述为 FPGA 看门狗、PWM 关闭或实车安全证明。

### 5.1 Git 发布边界

- `fpga_rtl` 的 `main` 已发布到 `9107a6b`，包含模式仲裁器、官方 UDP 回环 RTL 和自检 testbench。
- `fpga_gateway` 的 `main` 已发布到 `88f760c`，`robot_bringup` 的 `main` 已发布到 `c188c99`。
- 父仓库直接发布 `robot_navigation`、`vision_bridge`、`mission_control`、依赖脚本和项目文档；两个通信包仍由同步脚本从独立仓库获取。
- `ros_ws/src/external/` 是安装脚本在本机重建的依赖覆盖层，整体忽略且不上传；可上传的环境入口为 `Scripts/setup_ros_dependencies.bash`。

## 6. 下一步

ROS 主机通信基线、自动回归闭环和独立模式仲裁器 HDL 基线已经完成。当前只推进一个最小闭环：**FPGA 网络基线**。

1. 补齐 Icarus Verilog 仿真环境并重复有效帧与坏 FCS 用例；修复时必须把数据 FIFO 和报文元数据作为同一接收事务提交，不能只用 `rx_pkt_err` 门控报文 FIFO。
2. 使用 Gowin 工具完成官方 UDP 回环工程的综合与约束检查，再上板跑通 RGMII、ARP、Ping 和 UDP 回环。
3. 保持电机驱动禁用，按 [`udp_protocol.md`](./udp_protocol.md) 的黄金报文实现并联调本项目 FPGA 协议 RTL，验证大端字节序、长度、CRC16、序号、100 Hz 控制与遥测以及 10 Hz 状态，并保存抓包、频率和异常报文丢弃证据。

`robot_navigation`、`vision_bridge` 和 `mission_control` 的主机仿真基线已完成，当前不继续扩展算法或引入模型；网络传图和电机控制仍按硬件关键路径推进。当前下一最小闭环是坏 FCS 拒绝路径及其可重复仿真，随后进入 Gowin 工程和实板网络验证。仲裁器会在真实顶层接口确定后再完成输入同步、停止判定、综合和实板集成。

进入电机控制前，必须先把模式仲裁器接入真实顶层，并完成 Gowin 综合和实板安全验证；独立模块仿真通过不能替代这道门槛。

## 7. 未决项

在实现对应模块前再决定，不提前建抽象：

- 模式来自板上开关、外部按键还是配置寄存。
- “电机已停止”使用目标速度为零、编码器实测速度为零，还是两者同时满足。
- 安全层对 Jetson 速度命令是分量夹紧还是统一缩放。
- FPGA 独立路径和绕障内部定点格式。Jetson 速度命令已确定为 `int32` 的 mm/s 与 mrad/s。
- 遥测 `status_bits` 和状态 `fault_bits` 的具体位分配；当前主机模拟器发送零值。
- 实际引脚约束、时钟频率和复位极性。

## 8. 安全与实施边界

- 未完成仲裁器顶层集成以及急停、限幅和看门狗验证前，不得在车轮接地状态下启用电机。
- 实车首次闭环测试必须架空车轮并保证物理急停可直接切断驱动使能。
- 不从零重写通用网络栈，优先复用 ACG720 官方 RGMII/GMII、ARP、IPv4 和 UDP 模块。
- 不在首版引入 FPGA 内 SLAM、Nav2、DDS、micro-ROS、TCP、DHCP 或未证明必要的 PicoRV32。
- 硬件阈值、轮径、轮距、电机死区和控制器参数必须集中可标定，不在 RTL 中分散硬编码。

## 9. 验收基线

总体方案定义的最终门槛：

- 控制和遥测达到 100 Hz，不应用倒序或过期指令。
- 断开网线后在设定看门狗窗口内关闭 PWM。
- 图像满负载不阻断控制通道。
- FPGA 独立路线至少连续 30 次演示成功。
- 直行、旋转、停车距离和绕障参数经实车标定。
- 完整系统连续运行 1～2 小时无失控或异常复位。

任何局部仿真、离线测试或单模块上板结果都不等于实车、长时运行或现场演示证明。

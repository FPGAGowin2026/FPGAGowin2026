# 项目交接文档

最后更新：2026-08-24

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
- `FPGA_ws` 当前是 `fpga_rtl` 的独立检出。从父仓库干净克隆后通过同步脚本获取该仓库；`acg720_udp_loopback.gprj` 保留官方回环基线，新增 `acg720_robot_network.gprj` 引用机器人网络集成顶层和全部 RTL 依赖。器件仍为 `GW5AT-LV138PG484AC1/I0`；机器人工程只复用已知 RGMII 约束，不猜测系统时钟、电机、编码器、IMU、测距或急停引脚。
- 本机商业版 `Gowin_V1.9.12.03` 与 License 位于父仓库忽略的 `GoWin/` 目录；`gwlicense.ini` 已指向本地 License。2026-08-24 已在可读取本机网卡标识的环境完成官方 UDP 回环和机器人网络顶层综合。受限沙箱无法访问 `AF_NETLINK` 时会误报 hostid 不匹配；这不是 License 本身失效。
- `build/`、`install/`、`log/`、Python 缓存、`src/external/` 和两个独立通信包目录已由 `ros_ws/.gitignore` 排除，不进入父仓库提交。
- 已从 ACG720 官方第 58 章工程导入 RGMII/GMII、IPv4/UDP 收发和 UDP 回环 RTL 到 `FPGA_ws/src/ethernet/`，并在此基线上增加帧原子提交、IPv4/UDP 校验和检查和机器人网络连接。第 52 章资料只作为 ARP 帧字段与发送边界参考；最小 responder 判定由本项目实现。ICMP Echo Reply 也是项目 RTL，不能宣称来自不含完整 Ping 的第 58 章。
- 官方 UDP 回环已改为帧感知负载缓存：负载在 FCS 判定前只暂存，合法帧才与元数据原子提交，坏 FCS、负载区满或描述符 FIFO 满均回滚整帧写指针。自检按“合法帧—坏 FCS—描述符 FIFO Full 信号注入—不同负载合法帧”顺序验证 72 字节逐字节回环、写指针回滚、拒绝帧不发送和下一帧不受污染；真实 FIFO 计数填满与读写竞争仍留待 Gowin FIFO/板级验证。
- `acg720_robot_network_rgmii` 已将合法 IPv4/UDP Control 通过深度 4 整帧 CDC FIFO 送入 `robot_rtl_core`，再将 72 字节 Telemetry 与 38 字节 Status 编码为完整 UDP/IPv4/以太网帧。网络域和系统域分别同步释放复位；返回 CDC FIFO、固定 100 Hz/10 Hz 调度和单发送仲裁器已接入。电机 PWM/方向端口在集成顶层保持未连接。
- 已实现最小 ARP responder 和最多 64 字节 ICMP Echo Reply；不包含 ARP cache、IPv4 options、分片重组、ICMP 错误报文、DHCP 或通用网络框架。
- 已新增 OV5640 DVP RGB565 组帧、IMU 六轴字节流原子提交和前/左/右三路测距偏置/量程/超时监视。具体摄像头寄存器配置、I2C/SPI/TOF 协议、CDC、DDR3 和 HDMI 仍只保留板级接入边界。
- 已新增 AB 相正交解码、轮速估计、差速运动学、双轮 PI/PWM 目标，以及测距减速/停车、左右择向局部绕障、`MOVE/ROTATE/STOP` 路径执行和四段演示 ROM。所有增益、轮距、每脉冲距离、阈值和绕障节拍只是仿真初值。
- `robot_rtl_core` 已把协议、传感器、独立路径、Jetson 安全限制、模式仲裁和电机控制接成系统级仿真骨架。内部演示路径可完整走到 `FINISHED`，绕障期间通过 `path_progress_enable_o` 冻结并恢复当前段；离开 FPGA 模式、物理急停或严重故障会中止路径并清除缓存运动目标，恢复后必须收到新命令。当前进度仍由 testbench 按 `path_segment_opcode_o` 注入，尚不是编码器/IMU 实测闭环。
- 当前 `fpga_gateway` 与 `robot_bringup` 发布版本联合验证通过：4/4 协议 GTest 和一项自动化 launch 测试均通过。launch 测试覆盖约 100 Hz 遥测、速度命令、200 ms 命令超时归零、软件急停、急停释放后的安全保持与恢复、模拟 FPGA 进程退出后的断链诊断。
- launch 测试使用独立 ROS domain 和 Fast DDS 同机共享内存传输，只用于隔离主机测试环境，不是实机部署配置，也不证明以太网链路行为。
- 2026-08-23 已用独立 `/tmp` 构建目录复验主机侧 5 个 ROS 2 包：全部构建成功，4 项协议 GTest、1 项 UDP launch 测试和 3 项 C++ 算法 GTest 共 8 个直接用例通过。`colcon test-result` 因同时汇总 CTest 包装结果和底层 XML 显示 `13 tests, 0 errors, 0 failures`，不能把它解释为 13 个独立测试用例。
- 同轮完整主机仿真现场复现 `/odometry/filtered`、`120×80 @ 0.05 m/pix` 地图、Nav2 从 `(0, 0)` 到 `(1, 0)` 成功，以及任务状态 `WAITING_NAV → NAVIGATING → SEARCHING → TRACKING → COMPLETE`；SIGINT 后所有进程正常退出。
- `robot_localization`、`slam_toolbox`、Nav2 及其运行时闭包已安装到 `ros_ws/src/external/ros_humble_overlay`，不依赖系统级安装；构建和运行前必须先加载 `Scripts/setup_ros_dependencies.bash`。视觉只验证了合成相机上的红色阈值基线，尚未接入 TensorRT、真实相机或训练模型。
- `FPGA_ws/Scripts/run_all_sim.sh` 统一运行 10 个 RTL 功能自检和 1 个 Gowin PLL/RGMII 板级展开检查，最近一次完整回归均以零状态退出；端到端 testbench 为 72 项，异步帧 FIFO 为 23 项。当前本机没有系统级 `iverilog`/`vvp`，本轮使用 `/tmp/fpga_gowin_iverilog` 临时工具链；该目录不是仓库资产。
- 官方回环、机器人逻辑顶层和 14-I/O 离线 wrapper 的 Gowin 综合均以退出码 0 完成。逻辑顶层为 6,577 个寄存器、7,437 个逻辑单元、29 个 BSRAM 和 10 条 warning；125 MHz 网络 PLL 域综合估算达到 158.532 MHz。它仍暴露 562 个 I/O Buffer，只用于逻辑集成和信号可观察性。
- 离线 wrapper 只使用官方 ch58 RGMII/复位管脚，未知输入全部固定为失效安全值，系统域使用 RGMII RX `/2` 的 62.5 MHz 工具时钟。最终 P&R/STA 对 35,356 条路径、20,381 个端点报告 0 个 setup/hold 违规；网络 PLL 域 Fmax 129.443 MHz，最差 setup slack `+0.275 ns`，I/O Buffer 为 14。该结果不含未知 PHY/PCB `set_input_delay`/`set_output_delay`，生成的码流也未上板，不能表述为最终板级时序、PHY、真实 PWM、物理急停或实车安全通过。
- 逻辑顶层中两个 `range_safety_limiter` 的模块层次被等价扁平化，但综合网表仍保留测距到最终线速度和 `status_bits[4]` 的安全锥；端到端自检覆盖 400 mm 减半、100 mm 停车和恢复。离线 wrapper 中自治、相机、IMU、电机等锥因安全常量和无物理输出被裁剪，属于该 wrapper 的明确边界。官方 `pll_init.v` 两条截断 warning 保持不改。

### 5.1 RTL 仿真证据快照

最近一次成功完整回归（2026-08-24）如下：

| testbench | 最近成功结果 | 主要覆盖 |
|---|---:|---|
| `tb_motion_mode_arbiter` | 15 项 | 独占选源、停车后切换、切换保护和三类安全置零 |
| `tb_fpga_control_packet_rx` | 10 项 | 黄金向量、CRC/FCS、错误头、30/32 字节异常长度、重复序号和回绕 |
| `tb_fpga_telemetry_packet_tx` | 72 字节整帧通过 | 字段顺序、CRC，以及 `ready_i = 0` 时四拍背压保持 |
| `tb_sensor_frontends` | 37 项 | OV5640 RGB565、IMU 六轴原子提交和三路测距有效性 |
| `tb_autonomous_motion` | 18 项 | 减速停车、左右绕障、路径暂停/恢复和段执行 |
| `tb_motor_control` | 9 项 | AB 解码、轮速、差速运动学、定点截断/饱和边界、换向和零目标清 PWM |
| `tb_robot_rtl_core` | 21 项 | 系统连接、模式隔离、旧目标失效、急停/故障、看门狗和路径进度接口 |
| `tb_eth_udp_loopback_rgmii` | 完整序列通过 | 72 字节回环、坏 FCS、描述符 Full 注入、写指针回滚和后续帧无污染 |
| `tb_async_frame_fifo` | 23 项 | 异频异相、Gray 单比特变化、满空、溢出拒绝、整记录顺序和异步复位恢复 |
| `tb_robot_network_e2e` | 72 项 | ARP、连续 ICMP、完整 Control/Telemetry/Status、各层校验、序号、双异步时钟、FIFO 满、安全减速/停车、看门狗、急停、故障和复位 |
| `tb_acg720_udp_loopback_project` | 顶层展开通过 | Gowin 原语模型、官方 PLL、RGMII 顶层和本地消息 FIFO 共同展开；不检查时序 |

运行入口：

```bash
cd FPGA_ws
./Scripts/run_all_sim.sh
```

脚本不把仿真器复制进仓库，需要系统提供 `iverilog`/`vvp`，或通过 `IVERILOG_BIN`、`VVP_BIN`、`IVERILOG_BASE` 指定本地工具链。本轮成功证据使用解压到 `/tmp/fpga_gowin_iverilog` 的 Icarus 11；该临时目录不是仓库资产，被清理后必须重新安装或显式指定工具链。

### 5.2 Gowin 综合证据快照

在 `FPGA_ws` 目录运行：

```bash
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh Scripts/run_gowin_synth.tcl
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_synth.tcl
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_offline_synth.tcl
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_offline_pnr.tcl
```

前 3 项命令只执行综合；最后一项执行离线 wrapper 的布局布线、STA、码流和功耗报告。源码工程不提交被忽略的 `impl/` 产物。离线实现已解决综合工具中的 I/O 超量并达到内部 125 MHz 目标，但未知外设管脚、电气参数、真实系统时钟和 PHY/PCB 延迟仍保持不猜测。

### 5.3 Git 发布边界

- `fpga_rtl` 的 `main` 已发布到 `7922400`，包含模式仲裁器、官方 UDP 回环、FCS 原子拒绝、协议、CDC、传感器、电机、自主运动、系统级 RTL、离线综合/布局布线工程和自检 testbench。
- `fpga_gateway` 的 `main` 已发布到 `88f760c`，`robot_bringup` 的 `main` 已发布到 `c188c99`。
- 父仓库直接发布 `robot_navigation`、`vision_bridge`、`mission_control`、依赖脚本和项目文档；两个通信包仍由同步脚本从独立仓库获取。
- `ros_ws/src/external/` 是安装脚本在本机重建的依赖覆盖层，整体忽略且不上传；可上传的环境入口为 `Scripts/setup_ros_dependencies.bash`。

## 6. 下一步

ROS 主机基线和本阶段 FPGA 纯 RTL 仿真闭环已经完成。由于当前不上实板，继续推进时只建立由真实型号决定的板级骨架，不猜测引脚或电气接口：

1. 固化电机驱动器、编码器、IMU、三路测距和 OV5640 接口清单，以及 ACG720 引脚、电平、真实系统时钟和复位表；据此从离线 wrapper 派生最终板级顶层。
2. 根据 PHY 型号、时钟关系和 PCB 延迟补齐 RGMII 输入/输出时序约束，重新执行最终板级 P&R/STA；不得把当前离线内部时序结果直接沿用。
3. 上板条件具备后，先保持电机驱动禁用，依次验证 RGMII、ARP、Ping、UDP Control/Telemetry/Status 黄金报文，并保存抓包和故障注入证据。
4. 网络与仲裁器通过实板门槛后再架空车轮验证编码器、PWM、急停、看门狗和限幅，随后接真实 IMU、测距、摄像头、DDR3/HDMI、绕障和独立路径。

`robot_navigation`、`vision_bridge` 和 `mission_control` 当前不继续扩展算法或模型。纯 RTL、离线低 I/O 实现和内部时钟 STA 已到达无实物阶段目标；下一项不可替代门槛是真实接口清单、PHY/PCB I/O 约束与实物验证，而不是继续增加通用抽象。

## 7. 未决项

在实现对应模块前再决定，不提前建抽象：

- 模式来自板上开关、外部按键还是配置寄存。
- “电机已停止”使用目标速度为零、编码器实测速度为零，还是两者同时满足。
- 安全层对 Jetson 速度命令是分量夹紧还是统一缩放。
- FPGA 独立路径和绕障内部定点格式。Jetson 速度命令已确定为 `int32` 的 mm/s 与 mrad/s。
- 主机侧如何展示新增冻结的 `status_bits[13:0]` 与 `fault_bits[5:0]`；未知/保留位仍不得触发自动恢复。
- 实际引脚约束、时钟频率和复位极性。

## 8. 安全与实施边界

- 未完成机器人引脚/电气集成以及急停、限幅和看门狗的综合/架空轮验证前，不得在车轮接地状态下启用电机。
- 实车首次闭环测试必须架空车轮并保证物理急停可直接切断驱动使能。
- 不从零重写通用网络栈，优先复用 ACG720 官方 RGMII/GMII、IPv4/UDP 和第 52 章 ARP 帧边界；项目只补齐固定端点所需的最小 responder/ICMP 逻辑。
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

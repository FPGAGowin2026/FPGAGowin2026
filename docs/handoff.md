# 项目交接与证据边界

更新：2026-09-20。已完成工作区合并、麦轮控制逻辑与 Tang Primer 25K 单电机辨识工程，接下来在 Windows 端进行实物验证。修改尚未提交或推送 Git；未恢复已删除的旧固件/差速工程，未修改 ROS 包。

## Windows 接续入口

首先阅读 [25K 电机辨识：Windows 端接续](Tang_Primer_25K电机辨识.md#windows-端接续2026-09-20-交接)。该文档集中保存当前接线、搬运清单、码流哈希、Windows 构建示例、串口指令和待测表。

- 当前实物任务：确认上A/上B/下A/下B四通道对应的车轮位置及电机/编码器极性，尚无实测结论。
- 当前可下载产物：`FPGA_ws/boards/tang_primer25k_motor_id/impl/pnr/motor_id.fs`，目标25K **B版**，SRAM临时下载；实际器件版本待现场核对。
- 当前辨识工程已在Linux通过自检仿真、综合、布局布线及出码流；未在Windows重建或下载，未验证实际电机。
- 独立麦轮闭环 `mecanum_motor_control.sv` 只有RTL自检证据，不要把辨识工程出码流等同于麦轮闭环已完成板级验证。
- 父仓库忽略独立 `FPGA_ws/`，两处均有未提交/未跟踪内容，码流另被忽略。Windows只拉取父仓库不足以继续；需复制当前工作树与码流，或另行安排两个仓库的发布。本次仅更新文档，不执行提交/推送。

## 1. 当前基线

Jetson Orin Nano＋四轮麦轮；AT8236、IMU660RC（图片标明 LSM6DSV16XTR）、保留 IMU963RA、OV5640 和 N10。功能正确与开发速度优先，安全基础暂不实现。IMU660RC 驱动和实时反射链尚未完成。麦轮电机逻辑已新增独立 RTL 与两项功能自检，尚未完成网络接入、板级工程与实车验证。

### 电机控制补充（2026-09-20）

新增 `rtl/motor/mecanum_motor_control.sv`：四轮 IK/FK、协调限速、目标斜坡、四路 QEP/测速/PI/AT8236 输出；复用单轮 PI，并将积分限幅表达式改为 64 位，避免较大 GAIN_SHIFT 时提前溢出。新增 `sim/motor/` 两项自检及 `Scripts/run_motor_sim.sh`，Icarus 11.0 实跑通过。`-Wall` 的数组敏感列表提示不影响结果。参数、接口和测试边界见 [总体方案第 5 节](系统总体方案.md#5-运动控制与轻量本体估计)。未修改 N10、ROS、旧 AT8236 实验顶层或安全模块；未提交、推送 Git。

单电机辨识补充：新增 Tang Primer 25K B 版板级工程，接线按用户上A/上B/下A/下B提供，轮序尚未知。点动/串口自检及 Gowin 综合、布局布线、出码流通过，未烧录或实物验证；详见 [操作文档](Tang_Primer_25K电机辨识.md)。

## 2. 合并后的工作区

- FPGA_ws 统一管理 `rtl/`、`firmware/`、`sim/`；软核硬件位于 `rtl/softcore/`，供应商软核 IP 位于 `rtl/vendor/`。
- N10 在 `src/**/n10/`、`src/**/n10_rtt/` 和 `sim/n10/` 的内容与路径不变，其软核 SoC 已归入同仓库 `rtl/softcore/soc/n10_rtt/`。
- firmware 当前仅有说明，不恢复先前已删除的软核程序。镜像分别使用 `build/firmware/pico_root/ram32.hex` 与 `build/firmware/n10_rtt/ram32.hex`，当前均未生成。
- gowingame 的完整 Git 和原工作树移出活动目录归档；原同步脚本停用，发布脚本拒绝旧目标。统一入口为 `sync_fpga_workspace.sh` 与 `push_git.sh FPGA_ws`。
- ROS 工作区不变，仍使用历史差速协议。GoWin 与 llm_ws 不纳入主线已实现证据。

## 3. 迁移记录

本次搬移 51 个源码文件，其中 21 个来自 gowingame；21 个 N10 src 文件原位保留，29 个 sim 文件不变。除两个 SoC 配置的固件路径修改外，已核对所有迁移及保留文件 SHA256 与清单一致。

本地归档：`/home/aa/.fpga-workspace-backups/merge-20260920-174755/`。

- `before-merge.tar.gz`：迁移前 FPGA_ws、gowingame（均含完整 .git 和工作树）及父项目相关文档/脚本。
- `gowingame-retired/`：原 gowingame 完整目录及其未提交状态。
- `migration_manifest.json`：源路径、目标路径与迁移前后哈希。

此前清理的差速、安全、旧 IMU 文件不恢复。归档不在活动仓库内，不会被普通同步或发布脚本上传。

## 4. 验证边界

已通过文件映射与哈希检查（51 个搬移文件、21 个原位 N10 源文件、29 个仿真文件），未发现遗漏或额外源码；SoC include 路径、相对固件路径及脚本语法检查通过。离线模拟 Git 检查确认旧同步/发布入口退出且不调用 Git；Git 忽略规则与 diff --check 通过。这些检查不代替 HDL 编译。上述工作区迁移检查当时没有重跑 HDL 验证。其后已新增独立25K电机辨识工程/约束并完成仿真、综合、布局布线和码流生成，详见开头的最新证据；整车主线仍无完整板级集成，未进行板测。加密 IP、N10 历史测试缺失依赖等原有边界保持。

## 5. 历史验证档案的阅读规则

以下原记录主要截至 2026-08-27，保留用于追溯旧差速工程；包含当时的“当前”“已完成”和百分比，不指代现工作树或新基线。旧命令、临时工具、Gowin 工程与产物可能已不存在，不能作为当前可复跑入口。专项记录另见 [调研报告](./2026高云FPGA选题3_工程交付与传感器调研报告.md)。

## 6. 历史差速工程记录

- 已建立 `ros_ws/src/robot` 和 `ros_ws/src/external`；`external` 保存不写入系统目录的 ROS 2 Humble 二进制依赖覆盖层，通过 `Scripts/install_ros_dependencies.sh` 安装，并由 `Scripts/setup_ros_dependencies.bash` 激活。该目录整体忽略，不上传到 Git。
- `ros_ws/src/robot/fpga_gateway` 由 `ros_ws` 私有仓库统一管理，包含 C++17 固定二进制协议、POSIX UDP 网关、主机侧 `fake_fpga_node` 和 4 项 GTest。
- `ros_ws/src/robot/robot_bringup` 由 `ros_ws` 私有仓库统一管理，包含仿真与硬件两套 launch、集中 YAML 参数，以及一项自动化 ROS 主机闭环 launch 测试。
- `ros_ws` 包含 `robot_navigation`、`vision_bridge` 和 `mission_control` 三个 C++17/`rclcpp` 原型包：分别提供 EKF/SLAM/Nav2、红色单目标观测，以及“到搜索区—搜索—对准—接近—停车”状态机。Python 只保留给 ROS 2 launch。
- 当前网关发布版本增加了轮式里程计与 IMU 的非零协方差参数，`robot_bringup` YAML 提供对应初值；这些值只用于主机融合起步，必须用真实静止、直行和定点旋转数据重新标定。
- [`fpga_rtl`](https://github.com/FPGAGowin2026/fpga_rtl) 已作为独立私有仓库发布，包含 Verilog-2001 模式仲裁器和一项自检 testbench；运行 `Scripts/sync_fpga_workspace.sh` 可将其首次克隆或快进更新到 `FPGA_ws/`。
- 父仓库不重复保存独立仓库源码；运行 `Scripts/sync_ros_packages.sh` 可首次克隆或快进更新整个 `ros_ws/`，五个自研 ROS 包统一管理。
- 协议源文件为 [`udp_protocol.md`](./udp_protocol.md)，当前只定义控制、综合遥测和状态三类报文，不含参数或轨迹段上传。
- `FPGA_ws` 当前是 `fpga_rtl` 的独立检出。从父仓库干净克隆后通过同步脚本获取该仓库；`acg720_udp_loopback.gprj` 保留官方回环基线，新增 `acg720_robot_network.gprj` 引用机器人网络集成顶层和全部 RTL 依赖。器件仍为 `GW5AT-LV138PG484AC1/I0`；机器人工程只复用已知 RGMII 约束，不猜测系统时钟、电机、编码器、IMU、测距或急停引脚。
- 本机商业版 `Gowin_V1.9.12.03` 与 License 位于父仓库忽略的 `GoWin/` 目录；`gwlicense.ini` 已指向本地 License。2026-08-24 已在可读取本机网卡标识的环境完成官方 UDP 回环和机器人网络顶层综合。受限沙箱无法访问 `AF_NETLINK` 时会误报 hostid 不匹配；这不是 License 本身失效。
- `build/`、`install/`、`log/`、Python 缓存和 `src/external/` 由 `ros_ws/.gitignore` 排除；父仓库整体忽略 `ros_ws/`。
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
- 离线 wrapper 只使用官方 ch58 RGMII/复位管脚，未知输入全部固定为失效安全值，系统域使用 RGMII RX `/2` 的 62.5 MHz 工具时钟。最终 P&R/STA 使用 6,556 个逻辑单元、5,832 个寄存器和 58 个物理 BSRAM primitive，对 35,356 条路径、20,381 个端点报告 0 个 setup/hold 违规；网络 PLL 域 Fmax 129.443 MHz，最差 setup slack `+0.275 ns`，I/O Buffer 为 14。该结果不含未知 PHY/PCB `set_input_delay`/`set_output_delay`，生成的码流也未上板，不能表述为最终板级时序、PHY、真实 PWM、物理急停或实车安全通过。
- 逻辑顶层中两个 `range_safety_limiter` 的模块层次被等价扁平化，但综合网表仍保留测距到最终线速度和 `status_bits[4]` 的安全锥；端到端自检覆盖 400 mm 减半、100 mm 停车和恢复。离线 wrapper 中自治、相机、IMU、电机等锥因安全常量和无物理输出被裁剪，属于该 wrapper 的明确边界。官方 `pll_init.v` 两条截断 warning 保持不改。
- 已选定 Gowin_PicoRV32 RV32IMC 作为低速服务核；在 `GW5A-LV25MG121NC1/I0` 上完成 16 KiB ITCM、8 KiB DTCM、Simple UART 和 Open Wishbone 工程。最终 P&R 使用 4,875 个逻辑单元、2,687 个寄存器、12 个 BSRAM 和 4 个 DSP，50 MHz 下 Fmax 52.803 MHz且 setup/hold 违例端点均为 0。SRAM 烧录后 `P/I/S/?` 四条 COM5 命令、运行时 cycle CSR 与 `0xC0DEC0DE` 总线回读均通过；这不等于 ACG720 或整车验证。
- 通用 IPv4 checksum、辅助 ICMP/IP checksum 和发送字节选择已分阶段寄存；ARP/ICMP 字节、IPv4/ICMP checksum 与 FCS 保持正确，连续请求无跨帧污染。`gmii_to_rgmii` 未定宽常量/generate warning 和轮速算术截断 warning 已消除；轮速控制明确使用幅值低 16 位、饱和到 PWM 上限，并保留零目标清状态行为。

### 6.1 RTL 仿真证据快照

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

### 6.2 Gowin 综合证据快照

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

### 6.3 Git 发布边界

- `fpga_rtl` 的 `main` 已发布到 `7922400`，包含模式仲裁器、官方 UDP 回环、FCS 原子拒绝、协议、CDC、传感器、电机、自主运动、系统级 RTL、离线综合/布局布线工程和自检 testbench。
- ROS 工作区统一使用 `ros_ws` 仓库，沿用原 `fpga_gateway` 历史；原通信包发布基线为 `88f760c` 和 `c188c99`。
- 父仓库发布依赖脚本和项目文档；五个自研 ROS 包均由同步脚本从 `ros_ws` 仓库获取。
- `ros_ws/src/external/` 是安装脚本在本机重建的依赖覆盖层，整体忽略且不上传；可上传的环境入口为 `Scripts/setup_ros_dependencies.bash`。

### 6.4 完成度估算

2026-08-27 快照：按最终现场演示门槛估算，总体约 **38%**，剩余约 **62%**，估算区间 **35%～41%**。这是原报告的历史估算，不是本次检查后的新进度，也不按代码量或工期计算。

| 工作包 | 总体权重 | 当前完成 | 当前证据 |
|---|---:|---:|---|
| 架构、协议与仓库治理 | 5% | 约 95% | 总体方案、UDP 协议、独立仓库和同步流程已建立 |
| ROS 主机通信底座 | 10% | 约 80% | 网关、模拟器、bringup、4 项协议 GTest 和 1 项 launch 回归已完成 |
| FPGA 模式仲裁与安全基础 | 8% | 约 62% | 仲裁器、看门狗和测距限速进入端到端 RTL与综合网表，减速/停车可观察；尚无物理断电、最终板级输入或实板证据 |
| FPGA 以太网与协议 RTL | 12% | 约 67% | ARP、ICMP、Control、Telemetry/Status、各层校验、CDC 与背压完成纯 RTL闭环；14-I/O 离线 wrapper 内部 125 MHz STA 通过，PHY/PCB I/O 时序和实板未完成 |
| 电机闭环与物理安全 | 15% | 约 15% | AB 解码、轮速估计、差速运动学和双轮 PI/PWM 目标通过仿真；无真实驱动、编码器、堵转或急停断电 |
| 摄像头、IMU、测距与显示 | 12% | 约 12% | DVP RGB565、IMU 原子提交和三路测距有效性通过仿真；无器件配置、CDC、DDR3/HDMI 或实物数据 |
| FPGA 绕障与独立路径 | 12% | 约 25% | 减速/停车、左右择向绕障、暂停恢复和四段 `MOVE/ROTATE/MOVE/STOP` ROM 通过仿真；进度仍由 testbench 注入 |
| ROS 融合、SLAM 与 Nav2 | 10% | 约 35% | 主机仿真已生成融合里程计、地图并完成 Nav2 点到点；实机传感器和标定未验证 |
| 视觉 AI 与智能任务 | 5% | 约 25% | 合成相机红色目标与任务闭环通过；TensorRT 和真实相机未接入 |
| 系统集成与稳定性收口 | 11% | 约 13% | 10 个 RTL 功能自检、1 个板级展开、三项综合和一次离线 P&R/STA 通过；无真实板级 I/O、实车标定、长时运行或现场演示证据 |

加权后总体完成度约为 38%。分项百分比是用于排序和暴露风险的工程估算，不表示已经消耗或还需要相同比例的时间；离线 P&R/STA 不会按相同比例降低实板风险。

### 6.5 独立专项工程

[工程交付与传感器调研报告](./2026高云FPGA选题3_工程交付与传感器调研报告.md) 记录了 2026-08-29 的 `Choice3_Robot_25K`、`Choice3_Robot_138K`、29 个传感器示例和 ACG720 教程审计。25K 的板测是物理驱动禁用的片上激励/UART 场景，138K Choice3 仅有仿真和综合。它们与 `FPGA_ws` 网络主线分别统计，不把专项工程的四轮配置或验证结果自动并入本主线。

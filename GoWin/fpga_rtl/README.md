# FPGAGowin2026 FPGA RTL

本仓库保存 [FPGAGowin2026](https://github.com/FPGAGowin2026/FPGAGowin2026) 的 FPGA RTL、仿真和 Gowin 工程。当前已经形成从完整 RGMII 以太网帧、Control 解析、`robot_rtl_core` 安全门控到 Telemetry/Status 完整返回帧的纯 RTL 仿真闭环；官方 UDP 回环和机器人逻辑集成顶层均已完成 Gowin 综合，14-I/O 离线 wrapper 已完成布局布线和内部时钟 STA。该 wrapper 不是最终上板顶层，尚无 PHY/PCB I/O 时序或实板验证。

[`boards/tang_primer_25k_bringup`](boards/tang_primer_25k_bringup) 提供当前 `GW5A-LV25MG121NC1/I0` 测试板的独立 UART 板级自检工程，用于验证 Windows Gowin 工具链、JTAG SRAM 烧录和双向串口调试；它不改变 ACG720 正式工程的器件与接口边界。

[`templates/picorv32_robot_soc`](templates/picorv32_robot_soc) 是选题三的软核服务平面模板。它已在同一 25K 板上完成 RV32IMC 取指、cycle CSR、Simple UART、Open Wishbone、50 MHz 时序、SRAM 烧录和 `P/I/S/?` 双向串口实测；迁到 138K 时保持纯 RTL 实时/安全层拥有最终裁决权。

## Tang Primer 25K 赛题三验证链

本仓库与相邻的 [`sensor_examples`](../sensor_examples) 现已形成一条面向 2026 FPGA 赛题三的分层验证链：

- [`../sensor_examples`](../sensor_examples)：29 个传感器/前端示例；每项均有独立仿真、Gowin 综合、Device B 布局布线和时序门禁。D435i 仅作为 USB 3 主机侧扩展，不伪装成普通 FPGA GPIO 外设；TSL1401 的模拟输出明确要求外接 ADC。
- [`boards/tang_primer_25k_sensor_bus_debug`](boards/tang_primer_25k_sensor_bus_debug)：对 4 个 I2C 和 8 个 SPI 目标做无外设总线诊断；已在 Tang Primer 25K 上完成 SRAM 下载和 COM5 UART 验证。
- [`boards/tang_primer_25k_sensor_functional_bist`](boards/tang_primer_25k_sensor_functional_bist)：对其余 17 类数字接口/解析逻辑做片上功能激励；17/17 通过仿真和板上 UART 自检，但不等同于真实传感器电气验证。
- [`boards/tang_primer_25k_rtthread`](boards/tang_primer_25k_rtthread)：Gowin GoRV32 + RT-Thread 4.1.1，64 KiB RAM；已完成 W25Q64 固件下载、25K SRAM 位流下载和 20 轮 UART shell 严格回归。
- [`boards/tang_primer_25k_choice3_robot`](boards/tang_primer_25k_choice3_robot)：OV5640、ICM42688-P、VL53L0X、HC-SR04、4 路数字 IR、4 路编码器、4 电机 TB6612、双缓冲 HDMI 和 UART 服务平面的完整 25K 纯 RTL 顶层。冻结版本通过四项 Icarus 回归、Device B P&R（setup/hold 0/0）和无外设板级安全门禁；真实相机、IMU、测距、HDMI、电机及机械系统仍需联调。

最终整机工程的接线、命令、资源、时序、位流哈希和证据边界以其目录中的 `README.md`、`RESULTS.md` 为准。

安装 Icarus Verilog 后可一次运行全部回归：

```bash
./Scripts/run_all_sim.sh
```

脚本支持系统 `iverilog`/`vvp`，也支持通过 `IVERILOG_BIN`、`VVP_BIN` 和 `IVERILOG_BASE` 指定解压到工作区外的本地工具链。当前包含 10 个 RTL 功能自检和 1 个 Gowin PLL/RGMII 板级展开检查。

## ACG720 官方 UDP 回环仿真

[`src/ethernet`](src/ethernet) 中的 RGMII/GMII、IPv4/UDP 和 CRC 基线来自 ACG720 官方全套例程压缩包中的 `GW5AT138K/第58章 千兆以太网UDP回环测试/ch58_eth_udp_lookback_rgmii.zip`，之后在仓库内补上帧原子提交、IPv4/UDP 校验和检查和机器人集成逻辑。下载包 SHA-256 为 `563397fee37881cf1fd3c80088c0bec1814ef97ee8143838a64d34bfb5e2a81b`。

[`sim/tb_eth_udp_loopback_rgmii.sv`](sim/tb_eth_udp_loopback_rgmii.sv) 使用官方默认地址构造完整 RGMII 帧：开发板 `00:0a:35:01:fe:c0 / 192.168.0.2:5000`，测试端 `02:00:00:00:00:01 / 192.168.0.1:6000`。它逐字节检查返回报文、IP/UDP 端点交换、8 字节负载、以太网填充和 FCS，并覆盖“合法帧—坏 FCS—描述符 FIFO Full 信号注入—不同负载合法帧”，验证整帧原子拒绝、写指针回滚和无残留污染。

## ACG720 Gowin 工程

[`acg720_udp_loopback.gprj`](acg720_udp_loopback.gprj) 保留为官方回环板级基线；[`acg720_robot_network.gprj`](acg720_robot_network.gprj) 引用机器人逻辑集成顶层和全部依赖；[`acg720_robot_network_offline.gprj`](acg720_robot_network_offline.gprj) 使用只引出官方 RGMII/复位信号的 14-I/O 离线 wrapper。三者器件均固定为官方 138K 型号 `GW5AT-LV138PG484AC1/I0`，复用 ch58 的 RGMII 管脚与 125 MHz 相移 PLL。工程没有猜测电机、编码器、IMU、测距、系统时钟或急停引脚。

`run_all_sim.sh` 中的 `tb_acg720_udp_loopback_project` 只验证 Gowin 原语模型、官方 PLL 和该顶层可以共同展开；它本身不代表 Gowin 综合、布局布线或时序通过。综合证据来自下述独立 `gw_sh` 命令和生成报告。

本机商业版 License 已保存在父工作区被 Git 忽略的 `GoWin/` 目录，`gwlicense.ini` 已指向该本地文件。2026-08-24 已确认 `gw_sh` 在可读取本机网卡标识的环境中通过许可证检查并完成两项工程的综合。受限容器若不能创建 `AF_NETLINK` socket，可能误报 `License hostid not match`，应回到能读取本机网卡标识的宿主环境运行。

在本仓库根目录运行官方 UDP 回环综合入口：

```bash
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh Scripts/run_gowin_synth.tcl
```

`run_gowin_synth.tcl` 打开 `acg720_udp_loopback.gprj`；机器人网络工程由单独脚本打开，避免混淆两个顶层的报告。

机器人网络集成工程综合入口：

```bash
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_synth.tcl
```

2026-08-24 三项综合均以退出码 0 完成。机器人逻辑集成顶层报告 6,577 个寄存器、7,437 个逻辑单元、29 个 BSRAM 和 10 条 warning；`rgmii_rx_clk_i`、100 MHz 系统域和 125 MHz 网络 PLL 域综合估算分别为 230.342、126.767 和 158.532 MHz。逻辑顶层仍有 562 个 I/O Buffer，只用于 RTL 集成和信号可观察性，不能直接作为 PG484 物理顶层。

确定无歧义的 `gmii_to_rgmii` 未定宽常量、generate block identifier 和轮速控制算术截断 warning 已修复。剩余 warning 为两条官方 `pll_init.v` 截断、两个当前无消费者的物理输入，以及摄像头/电机等无可观察输出的层次 sweep。两个 `range_safety_limiter` 的模块层次虽被扁平化，但综合网表仍保留 `front_distance_mm_i → final_linear_mm_s_o/status_bits_o[4]` 的安全锥；端到端测试进一步验证 400 mm 减半、100 mm 停车并置位状态、测距恢复后只释放当前新命令，因此不使用 `keep` 掩盖层次优化。

离线 wrapper 用 RGMII RX 125 MHz 的 `/2` 生成 62.5 MHz 工具系统时钟，并把未知输入固定为急停/严重故障有效、路径中止、测距无效和运动停止。最终 P&R 使用 6,556 个逻辑单元、5,832 个寄存器、58 个物理 BSRAM primitive、14 个 I/O Buffer 和 1 个 PLL；35,356 条路径、20,381 个端点中 setup/hold 违规均为 0。实际 Fmax 为 RGMII RX 294.309 MHz、离线系统域 79.227 MHz、网络 PLL 域 129.443 MHz，最差 setup slack 为 `+0.275 ns`。工具还生成码流和功耗报告，但它们只是离线产物，未上板验证。

离线 SDC 仅对 RGMII RX、`/2` 系统时钟、PLL 自动 generated clock 和明确的同步器第一级建模；没有使用宽泛 clock-group false path。真实 PHY/PCB 延迟未知，故没有猜测 `set_input_delay`/`set_output_delay`。因此上述 0 违规只证明当前内部同步路径，不证明 RGMII 板级 I/O 时序、PHY 或实板通过。

离线实现入口：

```bash
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_offline_synth.tcl
LD_LIBRARY_PATH=../GoWin/Gowin_V1.9.12.03/IDE/lib \
  ../GoWin/Gowin_V1.9.12.03/IDE/bin/gw_sh \
  Scripts/run_gowin_robot_network_offline_pnr.tcl
```

综合与实现产物位于被 Git 忽略的 `impl/gwsynthesis/` 和 `impl/pnr/`，不纳入源码仓库。

官网原始 `ACG720-GW5AT138K例程文件.zip` 与通用管脚表保存在本机忽略目录 `official/`。例程包 SHA-256 为 `563397fee37881cf1fd3c80088c0bec1814ef97ee8143838a64d34bfb5e2a81b`，不进入 Git。

在本仓库目录运行：

```bash
SIM_IVERILOG=iverilog
SIM_VVP=vvp
GOWIN_PRIM=../GoWin/Gowin_V1.9.12.03/IDE/simlib/gw5a/prim_sim.v

$SIM_IVERILOG -g2012 -s tb_eth_udp_loopback_rgmii \
  -o /tmp/tb_eth_udp_loopback_rgmii.vvp \
  "$GOWIN_PRIM" \
  src/ethernet/eth_msg_buf.v \
  src/ethernet/crc32_d8.v \
  src/ethernet/ip_checksum.v \
  src/ethernet/eth_udp_rx_gmii.v \
  src/ethernet/eth_udp_tx_gmii.v \
  src/ethernet/rgmii_to_gmii.v \
  src/ethernet/gmii_to_rgmii.v \
  src/ethernet/eth_udp_loopback_rgmii.v \
  sim/tb_eth_udp_loopback_rgmii.sv
$SIM_VVP /tmp/tb_eth_udp_loopback_rgmii.vvp +VALID_ONLY
$SIM_VVP /tmp/tb_eth_udp_loopback_rgmii.vvp
```

2026-08-24 最近一次完整回归中，两条运行命令均以零状态通过。当前本机没有系统级 `iverilog`/`vvp`，本轮使用 `/tmp/fpga_gowin_iverilog` 临时工具链；该目录不是仓库资产，再次验证前需安装工具或通过脚本支持的环境变量指定工具链。修复使用帧感知负载缓存：FCS 判定前只暂存数据，合法帧才原子提交负载与元数据；坏 FCS、负载区满或描述符 FIFO 满都会回滚整帧写指针。这样既不会发送坏帧，也不会把被拒绝帧的负载残留给下一合法帧。

该 testbench 只对 PLL 外壳和消息 FIFO 生成核使用等价行为模型；RGMII `IDDR/ODDR` 使用已安装 Gowin IDE 的 GW5A 官方原语模型。此结果属于 RTL 功能仿真，不等于综合、时序收敛、PHY 时序或 ACG720 实板验证。

## 项目 RTL 闭环

- [`src/protocol`](src/protocol)：Control 固定报文解析、CRC16-CCITT-FALSE、长度/头部/序号拒绝，以及固定 72 字节 Telemetry 和 38 字节 Status 编码。
- [`src/cdc/async_frame_fifo.v`](src/cdc/async_frame_fifo.v)：深度 4 的整记录双时钟 FIFO，Gray 指针只发布完整 Control 或完整返回报文。
- [`src/ethernet/acg720_robot_network_rgmii.v`](src/ethernet/acg720_robot_network_rgmii.v)：最小 ARP responder、ICMP Echo Reply、IPv4/UDP Control 入口、100 Hz Telemetry、10 Hz Status 和发送仲裁；电机 PWM/方向输出在该阶段保持未连接。
- [`src/sensors`](src/sensors)：OV5640 DVP RGB565 组帧、IMU 六轴字节流原子提交，以及前/左/右三路测距的偏置、量程和超时监视。
- [`src/motor`](src/motor)：AB 相正交解码、轮速估计、差速运动学和双轮 PI/PWM 目标；参数仅为仿真初值。
- [`src/control`](src/control)：前向减速/停车、左右择向局部绕障，以及可由 ROM/BRAM 驱动的 `MOVE/ROTATE/STOP` 路径执行。
- [`src/system/robot_rtl_core.v`](src/system/robot_rtl_core.v)：把协议、传感器、演示路径、模式仲裁和电机控制接成系统级仿真骨架。

系统级自检覆盖四段内部演示路径、绕障暂停恢复、FPGA/Jetson 模式隔离、离开独立模式时中止旧路径、软件与物理急停、严重故障、缓存命令失效、零目标清 PI/PWM、控制看门狗、Jetson 命令安全限速、摄像头像素和 IMU 样本原子提交。新增端到端自检另覆盖 ARP、ICMP、完整 Control/Telemetry/Status 帧、FCS/IP/UDP/应用校验、目标端点、序号、CDC FIFO 满和异步复位。具体 I2C/SPI/TOF 驱动、DDR3、HDMI、PHY 状态、机器人系统时钟/复位来源、引脚约束和物理急停断电仍须在板级工程中补齐。

## 模式仲裁器

[`src/motion_mode_arbiter.v`](src/motion_mode_arbiter.v) 实现以下已冻结契约：

- `active_mode_o = 0` 时只转发 FPGA 独立路径命令，`active_mode_o = 1` 时只转发 Jetson 命令。
- 模式请求变化后立即输出零；只有 `motion_stopped_i = 1` 才更新活动模式，并再保持一个时钟周期零输出。
- 复位、急停、严重故障、安全停车、模式切换或当前来源命令无效时，命令输出为零且 `motion_enabled_o = 0`。
- 复位后的安全默认模式为 FPGA 独立模式。

集成边界：

- `requested_mode_i` 必须由顶层完成同步和按键消抖；其来源仍可选择板上开关、外部按键或配置寄存器。
- `motion_stopped_i` 必须由顶层根据最终停止判据生成；仲裁器不猜测目标速度和编码器实测速度之间的关系。
- `safety_stop_i` 表示必须置零的障碍物安全停车。速度限幅或按比例减速在安全层单独实现。
- 本模块的急停门控不能代替直接切断电机驱动使能的物理急停链路。

也可单独运行模式仲裁器自检：

```bash
iverilog -g2012 -s tb_motion_mode_arbiter \
  -o /tmp/tb_motion_mode_arbiter \
  src/motion_mode_arbiter.v \
  sim/tb_motion_mode_arbiter.sv
vvp /tmp/tb_motion_mode_arbiter
```

该自检覆盖模式隔离、运动中切换阻止、切换零输出保护、三类安全停车和无效命令停车。它不等于 Gowin 综合、时序分析或实板安全验证。

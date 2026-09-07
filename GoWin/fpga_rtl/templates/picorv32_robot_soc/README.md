# Gowin PicoRV32 机器人服务核工程模板

本模板面向竞赛选题三“基于 FPGA 的具身智能机器人实时感知与自主控制系统设计”。正式软核选择为 **Gowin_PicoRV32（RV32IMC）**；它只承担低速服务与任务管理，电机闭环、避障停车、急停和最终 PWM 门控继续由纯 RTL 独立完成。

## 1. 为什么选择 PicoRV32

| 候选 | 优点 | 不作为当前主核的原因 |
|---|---|---|
| Gowin_PicoRV32 | 官方同时支持 GW5A-25B 与 GW5AT-138B；RV32IMC；开放 Wishbone；资源较小 | 当前高云版本的 Tcl IP 生成接口不完整，需要本模板的可复现准备脚本 |
| Gowin_GoRV32 Nano | 新架构、APB 简洁 | 本机缺少成熟工程/SDK闭环；RV32I 代码密度与当前可验证性不如 PicoRV32 |
| Gowin_GoRV32 | 标准外设和调试体系更完整 | 对服务平面偏重，会挤占摄像头、DDR、RGMII 与实时控制资源 |

25K 实测完整 SoC 使用 4,875/23,040 个 Logic（22%）、2,687/23,280 个寄存器（12%）、12/56 个 BSRAM（22%）和 4/28 个 DSP（15%）。它为当前 25K 测试板留出了足够余量，也符合迁移到 138K 后把主要资源留给赛题数据面的目标。

## 2. 最终架构边界

```text
Jetson / RK3588：SLAM、视觉识别、语义导航、任务规划
                         │ UDP / 固定协议
GW5AT-138 FPGA           │
├─ PicoRV32 服务平面（50 MHz）
│  ├─ 摄像头、IMU、TOF 的低速寄存器初始化
│  ├─ 参数 shadow/校验/原子 commit
│  ├─ 路径表、任务顺序、诊断、UART CLI
│  └─ 心跳与上电自检
├─ 纯 RTL 实时数据面
│  ├─ DVP、DDR、HDMI、RGMII/UDP
│  ├─ 编码器、轮速估计、PI/PWM
│  ├─ 路径段执行和局部绕障
│  └─ 跨时钟域整记录 FIFO
└─ 纯 RTL 安全平面（拥有最终裁决权）
   ├─ 物理急停、测距减速/停车、限幅
   ├─ 网络和软核看门狗
   └─ PWM/驱动使能最终门控
```

CPU 不能直接写 PWM、方向或驱动使能，也不能释放物理急停。CPU 卡死后，测距停车、控制看门狗和最终输出门控仍必须工作。

## 3. 目录

```text
picorv32_robot_soc/
├─ Scripts/run_template_tests.ps1     # 固件解释执行与两组 RTL 一键回归
├─ common/
│  ├─ firmware/build_firmware.py      # 无外部 GCC 的首板固件生成与解释执行自检
│  ├─ rtl/robot_service_regs.v        # 开放 Wishbone 最小寄存器组
│  ├─ rtl/softcore_service_watchdog.v # 服务核失联锁存（接入正式工程时使用）
│  └─ sim/                            # Wishbone/看门狗测试
└─ boards/
   ├─ tang_primer_25k/tang_primer_25k_picorv32/
   │  ├─ tang_primer_25k_picorv32.gprj
   │  ├─ config/                      # 25K RV32IMC 配置
   │  ├─ constraints/                 # 已实板核对的 50 MHz/UART/S1 引脚
   │  ├─ Scripts/                     # IP准备、构建和串口调试
   │  └─ src/                         # 25K 板级顶层
   └─ acg720_138k/                    # 138K 迁移契约；不猜测未核对引脚
```

`vendor_ip/` 由准备脚本从本机已安装的官方 Gowin IDE 复制，已在 `.gitignore` 中排除；这避免在仓库重复保存高云交付源码，也保留了可复现构建入口。

## 4. 25K 已验证配置

- 器件：`GW5A-LV25MG121NC1/I0`（GW5A-25B）。
- CPU：RV32IMC、counter、interrupt、timer。
- 时钟：50 MHz。
- 存储：16 KiB ITCM、8 KiB DTCM，从 ITCM 启动。
- 外设：Simple UART + Open Wishbone；调试 JTAG、GPIO、I²C、SPI、AHB 均关闭。
- UART：115200 8-N-1。
- 固件：972 字节，启动时真实写/读 `0x2000_F000`，并打印运行时 cycle CSR。

高云 1.9.12.03 的 `gowin_picorv32` 自定义插件无法通过标准 `gw_sh create_ipc/set_property/generate_target` 完整生成；普通 FIFO IP 在同一环境可正常生成。`prepare_vendor_ip.ps1` 因而从本机官方安装目录准备所需 RTL 和配置，并对该版本官方模板做两处受控修正：

1. `mem_ready/mem_rdata` 改为与连续赋值匹配的 `wire`；
2. 内部 16 拍复位移位器由下降沿改为上升沿，消除复位释放到 CPU 的半周期路径。

第二项修复前实测 Fmax 为 43.311 MHz；修复后为 **52.803 MHz**，50 MHz setup/hold 违例端点均为 0。脚本会先核对模板中的精确文本，版本不一致时拒绝盲改。

## 5. 构建、烧录与串口调试

先在模板根目录执行不依赖开发板的回归：

```powershell
.\Scripts\run_template_tests.ps1
```

脚本会重建首板固件，并运行 Wishbone 寄存器组和软核失联看门狗测试。

进入 25K 工程目录：

```powershell
.\Scripts\prepare_vendor_ip.ps1
& 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe' Scripts\run_gowin.tcl
```

仅写 SRAM，不修改启动 Flash：

```powershell
& 'D:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe' `
  --device GW5A-25B --operation_index 2 `
  --fsFile '<工程绝对路径>\impl\pnr\tang_primer_25k_picorv32.fs' `
  --cable-index 4
```

双向串口测试：

```powershell
.\Scripts\serial_debug.ps1 -Port COM5
```

命令及运行时响应：

| 命令 | 关键响应 |
|---|---|
| `P` | `PONG;CORE=PICORV32;WB=PASS` |
| `I` | 从 Wishbone 读取 `ID=0x52424F54` |
| `S` | scratch 回读和动态 cycle 值 |
| `?` | 命令帮助 |

本次实板四条命令均通过；这属于 UART 运行时调试。当前电脑未安装 GMD、RISC-V GCC/GDB/OpenOCD，因此尚未提供 C 源码断点/单步调试；正式应用固件阶段再安装高云 SDK/GMD，不应把这一项与已完成的串口调试混淆。

## 6. Wishbone ABI

| 地址 | 模块 | 当前状态 |
|---|---|---|
| `0x2000_0000` | 系统 ID、版本、能力、CPU heartbeat | 已实现最小寄存器 |
| `0x2000_1000` | 机器人状态一致性快照 | 138K 接入 |
| `0x2000_2000` | 参数 shadow/校验/原子 commit | 138K 接入 |
| `0x2000_3000` | 双 bank 路径控制 | 138K 接入 |
| `0x2000_4000` | 传感器初始化 mailbox | 138K 接入 |
| `0x2000_F000` | scratch、总线事务/错误计数 | 已实现并实板回读 |
| `0x2001_0000` | 路径 BRAM 窗口 | 138K 接入 |

所有未定义或未对齐访问也会在有限时间 ACK；非法读返回 `0xDEADBEEF` 并计错，避免 CPU 因软件地址错误永久挂住 Wishbone。

## 7. 迁移到 ACG720 138K

迁移时保持不变：RV32IMC ISA、固件 ABI、Wishbone 地址、UART 命令、服务核/实时层/安全层职责。需要替换的只有：

1. 器件改为项目已确认的 `GW5AT-LV138PG484AC1/I0`，重新生成或准备 138K 对应 IP；
2. 使用 ACG720 官方工程的真实时钟、复位、DDR、RGMII 和摄像头约束；
3. 扩大 ITCM/DTCM 和路径双 bank 深度；
4. 把 `robot_service_regs` 接到真实状态快照、参数提交和路径 provider；
5. 增加独立软核看门狗，超时只能禁止新任务并请求中止，不能绕过硬件安全层；
6. 完成架空车轮、物理急停和真实传感器失效测试后，才允许接地驱动。

ACG720 电机、编码器、IMU、TOF 和维护 UART 的最终管脚尚未在仓库中被硬件资料确认，因此模板有意不提供猜测 CST。

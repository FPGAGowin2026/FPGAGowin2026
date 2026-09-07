# Choice3 Robot — GW5AT-138B

这是独立的选题三 138K 工程，精确目标为：

- Device：`GW5AT-138B`
- Part：`GW5AT-LV138PG484AC1/I0`
- Gowin database：`gw5at138b-012`
- 顶层：`choice3_robot_138k_top`

工程只承诺 Icarus 功能回归和 Gowin `run syn`。由于当前没有经原理图核对的
ACG720 引脚和板级时序数据，工程故意不含 CST/SDC，不执行 PnR，不生成或宣称
可下载 bitstream，也不声称完成板级时序收敛。

## 实现链

- OV5640：SCCB 上电/ID/寄存器初始化，DVP RGB565 采集，160x120 RGB444
  双缓冲，再 4 倍显示为 640x480。
- JY61P：UART 校验帧解析，原始姿态换算整数角度，跨时钟快照后叠加
  `R/P/Y` OSD。
- TF-Luna + 4 路 IR：距离帧校验、强度/量程门限、freshness、自动减速、停车
  和绕障状态机。
- 四轮闭环：4 路正交编码器、速度估计、独立 PI、方向死区、TB6612
  `PWM/IN1/IN2/STBY`。
- 导航：前进、后退、转向、定点旋转、起终点路径状态机、障碍绕行。
- DVI：GW5AT `PLL` 产生 125/25/25 MHz，VGA RGB/OSD 经 TMDS 编码、
  `OSER10` 和 `ELVDS_OBUF` 输出。
- 软核：Gowin PicoRV32 RV32IMC，32 KiB ITCM、16 KiB DTCM、115200 UART、
  开放 Wishbone 服务寄存器和独立心跳看门狗。

## 安全执行边界

CPU 和外部 AI/主控都只能提交 `ARM/DISARM/CLEAR/ROUTE/MANUAL` 等高层请求，
不能写 PWM、方向或 STBY。命令经仲裁后仍必须通过传感器健康、急停、编码器、
测距、导航和电机死区逻辑。最终顶层还用原始急停与 CPU 心跳再次组合门控全部
物理电机输出。

- CPU 心跳超时：5,000,000 个 50 MHz 周期，即 100 ms；故障锁存，只有外部
  安全复位可清除。
- 外部命令 lease：10,000,000 个 50 MHz 周期，即 200 ms；到期产生撤武装。
- CPU 与外部控制器同周期发出非撤武装动作：判为所有权冲突并撤武装。
- 急停无效、CPU 未 alive 或 CPU 看门狗故障：外部请求被拒绝并撤销 lease。
- 最终物理门：`reset released && estop_n && cpu_alive && !watchdog_fault &&
  motor_enable`。

## AI/主控扩展接口

顶层提供协议无关的同步逻辑接口，供后续 UART/SPI/RGMII/PCIe 桥接。这里没有
虚构桥接器引脚。

| 信号 | 方向 | 含义 |
|---|---|---|
| `ext_cmd_valid_i/ready_o` | in/out | 单周期 valid-ready 事务 |
| `ext_cmd_sequence_i[31:0]` | in | 严格递增序号，支持 `ffffffff -> 00000000` 回绕 |
| `ext_cmd_opcode_i[3:0]` | in | 高层命令 |
| `ext_cmd_motion_i[2:0]` | in | MANUAL 动作 0..6，其余 opcode 必须为 0 |
| `accepted/rejected/timeout` | out | 单周期验收结果 |
| `reject_reason[2:0]` | out | 1 序号、2 opcode、3 payload、4 safety、5 无 lease |
| `lease_active/last_sequence` | out | 当前 lease 与最后接受序号 |

Opcode：`0 KEEPALIVE`、`1 ARM`、`2 DISARM`、`3 CLEAR`、`4 ROUTE_START`、
`5 MANUAL`、`6 STOP`。只有接受的命令才推进最后序号；重复、倒退、跨度达到
半个 32 位序号空间、未知命令和非法载荷全部拒绝。

PicoRV32 可在 `0x2000_1014..0x2000_1024` 只读 lease 状态、最后序号、接受、
拒绝和超时计数。服务 ABI 版本为 `0x0003_0000`，capabilities bit 6 表示外部
受保护命令接口。

## 回归与综合

```powershell
pwsh -File .\Scripts\run_sim.ps1
pwsh -File .\Scripts\run_gowin.ps1
```

`run_sim.ps1` 会自动发现 `sim/tb_*.sv` 并要求每个 testbench 恰好出现一次
PASS，同时解释执行 PicoRV32 固件并校验生成 HEX。`run_gowin.ps1` 会在执行前后
校验精确 part/database、无 CST/SDC、Tcl 只有 `run syn`、日志 error、报告器件、
资源容量以及 OSD 综合层级。

实际结果见 `RESULTS.md` 和 `reports/*.json`。

`Scripts/write_evidence_manifest.ps1` 生成 `reports/evidence_manifest.json`，用 SHA-256 绑定当前 RTL/TB/GPRJ/脚本/固件 HEX 与仿真、纯综合报告和综合网表；它不改变本工程“无 CST/SDC、无 PnR/位流”的边界。

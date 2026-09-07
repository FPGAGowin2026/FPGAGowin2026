# 138K 验收结果

验收时间：2026-08-29（Asia/Shanghai）。Gowin 工具：V1.9.12.03 x64。

## 门禁矩阵

| 门禁 | 可执行证据 | 结果 |
|---|---|---|
| 精确器件 | GPRJ、Tcl 自动检查、综合报告 Part/Device/Version | PASS |
| 全部本地 testbench | `Scripts/run_sim.ps1` 自动发现并逐一检查唯一 PASS | PASS，9/9 |
| PicoRV32 固件 | 机器码生成、解释执行、启动输出、UART ABI=3、HEX SHA256 | PASS |
| AI/主控序号与超时 | duplicate/stale/非法载荷/回绕/安全阻断/lease 超时/冲突 | PASS |
| 真实传感器/闭环/软核/DVI 链 | 完整顶层 `run syn`，无 black box error | PASS |
| 姿态 OSD 未删除 | 资源 XML 中 `u_vga/u_attitude_osd` 为 4 Register、247 LUT | PASS |
| Gowin 错误 | 最终 `gowin_syn.log` 中 ERROR/FATAL 行数 | PASS，0 |
| PnR/伪约束 | 无 CST/SDC，Tcl 无 `run pnr`，`impl` 仅 `gwsynthesis` | PASS，未运行 PnR |

## Icarus 全回归

| Testbench | 覆盖内容 | 结果 |
|---|---|---|
| `tb_choice3_control_core` | 健康门、四轮闭环、运动、避障、超时 fail-safe | PASS |
| `tb_choice3_navigation_route` | 第一航段、定点旋转、第二航段、`route_done` 锁存与停止清除 | PASS |
| `tb_choice3_video_pipeline` | DVP、双缓冲、VGA、TMDS 数据/控制期 | PASS |
| `tb_choice3_sensor_guards` | IMU/freshness 校验与超时 | PASS |
| `tb_choice3_sensor_protocols` | 50 MHz 下 JY61P 9600 与 TF-Luna 115200 真实逐位帧、坏校验与恢复 | PASS |
| `tb_choice3_attitude_osd` | raw-to-degree、正负角、R/P/Y 像素与颜色 | PASS |
| `tb_choice3_external_command_guard` | 序号、防重放、回绕、lease、safety、仲裁冲突 | PASS |
| `tb_choice3_service_regs` | Wishbone 单次 ACK、命令、扩展状态、非法访问 | PASS |
| `tb_choice3_softcore_watchdog` | 首次心跳授权、精确超时、故障锁存、单周期 abort、安全复位恢复 | PASS |

固件：1,648 bytes / 32 KiB ITCM，SHA256
`b7192fc957513b5ffcae0d1c27e64517016420fca9c332a7bca02f40e0d5fa5c`。

## Gowin `run syn` 资源

精确报告目标：`GW5AT-LV138PG484AC1/I0`，Device `GW5AT-138`，Version `B`。

| 资源 | 使用 / 容量 |
|---|---:|
| Logic | 9,809 / 138,240 |
| LUT（Logic 口径） | 7,903 |
| ALU | 1,906 |
| Register | 5,259 / 139,095 |
| BSRAM | 61 / 340 |
| MULTALU27X18 | 4 |
| MULT27X36 | 6 |

关键层级：PicoRV32 使用 24 BSRAM；摄像头双缓冲使用 36 BSRAM；SCCB ROM 使用
1 BSRAM；`u_attitude_osd` 使用 4 Register、247 LUT；外部命令 guard 使用
176 Register、125 ALU、104 LUT。

最终综合有 49 条 warning，全部来自本地随工程封装的官方 Gowin PicoRV32 vendor
源：CSR compilation-unit parameter 兼容提示，以及官方加密块的位宽/full_case/
parallel_case 提示。用户 RTL 的截断 warning 已消除，最终没有 NL0002 OSD swept，
脚本会拒绝任何非该 vendor 白名单的 warning。

## 证据文件

- `reports/simulation.log`
- `reports/simulation_summary.json`
- `reports/gowin_syn.log`
- `reports/synthesis_summary.json`
- `reports/evidence_manifest.json`
- `impl/gwsynthesis/choice3_robot_138k_syn.rpt.html`
- `impl/gwsynthesis/choice3_robot_138k_syn_resource.html`
- `impl/gwsynthesis/choice3_robot_138k_syn_rsc.xml`
- `impl/gwsynthesis/choice3_robot_138k.vg`

本次没有 PnR、STA 或 bitstream，因此这里不提供虚假的引脚、频率收敛或板级 PASS。

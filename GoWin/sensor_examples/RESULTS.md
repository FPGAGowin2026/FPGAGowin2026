# 29 个传感器工程最终构建与审计结果

验证日期：2026-08-29

## 工具、目标与验证门禁

| 项目 | 结果 |
|---|---|
| FPGA | `GW5A-LV25MG121NC1/I0` |
| Gowin family/database | `GW5A-25B` / `gw5a25b-003` / Device B |
| Gowin IDE/CLI | V1.9.12.03 64-bit |
| RTL 编译/仿真 | Icarus Verilog 12.0 devel / `vvp` |
| 独立工程 | 29/29 |
| Verilog-2001 生产顶层编译 | 29/29 PASS |
| 自检查动态仿真 | 29/29，各唯一一条 `PASS:` |
| Gowin 综合/PnR/位流 | 29/29 PASS |
| I/O 端口上限 | 29/29 ≤ MG121 的 69 |
| setup/hold violated endpoints | 29/29 均为 0/0 |

多数控制域约束为 27 MHz。OV5640 的 XCLK/配置域为 24 MHz，独立的 DVP PCLK 域按器件规格上限 96 MHz 保守约束；其板外 input delay 必须根据最终模组与 PCB 实测补齐。

## PnR 资源与时序

| 工程 | Logic | Register | I/O Port | Actual Fmax (MHz) |
|---|---:|---:|---:|---:|
| `digital_ir_proximity` | 6 | 6 | 5 | 597.973 |
| `gnss_nmea_uart` | 197 | 129 | 26 | 172.675 |
| `gy39_uart` | 316 | 288 | 40 | 167.147 |
| `hc_sr04` | 322 | 162 | 39 | 124.636 |
| `icm20602_spi` | 505 | 554 | 29 | 122.745 |
| `icm42688p_spi` | 337 | 326 | 28 | 141.531 |
| `imu660ra_spi` | 377 | 382 | 30 | 153.131 |
| `imu660rx_spi` | 440 | 416 | 32 | 136.453 |
| `imu963ra_spi` | 455 | 409 | 30 | 151.629 |
| `ina219_i2c` | 285 | 173 | 56 | 189.004 |
| `itr9606_pulse_speed` | 270 | 157 | 38 | 151.068 |
| `jy61p_uart` | 211 | 199 | 23 | 160.695 |
| `lis3mdl_spi` | 243 | 240 | 57 | 157.130 |
| `maxsonar_uart` | 170 | 95 | 22 | 198.423 |
| `mechanical_bumper` | 118 | 73 | 10 | 263.882 |
| `menc15a_spi` | 192 | 184 | 43 | 138.628 |
| `mpu6050_i2c` | 493 | 395 | 28 | 161.844 |
| `mt6816_spi` | 223 | 141 | 26 | 198.578 |
| `mt9v03x_dvp` | 190 | 113 | 63 | 131.473 |
| `multi_ir_array` | 265 | 145 | 19 | 204.553 |
| `ov5640_dvp` | 463 | 224 | 56 | 136.285 |
| `ov7725_dvp` | 204 | 134 | 58 | 165.482 |
| `quadrature_encoder` | 138 | 44 | 39 | 147.386 |
| `scc8660_dvp` | 204 | 131 | 64 | 126.658 |
| `tcrt5000` | 50 | 37 | 38 | 160.928 |
| `tf_luna_uart` | 142 | 154 | 53 | 135.408 |
| `tsl1401_adc_frontend` | 133 | 61 | 42 | 156.985 |
| `vl53l0x_i2c` | 996 | 441 | 45 | 138.085 |
| `vl53l1x_i2c` | 711 | 258 | 28 | 134.915 |

`Actual Fmax` 是 Gowin 对当前已约束内部路径给出的报告值，不是建议的传感器接口速率。没有最终 PCB 的板外时序与实测传感器数据，因此不能从这张表推导“实物已经通信”。

## 本轮额外修正与回归

- TSL1401：补齐每行第 129 个终止时钟；ADC 超时后仍补足到 129；上电首轮只清洗不确定数据；默认强制至少 20 us `tqt`。
- OV5640：按 `0x4300=0x6F` 的低字节优先 DVP 顺序组帧；配置状态经双触发同步后再释放 PCLK 域复位。
- OV5640 SCCB：第九位按 Don’t-Care，不误判为 I²C NACK；SCL 超时仍报错并实际验证恢复路径。
- OV5640 初始化：先写 `0x3103=0x11` 再软件复位；TB 使用独立黄金寄存器表；`0x4740=0x23` 明确 PCLK/HREF/VSYNC 高有效。
- OV5640 STA：PCLK 从错误的 24 MHz 假设改为 96 MHz 上限约束，并重新完成综合、PnR、位流和 0/0 时序门禁。
- 位宽审计：GNSS、ICM20602、IMU660RA/RX、IMU963RA、MaxSonar、MENC15A、MT6816 的算术/计数位宽已显式化；10 个本轮受影响工程的 Gowin 日志均为 0 条告警。
- MPU6050：新增 `PWR_MGMT_2=0x00` 显式六轴使能，退出睡眠后默认等待约 30 ms；独立 TB 校验精确配置顺序、等待下界、未就绪不发布和 NACK 路径。
- VL53L0X：TB 不再调用 DUT 的调参函数，改用独立 80 项黄金表；新增 `range_status_raw`，并把 `distance_valid` 限定为新结果 ready 脉冲而非质量合格证明。
- 原先因 MG121 I/O 超限的 6 项已改为窄生产顶层；完整 core 验证仍在 TB 中。
- 多路红外阵列：同步器复位值改为“对应极性的未触发原始电平”，消除 `ACTIVE_LOW=0` 且去抖周期为 1 时的上电伪事件；新增 active-high 动态回归并重跑 29/29 仿真审计与该工程 Gowin PnR。
- 审计清理：删除 `build/sim` 中 6 个不被正式脚本引用的 `*_new.vvp` 临时产物；目录现仅保留 29 个生产顶层编译产物和 29 个自检 TB 产物，共 58 个 `.vvp`。

## 集成、运行时与审计风险清单（9 项）

1. **范围边界：** 本报告覆盖一级目录中的 29 个 `sensor_example.gprj`。`sensor_examples` 递归共有 114 个 `.gprj`，其余 85 个位于 `acg720_tutorial`，不属于本次 29/29 门禁，不能宣称递归 114 个工程全部通过。
2. **顶层重名：** 29 个 `src/top.v` 都定义 `sensor_example_top`。多传感器系统只能引入核心 RTL/公共总线 RTL，或另写系统顶层，不能同时加入多个示例 `top.v`。
3. **UART 新鲜度：** GNSS、GY39、JY61P、MaxSonar、TF-Luna 等核心仍需系统级最后有效帧时间、失联超时、连续坏帧计数和 `stale/fault` 门控。
4. **I²C 运行期健康：** INA219、VL53L0X、VL53L1X 的部分运行期 NACK/超时路径会置 `bus_error`，但未必撤销 `initialized/sensor_ok`；需要统一连续错误阈值、自动重新初始化和数据新鲜度判定。
5. **SPI 运行期健康：** ICM42688P、LIS3MDL 等主要在启动阶段验证身份；需增加周期 WHO/config 回读、全零/全一及器件无效码拒绝、连续错误计数和自动重启。
6. **VL53 多机地址：** 当前 VL53L0X/VL53L1X 固定使用默认 `0x29`，未实现 XSHUT 编排和重新寻址；多机必须分总线或补齐地址配置。
7. **I²C 总线恢复：** 公共 I²C RTL 支持开漏、ACK 和 clock-stretch 超时，但不支持多主仲裁及 SDA 卡死后的 9 脉冲恢复。
8. **DVP 板外时序：** 当前只证明内部时钟约束和 STA；最终 PCB 仍需依据模组/走线补 `set_input_delay/set_output_delay`、CDC/异步 FIFO，并重新实现与上板验证。
9. **证据边界：** `audit_results.ps1` 尚未用源码/TB/约束哈希绑定输入与构建产物；29项也没有板级 CST、下载记录和真实传感器数据。当前 PASS 只证明仿真和离线实现，不能冒充实物验证。

## 证据位置

每个工程都保留：

- `reports/simulation.log`：Verilog-2001 顶层编译和动态自检终态。
- `reports/gowin.log`：Gowin 综合、PnR、位流生成日志。
- `impl/gwsynthesis/sensor_example_syn.rpt.html`：综合报告。
- `impl/pnr/sensor_example.rpt.txt`：器件、版本与资源报告。
- `impl/pnr/sensor_example_tr_content.html`：setup/hold 与 Fmax 报告。
- `impl/pnr/sensor_example.fs`：离线位流产物。

复核：

```powershell
pwsh -File .\scripts\run_all_sim.ps1
pwsh -File .\scripts\run_all_gowin.ps1
pwsh -File .\scripts\audit_results.ps1
```

预期终态：

```text
AUDIT PASS：29/29 仿真、综合、PnR、位流、I/O 上限和 setup/hold 门禁均通过。
```

## 结论边界

本结果证明一级目录中的 29 个独立 RTL 工程在指定 GW5A-25B 器件上的仿真与离线实现门禁通过；它不覆盖 `acg720_tutorial` 中的另外 85 个 Gowin 工程，因此不能表述为递归 114 个工程全部通过。29 个示例 `top.v` 均定义 `sensor_example_top`，多传感器集成时不得同时加入。该结果也不证明无 CST 的独立位流可以直接烧录，更不证明未连接的传感器、摄像头板外时序、模拟前端或整机抗干扰已经实测。实板调试结果必须引用板级 CST、下载记录和 UART 日志，并区分真实外设数据、无设备故障路径和片上激励。

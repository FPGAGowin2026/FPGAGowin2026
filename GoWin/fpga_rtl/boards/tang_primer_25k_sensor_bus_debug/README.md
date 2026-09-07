# Tang Primer 25K 传感器总线无设备诊断

本工程在 `GW5A-LV25MG121NC1/I0` 上同时实例化 4 个 I2C 传感器核心和 8 个 SPI 传感器核心，用于验证传感器驱动的故障路径、总线事务确实发生以及 UART 可观测性。它是工程诊断，不是假装已有传感器实物。

## 诊断边界

- I2C 使用八个真实 Dock GPIO 和芯片内部上拉。保持这些引脚完全悬空时，四个核心会发送真实 START、地址和时钟，并在第九拍读取高电平，得到地址 NACK/`bus_error`。
- SPI 的八路 MISO 在 FPGA 内固定为 `1'b1`；各核心的 CS/SCLK/MOSI 状态机真实运行，但没有从封装引脚输出。诊断检查完整 CS 事务，并要求 WHO_AM_I 或帧完整性逻辑拒绝全 1 响应。
- `BUS_BIST=PASS` 只证明“无设备/全 1 故障路径”和 UART 调试链路，不证明任何真实传感器的数据正确，也不证明外部 SPI 电气连接。
- 做无设备测试前，必须确认下表八个 I2C GPIO 未接传感器、逻辑分析仪或其他主动器件。

## 核心与位图

| 位 | I2C 核心 | SCL | SDA | PASS 条件 |
|---:|---|---|---|---|
| 0 | INA219 | G4 | H2 | 地址 NACK 置位 `bus_error` |
| 1 | MPU6050 | H1 | J7 | 地址 NACK 置位 `bus_error` |
| 2 | VL53L0X | K7 | L8 | 地址 NACK 置位 `bus_error` |
| 3 | VL53L1X | L7 | K10 | 地址 NACK 置位 `bus_error` |

| 位 | SPI 核心 | 全 1 拒绝判据 |
|---:|---|---|
| 0 | ICM20602 | ID 错误脉冲被锁存 |
| 1 | ICM42688P | WHO_AM_I 事务完成且 `sensor_ok=0` |
| 2 | IMU660RA | ID 错误脉冲被锁存 |
| 3 | IMU660RX | 两种候选 ID 均失败后错误脉冲被锁存 |
| 4 | IMU963RA | 复位写事务之后，ID 错误脉冲被锁存 |
| 5 | LIS3MDL | WHO_AM_I 事务完成且 `sensor_ok=0` |
| 6 | MENC15A | 全 1 位置帧错误脉冲被锁存 |
| 7 | MT6816 | 三次读事务后的全 1/奇偶校验错误被锁存 |

固定板级管脚为：E2 50 MHz 时钟、H11 高有效复位、B3 UART RX、C3 UART TX。串口格式为 115200、8-N-1。

## UART 协议

上电横幅：

```text
GW5A25 SENSOR BUS DEBUG READY
```

| 命令 | 正常无设备响应 |
|---|---|
| `P` | `PONG;BUS_DEBUG=READY` |
| `B` | `BUS_BIST=PASS;I2C=4/4;SPI=8/8;MODE=NO_DEVICE` |
| `D` | `I2C_ERR=0xF;SPI_TXN=0xFF;SPI_REJECT=0xFF` |
| 其他 | `CMDS:P,B,D` |

若过早发送 `B`，计数可能暂时小于 4 或 8；50 MHz 默认参数下，上电数毫秒后即可稳定。

## 构建与验证

在本目录执行：

```powershell
pwsh -File .\Scripts\run_sim.ps1
pwsh -File .\Scripts\run_gowin.ps1
```

仿真使用 `tri1` 建模四组悬空 I2C 总线，并独立检查每组 SCL/SDA 都出现过低电平、四个 NACK 错误位、八个完整 SPI CS 事务、八个拒绝位以及 `P/B/D` UART 文本。

综合脚本使用 Gowin V1.9.12.03 x64，目标器件固定为 `GW5A-LV25MG121NC1/I0`（Device B，数据库 `gw5a25b-003`）。位流路径为：

```text
impl/pnr/tang_primer_25k_sensor_bus_debug.fs
```

当前已执行结果与证据索引见 [`RESULTS.md`](RESULTS.md)。

## 实板步骤与已执行结果

1. 断开上表八个 I2C 引脚上的所有外设，确认没有 5 V 信号。
2. 用 Gowin Programmer 扫描 JTAG，确认 IDCODE 对应当前 25K 板。
3. 只用 SRAM Program 写入 `.fs`；不要在首次诊断时改写外部启动 Flash。
4. 等待约 250 ms，然后执行：

   ```powershell
   pwsh -File .\Scripts\serial_debug.ps1 -Port COM5
   ```

5. 脚本会验证 `P/B/D` 三条响应，并把原始结论写入 `reports/serial_debug.log`。

本目录的构建流程不会自动调用 Programmer。2026-08-27 已协调后完成一次 SRAM Program 和 COM5 `P/B/D` 回归，三条命令全部通过；启动 Flash 未修改。详情与边界见 [`RESULTS.md`](RESULTS.md)。

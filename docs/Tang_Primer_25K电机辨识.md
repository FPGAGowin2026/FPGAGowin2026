# Tang Primer 25K 单电机辨识

目标：确认四个驱动通道对应的车轮位置、正向驱动对应的转向及编码器计数符号。此工程为开环点动，不运行麦轮控制，不继承旧 M1～M4 轮序。

## 接线与工程

以下均为 FPGA 封装球位，不是排针序号。电机/编码器接线由用户提供；时钟、按键与 USB 调试 UART 沿用工作区既有 Tang Primer 25K Dock 约束。

| 通道 | 驱动板 | IN1 | IN2 | 编码器 A | 编码器 B |
|---|---|---|---|---|---|
| M1 | 上层 A | K2 | L1 | K4 | G1 |
| M2 | 上层 B | C2 | F1 | A1 | D1 |
| M3 | 下层 A | B2 | F2 | E1 | E3 |
| M4 | 下层 B | K1 | L2 | J4 | G2 |

时钟 E2、复位 H11（高有效）、UART RX B3、TX C3；I/O 按 LVCMOS33 约束。电机电源由驱动板独立供电并与 FPGA 共地，编码器输出须兼容 3.3 V 输入；编码器可用 5 V 供电不代表 AB 可以直接接 5 V。

- 工程：`FPGA_ws/boards/tang_primer25k_motor_id/motor_id.gprj`
- 顶层：`tang_primer25k_motor_id`
- 当前目标：`GW5A-LV25MG121NC1/I0`，Device Version **B**，沿用本地历史25K工程；下载前用实际器件/Programmer扫描结果核对，A版不能直接使用该码流，应修改目标并重新构建。
- 构建：在 Gowin Tcl 环境执行 `source /实际路径/FPGA_ws/boards/tang_primer25k_motor_id/build.tcl`。脚本设置 SystemVerilog、顶层以及 SSPI/CPU/I2C 引脚 GPIO 复用，然后执行综合、布局布线。仅打开 gprj 手动构建时也必须设置这些选项，不能依赖未跟踪的 `.gprj.user`。
- 码流：`FPGA_ws/boards/tang_primer25k_motor_id/impl/pnr/motor_id.fs`，产物由 Git 忽略。

## Windows 端接续（2026-09-20 交接）

当前下一步是实物辨识，尚未得到任何 M1～M4 与实际轮位的对应关系。先完成本页记录表，再配置四轮闭环；不要烧录旧 `tang_primer25k_at8236_top` 固定动作工程替代本工程。

### 先把当前文件带到 Windows

本次工作未提交、未推送；父项目与 `FPGA_ws` 是两个独立 Git 仓库，父项目忽略整个 `FPGA_ws/`。仅拉取父仓库、仅复制 docs 或使用仅包含已跟踪文件的 Git 归档，都不能完整带走当前成果。

推荐将当前 `docs/` 和完整 `FPGA_ws/` 工作树复制到 Windows 的同一个 `FPGA_GOWIN/` 目录下，保留相对层级。保留未跟踪文件，并显式带上 `FPGA_ws/boards/tang_primer25k_motor_id/impl/pnr/motor_id.fs`；它被 Git 忽略，不会随普通提交传输。若目标端已有修改，先备份/比较，勿直接覆盖。

若只转移辨识工程，至少需要以下内容；后续继续麦轮闭环开发时仍应带上完整工作树：

```text
FPGA_GOWIN/
  docs/
  FPGA_ws/
    boards/tang_primer25k_motor_id/  # gprj、cst、sdc、build.tcl，以及生成的 fs
    rtl/system/at8236/tang_primer25k_motor_id.sv
    rtl/motor/at8236/at8236_motor_channel.sv
    rtl/motor/encoder/quadrature_decoder.v
    rtl/protocol/at8236/uart_rx.sv
    rtl/protocol/at8236/uart_tx.sv
    rtl/common/uart/packet_uart_tx.v
    sim/at8236/tb_motor_id.sv
    Scripts/run_motor_id_sim.sh
```

无需搬运 Linux 下的 Gowin 可执行程序或 `/tmp/motor-iverilog` 临时仿真器；Windows 需使用本机适用的工具。`impl/` 内旧报告可能带 Linux 绝对路径，不是新机器已构建的证据。

当前 Linux 生成的 `motor_id.fs` SHA256 为：

```text
87a0b57fcab6d30b4c14ee7145d20516838ae894a61c415f39bc4b16f5b067a8
```

Windows PowerShell 校验（路径举例，替换为实际目录）：

```powershell
Get-FileHash 'D:\FPGA_GOWIN\FPGA_ws\boards\tang_primer25k_motor_id\impl\pnr\motor_id.fs' -Algorithm SHA256
```

此哈希用于确认直接复制的码流一致；重新构建后的码流可能不同，应记录新产物，不要求匹配旧哈希。

### Windows 构建与下载入口

已有对应 B 版器件的码流时，可直接按下节操作进行 SRAM 下载。需要修改占空比、点动时间或目标器件时，再重新构建。

本轮验证工具为 Gowin V1.9.12.03。Windows 本机准备支持目标器件的 Gowin IDE/Programmer 及板载下载器驱动，在设备管理器中确认下载器和串口；COM 号现场确定，不沿用历史 COM5。

PowerShell 调用 Windows 版 `gw_sh.exe` 的示例：

```powershell
& 'C:\实际安装目录\IDE\bin\gw_sh.exe' 'D:\FPGA_GOWIN\FPGA_ws\boards\tang_primer25k_motor_id\build.tcl'
```

脚本含 `exit 0`，建议启动独立 `gw_sh.exe` 进程执行。路径只是占位示例，尚未在 Windows 运行。脚本内部使用项目相对路径，并完整设置顶层、语言与 GPIO 复用；不依赖 Linux 环境变量或 `.gprj.user`。若手动在 GUI 操作，也应核对上述选项。

只使用本工程的 `impl/pnr/motor_id.fs`，不要从旧案例或其他同名 `impl/` 目录挑选码流。下载前核对实际器件版本；当前没有 A 版构建结果。

### 交给下一次会话的任务

可直接使用以下说明继续：

> 请先读 docs/handoff.md 和 docs/Tang_Primer_25K电机辨识.md。当前在 Windows 上验证 Tang Primer 25K＋四路 AT8236，接线表已经确定，但四轮顺序和极性未知。motor_id 是已在 Linux 完成仿真、综合、布局布线和出码流的独立开环辨识工程，尚未上板。先检查本机文件是否完整、实际器件版本和 COM 端口，再协助 SRAM 下载、逐通道点动及编码器记录。保留现有修改，不使用旧固定动作顶层，不假定旧 M1～M4 轮序。得到四轮记录后再配置 mecanum_motor_control 的轮序和极性。

测试时记录：Windows 工具版本、器件型号/版本、COM 号、下载文件路径/哈希、串口日志及下方四轮表。记录数字指令前后的计数与实际转向；没有观察到的项目保持“待测”。

## 上板操作

1. 四轮架空，初次下载时断开电机电源。用 Gowin Programmer 扫描确认器件后，将上述 `.fs` 下载到 **SRAM** 进行临时验证。当前没有自动烧录脚本，不需要先写 Flash。
2. 打开板载调试串口：**115200，8N1，无流控，文本/ASCII 发送，关闭自动循环发送**。上电约 10 ms 初始化后保持停止，约每 100 ms 收到一行计数。复位按键清除计数和点动状态。
3. 电机电源断开时，手转一个轮子，观察哪列计数改变，先填写通道与车轮位置。观察者站在车后、面朝车头，统一左右方向。
4. 接通电机电源，单次发送 `1`，观察 M1 对应车轮的转动方向；等停止后依次发送 `2`、`3`、`4`。每次默认 **20% 占空比、20 kHz、300 ms**，运行中忽略新运动命令，不延长本次点动。一直循环发送仍可能在结束后触发下一次，所以必须关闭自动发送。
5. 需要反向核实时，分别发送 `a`、`b`、`c`、`d`。随时发送 `0` 结束当前点动；按住复位会禁止输出。这里是功能测试停止，不构成独立硬件急停。

| 操作 | M1 | M2 | M3 | M4 |
|---|---|---|---|---|
| 正向点动（IN1=PWM） | `1` | `2` | `3` | `4` |
| 反向点动（IN2=PWM） | `a` | `b` | `c` | `d` |

发送回车/换行等其他字符不产生动作。占空比较低时，电机可能不能克服静摩擦，不能仅凭“不转”判定通道错误。先核对供电与波形；若需要调整，在顶层修改 `JOG_DUTY`，保持 `0 < JOG_DUTY <= PWM_PERIOD`，重新构建。

## 串口读数与记录

每帧固定 39 字节：`状态 M1计数 M2计数 M3计数 M4计数\r\n`。状态 `0` 表示停止，`1～4/a～d` 表示当前点动；四列是未翻转极性的32位有符号累计计数，以八位十六进制输出。

```text
0 00000004 00000000 00000000 FFFFFFFC
```

示例表示停止，M1 为 +4，M4 为 -4。若值大于等于 `80000000`，十进制值减去 `4294967296` 得到有符号计数。比较动作前后差值，而非仅看累计值正负。解码正方向为 AB `00→01→11→10→00`，一周期四计数。上电相位可能带来一个初始计数，不影响通过差值辨识方向。

“前进转向”定义为：该轮安装在车上时，轮底接地点向车后运动的转向。不要用左右两侧“顺时针”作为统一正方向。

| 通道 | 实际轮位（左前/右前/左后/右后） | 发数字后前进/后退 | 编码器差值正/负 |
|---|---|---|---|
| M1 上层A | 待测 | 待测 | 待测 |
| M2 上层B | 待测 | 待测 | 待测 |
| M3 下层A | 待测 | 待测 | 待测 |
| M4 下层B | 待测 | 待测 | 待测 |

确认后才能将 M1～M4 重排为 FL/FR/RL/RR，并设置麦轮模块的 `MOTOR_REVERSE` 和 `ENCODER_REVERSE`。本工程不自动推断轮序。

## 验证证据（2026-09-20）

- `FPGA_ws/Scripts/run_motor_id_sim.sh` 自检通过：真实串口字节接收、四通道正反向隔离、20% PWM计数、到时停止、运动中拒绝切换、停止指令、复位、非法字符及四路计数ASCII回传。
- Gowin V1.9.12.03 在上述25K B目标综合、布局布线与码流生成成功；使用 1420 Logic、687 Register、20 I/O。
- SDC 定义 50 MHz 时钟；未建立外部编码器、UART和驱动板时序模型，不把成功出码流表述为整板时序或实物通过。
- 当前环境未发现 `/dev/ttyUSB*` 或 `/dev/ttyACM*`，未下载或实测电机。四轮对应关系、有效占空比和编码器极性仍待填写。

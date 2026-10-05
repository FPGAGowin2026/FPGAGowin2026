> 当前 BX72 基线统一于 2026-10-05：融合远端 `4e53c06` 的 `BX72 v1` 五类遥测与 `0xD2/0xB/0xA` 映射；协议、部署身份和验证边界以 [统一记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md) 为准。ACG720 保留独立 BXT1 接入。

# 运动控制开发与日志分析

## 当前进度（2026-10-05）

电机辨认已由用户确认完成，结果与运行配置见[BX72 接口](../FPGA_ws/boards/BX72_138K_robot/docs/hardware_interfaces.md)。UDP 速度与停车已实现，当前 296674D6 候选待悬空并发/停车验收，随后验证测速尺度和落地负载；三人交付见[开发计划](后续开发计划.md)。下方辨识方法保留供复查，不作为重复待办。
## 当前板级控制

两板当前采用 OpenCTR 增量速度环及麦轮运动学，具体管脚、方向和串口命令分别见 [ACG720 硬件接口](../FPGA_ws/boards/ACG720_60K_robot/docs/hardware_interfaces.md)、[BX72 硬件接口](../FPGA_ws/boards/BX72_138K_robot/docs/hardware_interfaces.md)。通道到轮位不能跨板复用。

## 商家参数与运动学研究

### 源码与适用范围

商家固件 `OpenCTR_H60V32_FreeRTOS_R20_ABOT_V1.6.0/Robot/ax_robot.h` 将 `ROBOT_TYPE` 设为 `ROBOT_MEC`。`ax_kinematics.c` 的 MEC 分支包含正逆运动学；`ax_robot.c` 每 20 ms 调用一次。ROS 1 仓库 `xtark/src/xtark_robot/src/xtark_robot.cpp` 将 `/cmd_vel` 打包为串口速度指令，并对固件回传的车体速度积分为里程计。FPGA 移植直接复用本仓库已有的 `rtl/motor/mecanum_motor_control.sv`，不移植 ROS 1 通信代码或 STM32 外设调用。

## 算法映射

源固件的轮序 A/B/C/D 按运动学矩阵对应本模块的 FL/FR/RL/RR。车体坐标约定为 x 前、y 左、z 上，正 `wz` 为逆时针。原固件使用 m/s 与 rad/s；RTL 使用 mm/s 与 mrad/s。令 `L=(轴距+轮距)/2`，则四轮目标线速度为：

```text
FL = vx - vy - L*wz/1000
FR = vx + vy + L*wz/1000
RL = vx + vy - L*wz/1000
RR = vx - vy + L*wz/1000

vx = (FL+FR+RL+RR)/4
vy = (-FL+FR+RL-RR)/4
wz = 1000*(-FL+FR-RL+RR)/(4*L)
```

`rtl/motor/xtark_r20_mecanum_params.sv` 保存源固件参数：轮距 196 mm、轴距 160 mm、轮径 98 mm、每轮每圈 122880 计数、50 Hz 采样，因此 `L=178 mm`。编码器比例在 RTL 中取 `MM_PER_TICK_Q16=164`，即约 0.002502 mm/计数；与源固件浮点比例约 0.002505 mm/计数相差约 0.12%。测速模块使用累计计数之差，等价于源固件每窗读取后清零的计算方式，但负数右移有不到 1 mm/s 的取整差异。

`sim/motor/tb_xtark_r20_mecanum_port.sv` 使用这些参数验证混合速度命令、编码器测速、直行与原地旋转的正运动学。运行 `Scripts/run_motor_sim.sh` 会同时执行该仿真和现有电机仿真。

## 板级应用与边界

按当前需求，`boards/ACG720_60K_robot/src/motor/motor_closed_loop_config.sv` 已直接引用商家参数快照，并与 UDP 视频发送及 OV5640/DDR3/TFT 链路接入同一顶层；整体构建和剩余 DDR3 时序问题见[机器人融合工程](../FPGA_ws/boards/ACG720_60K_robot/docs/build_validation.md)。商家固件的 A/C 编码器与 C/D 电机反向对应原 STM32 接线。BX72 已采用 round5 实测映射 0xD2、电机反向 0xB、编码器反向 0xA；不再使用商家默认极性。实际轮径、每圈计数与几何仍需确认；ACG720 配置独立管理。

当前板级麦轮 RTL 已有四轮目标协调限幅、目标斜坡、增量速度环与 PWM 驱动，行为不完全等同于源固件的速度限幅与 PID。该移植验证的是运动学矩阵和参数换算；不表示 ROS 2 通信、板级接线或实车闭环已经联调。


## 电机辨认入口

电机辨认已完成。旧根目录独立辨识/闭环包装、串口同步实验及配套测试已移除；复查方法使用各板 hardware_interfaces.md 中的现行命令。当前板级电机实现和通用运动学对照回归保留。

## 电机串口日志离线分析

`Scripts/motor_log_analyzer.py` 只读已有文本文件，不连接串口，也不发送指令。每个有效帧包含一位状态、四个八位十六进制计数和可选行尾，例如：

```text
0 00000004 00000000 00000000 FFFFFFFC
```

该合成示例按 32 位二进制补码解释，四路值为 4、0、0、-4。工具不从状态字段推断复位或机械运动；无效非空行会写入独立诊断 CSV。

## 运行

需要 Python 3.10 或更新版本，无第三方依赖。在 `FPGA_ws` 目录运行：

```powershell
py -3 .\Scripts\motor_log_analyzer.py .\logs\motor.log --output-prefix .\analysis\motor
```

默认 `--layout closed-loop`，输出 FL、FR、RL、RR 列。若日志记录的是尚未映射的四个原始输入通道，可使用 `--layout raw-channel`，列名为 CH0～CH3。工具不能从帧内容判断布局；布局只改变列名和图例，不翻转极性或应用标定。

每次运行生成四个文件：

- `.csv`：每帧原始十六进制值、有符号累计计数、相邻有效帧增量、时间间隔、计数率和诊断标记。
- `.svg`：四路有符号累计计数与相邻有效帧增量曲线。横轴是有效帧序号，不将主机收包时间冒充 FPGA 采样时间。
- `.summary.json`：有效/无效行数、布局、时间选项、限制，以及输入原始字节和分析脚本 SHA256。
- `.invalid.csv`：每条无法解析的非空行、行号和原因；空行只计入汇总。

若日志带时间前缀，工具支持 `HH:MM:SS[.fraction]`、Unix 秒、ISO 日期时间，以及方括号、`RX`、`接收`、`收`、`<-`、`←` 和 `◆` 标签。日期时间可带时区。只有前后两条有效帧都带同类型时间且间隔为正时才计算 counts/s；无时间、重复/倒退时间、混合时间格式或超出 `--max-gap-seconds` 的间隔会留空。日志时间一般是主机收包时间，只能作为观察依据。

HH:MM:SS 跨午夜默认不推断日期。确定日志连续跨过一天时才显式使用 `--clock-rollover`。无时间依据时不输出速度；工具不输出未经标定的 mm/s、rpm 或其他物理速度。

## 增量与诊断边界

相邻有效帧每路增量按模 `2^32` 取最短有符号差，适用于单次计数回绕，并假设两帧之间真实增量绝对值小于 `2^31`。恰好相差 `2^31` 时方向不可判定，该路增量和计数率留空并标记歧义。无效行不会重置前一有效帧；结果保留原始行号并标记间隔中有其他行。

状态变化不代表计数器复位，状态 `0` 也不代表轮子已经停止。已知复位前后应分开分析；只有相邻帧差而没有动作前后的静止基线时，不能推断整次行程。

默认模式保留可解析结果并返回 0。`--strict` 仍先生成输出文件，在存在无效行、不可用时间间隔或半量程歧义时返回 2，便于批处理检查。

## 回归测试

所有测试帧均为合成输入。回归覆盖 32 位正负边界、计数回绕、半量程歧义、无效行、时间前缀/时区/午夜、重复或倒退时间、间隔上限、原始/闭环列布局、CSV/SVG/JSON 输出与原始日志哈希。

```powershell
py -3 -m unittest discover -s .\sim\tools -p test_motor_log_analyzer.py -v
```

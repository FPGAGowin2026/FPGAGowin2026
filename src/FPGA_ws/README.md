# Gowin FPGA 工程

| 板卡 | Gowin 工程 | 顶层 | 说明 |
| --- | --- | --- | --- |
| BX72 小车 | [BX72_138K_robot.gprj](boards/BX72_138K_robot/BX72_138K_robot.gprj) | `robot_top` | [GoRV32 导航固件与 ROM](boards/BX72_138K_robot/src/firmware/README.md) |
| ACG720 六足 | [ACG720_60K_robot.gprj](boards/ACG720_60K_robot/ACG720_60K_robot.gprj) | `camera_udp_top` | [接线、遥测与控制协议](boards/ACG720_60K_robot/README.md) |

用兼容的 Gowin IDE 打开目标板的工程文件。先按部署环境设置 FPGA 和主机网卡的网络参数，再执行综合与布局布线。两块板的引脚、时序约束及 IP 配置位于各自工程目录。

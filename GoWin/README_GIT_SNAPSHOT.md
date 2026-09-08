# GoWin 本地资源目录

该目录只保存当前工程需要的本地工具、参考案例和传感器示例：

- `Gowin_V1.9.12.03`：本机 Gowin EDA，整体由父仓库忽略。
- `choiceRobot`：赛题与板级参考工程。
- `sensor_examples`：传感器及数字接口验证示例。

项目文档、专项报告和参考资料已统一迁移到 `../docs`。

正式源码使用独立仓库维护：

- FPGA RTL：`../FPGA_ws`，远端为 `FPGAGowin2026/fpga_rtl`。
- UDP 网关：`../ros_ws/src/robot/fpga_gateway`。
- ROS 2 启动与回归：`../ros_ws/src/robot/robot_bringup`。
- Tang Primer 20K 软核实验：`../gowingame`。

为避免双份源码继续分叉，本目录不再保存上述仓库的源码快照。Gowin 的 `impl/`、仿真数据库、码流、日志和临时文件仍由父仓库忽略。

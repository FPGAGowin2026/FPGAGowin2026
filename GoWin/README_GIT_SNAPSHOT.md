# GoWin 本地工具

当前板卡工程为 ACG720-60K，源码和构建入口位于 `../FPGA_ws/boards/acg720_60k_motor_closed_loop/`。

本目录只保留本机 Gowin V1.9.12.03 x64 安装目录和许可证。安装目录保持原位，构建脚本会从这里查找 `IDE/bin/gw_sh.exe`。这两项本机资源由父仓库忽略，不纳入 FPGA RTL 仓库。

旧的 138K `choiceRobot` 工程和 `sensor_examples` 教程副本已从当前工作树移除；它们不参与 60K 工程。
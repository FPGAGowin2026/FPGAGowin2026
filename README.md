# FPGAGowin2026

本仓库保存 FPGAGowin2026 项目的总体方案、接口文档以及 FPGA 和 ROS 2 工作区入口。功能源码使用独立仓库维护，不在本仓库重复提交。

## 独立仓库

- [`fpga_rtl`](https://github.com/FPGAGowin2026/fpga_rtl)：ACG720 FPGA RTL、仿真与后续 Gowin 工程。
- [`fpga_gateway`](https://github.com/FPGAGowin2026/fpga_gateway)：Jetson 与 FPGA 之间的 UDP 网关和主机侧 FPGA 模拟器。
- [`robot_bringup`](https://github.com/FPGAGowin2026/robot_bringup)：网关参数、仿真启动、硬件启动和主机闭环测试。

## 获取工作区

```bash
git clone https://github.com/FPGAGowin2026/FPGAGowin2026.git
cd FPGAGowin2026
./Scripts/sync_fpga_workspace.sh
./Scripts/sync_ros_packages.sh
```

脚本首次运行时分别克隆 FPGA 工作区和两个 ROS 2 功能包；再次运行时对已有的独立仓库执行快进更新。若目标目录不是独立 Git 仓库，脚本会停止，不覆盖其中的文件。

## FPGA 仿真

安装 Icarus Verilog 后运行：

```bash
cd FPGA_ws
iverilog -g2012 -s tb_motion_mode_arbiter \
  -o /tmp/tb_motion_mode_arbiter \
  src/motion_mode_arbiter.v \
  sim/tb_motion_mode_arbiter.sv
vvp /tmp/tb_motion_mode_arbiter
```

## ROS 2 构建与测试

环境要求为 Linux、ROS 2 Humble、`colcon` 和包清单中声明的依赖。

```bash
source /opt/ros/humble/setup.bash
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select fpga_gateway robot_bringup
colcon test-result --verbose
```

## 仓库结构

```text
FPGAGowin2026/
├── FPGA_ws/                  # FPGA RTL、仿真与集成说明
├── Scripts/                   # 工作区辅助脚本
├── docs/                      # 总体方案、交接记录和 UDP 协议
└── ros_ws/
    └── src/robot/             # 同步脚本拉取独立 ROS 2 功能包的位置
```

当前自动化验证仅覆盖模式仲裁器 HDL 仿真、ROS 主机编译、协议单元测试和本机 UDP 闭环，不代表 FPGA 综合、实板、RGMII 时序、电机或实车验证。当前状态和下一步门禁见 [`docs/handoff.md`](docs/handoff.md)。

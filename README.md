# FPGAGowin2026

本仓库保存 FPGAGowin2026 项目的总体方案、接口文档和 ROS 2 工作区入口。ROS 2 功能包使用独立仓库维护，不把包源码重复提交到本仓库。

## ROS 2 功能包

- [`fpga_gateway`](https://github.com/FPGAGowin2026/fpga_gateway)：Jetson 与 FPGA 之间的 UDP 网关和主机侧 FPGA 模拟器。
- [`robot_bringup`](https://github.com/FPGAGowin2026/robot_bringup)：网关参数、仿真启动、硬件启动和主机闭环测试。

## 获取工作区

```bash
git clone https://github.com/FPGAGowin2026/FPGAGowin2026.git
cd FPGAGowin2026
./Scripts/sync_ros_packages.sh
```

脚本首次运行时把两个功能包克隆到 `ros_ws/src/robot/`；再次运行时对已有的独立仓库执行快进更新。若目标目录不是独立 Git 仓库，脚本会停止，不覆盖其中的文件。

## 构建与测试

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
├── Scripts/                   # 工作区辅助脚本
├── docs/                      # 总体方案、交接记录和 UDP 协议
└── ros_ws/
    └── src/robot/             # 同步脚本拉取独立 ROS 2 功能包的位置
```

当前自动化验证仅覆盖 ROS 主机编译、协议单元测试和本机 UDP 闭环，不代表 FPGA 实板、RGMII 时序、电机或实车验证。当前状态和下一步门禁见 [`docs/handoff.md`](docs/handoff.md)。

# FPGAGowin2026

本仓库保存 FPGAGowin2026 项目的总体方案、接口文档以及 FPGA 和 ROS 2 工作区入口。已发布的 FPGA、网关与 bringup 源码使用独立仓库维护；尚在快速迭代的导航、视觉和任务包暂由本仓库直接维护。自研运行时节点统一使用 C++17 与 `rclcpp`，Python 只保留给 ROS 2 launch。

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

环境要求为 Ubuntu 22.04、ROS 2 Humble 和 `colcon`。导航依赖安装到工作区的 `ros_ws/src/external/ros_humble_overlay`，不需要修改系统 ROS：

```bash
./Scripts/install_ros_dependencies.sh
```

```bash
source /opt/ros/humble/setup.bash
source Scripts/setup_ros_dependencies.bash
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="robot_localization slam_toolbox nav2_bringup"
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select \
  fpga_gateway robot_bringup robot_navigation vision_bridge mission_control
colcon test-result --verbose
```

无实物时可启动完整主机仿真：

```bash
ros2 launch mission_control simulation.launch.py
```

该入口依次运行本机 UDP 网关、轮式里程计与 IMU 融合、矩形场景激光、在线 SLAM、Nav2、合成相机、红色目标识别和任务状态机。任务状态应经过 `WAITING_NAV → NAVIGATING → SEARCHING → TRACKING → COMPLETE`。

实机建图使用：

```bash
ros2 launch robot_navigation mapping.launch.py \
  imu_x:=0.0 imu_y:=0.0 imu_z:=0.0 \
  laser_x:=0.0 laser_y:=0.0 laser_z:=0.0 laser_yaw:=0.0
```

上面的零外参只适合主机仿真；实机必须替换成实际测量值，并保证真实激光发布 `/scan`、相机发布 `/camera/image_raw`。

## 仓库结构

```text
FPGAGowin2026/
├── FPGA_ws/                  # FPGA RTL、仿真与集成说明
├── Scripts/                   # 工作区辅助脚本
├── docs/                      # 总体方案、交接记录和 UDP 协议
└── ros_ws/
    └── src/
        ├── external/          # 本地 ROS 依赖覆盖层，整体忽略、不上传
        └── robot/
            ├── fpga_gateway       # 独立仓库：UDP 网关与 FPGA 模拟器
            ├── robot_bringup      # 独立仓库：通信启动与闭环测试
            ├── robot_navigation   # 本仓库 C++：融合、SLAM、Nav2 与激光仿真
            ├── vision_bridge      # 本仓库 C++：单目标观测与合成相机
            └── mission_control    # 本仓库 C++：搜索、对准、接近、停车任务
```

当前自动化与主机仿真验证覆盖模式仲裁器 HDL、ROS 构建、协议测试、本机 UDP、融合、SLAM、Nav2 和红色目标任务闭环。红色阈值识别是可替换的算法基线，不是 TensorRT/神经网络性能证明；所有主机结果也不代表 FPGA 综合、实板、RGMII 时序、电机、真实传感器或实车验证。当前状态和下一步门禁见 [`docs/handoff.md`](docs/handoff.md)。

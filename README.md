# FPGAGowin2026

本仓库保存 FPGAGowin2026 项目的总体方案、接口文档以及 FPGA 和 ROS 2 工作区入口。FPGA 和 ROS 2 工作区分别使用独立仓库维护，全部自研 ROS 2 包统一保存在 `ros_ws` 仓库。自研运行时节点统一使用 C++17 与 `rclcpp`，Python 只保留给 ROS 2 launch。

## 独立仓库

- [`fpga_rtl`](https://github.com/FPGAGowin2026/fpga_rtl)：ACG720 FPGA RTL、仿真与后续 Gowin 工程。
- [`ros_ws`](https://github.com/FPGAGowin2026/ros_ws)：统一管理网关、bringup、导航、视觉和任务控制包。

## 获取工作区

```bash
git clone https://github.com/FPGAGowin2026/FPGAGowin2026.git
cd FPGAGowin2026
./Scripts/sync_fpga_workspace.sh
./Scripts/sync_ros_packages.sh
```

脚本首次运行时分别克隆 FPGA 工作区和整个 ROS 2 工作区；再次运行时对已有的独立仓库执行快进更新。若目标目录不是独立 Git 仓库，脚本会停止，不覆盖其中的文件。

## FPGA 仿真

安装 Icarus Verilog 后运行全部 FPGA RTL 自检：

```bash
./FPGA_ws/Scripts/run_all_sim.sh
```

当前包含 10 个 RTL 功能入口和 1 个 Gowin PLL/RGMII 板级展开检查，覆盖官方 UDP 回环坏 FCS 原子拒绝、完整机器人网络 Control/Telemetry/Status、ARP/ICMP、双时钟 CDC、摄像头/IMU/三路测距、差速电机、局部绕障、独立路径、模式仲裁和系统级组合闭环。具体运行边界见 [`FPGA_ws/README.md`](FPGA_ws/README.md)。

## Gowin EDA

本机商业版 `Gowin_V1.9.12.03` 和 License 保存在被 Git 忽略的 `GoWin/` 目录。2026-08-24 已确认 `gwlicense.ini` 指向本地 License，且 `gw_sh` 完成官方 UDP 回环、机器人逻辑集成顶层和 14-I/O 离线 wrapper 的综合；离线 wrapper 还完成布局布线、STA、码流与功耗报告。真实 PLL 相位下内部网络域 Fmax 为 129.443 MHz，setup/hold 违规均为 0。该 wrapper 只使用官方 RGMII/复位管脚并把未知硬件输入固定为失效安全值，系统域时钟也是离线专用的 RGMII `/2` 分频；尚无 PHY/PCB I/O 延迟、真实系统时钟、完整板级 I/O 或实板证据。入口和证据边界见 [`FPGA_ws/README.md`](FPGA_ws/README.md#acg720-gowin-工程)。

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
└── ros_ws/                    # 独立仓库：全部自研 ROS 2 包
    └── src/
        ├── external/          # 本地 ROS 依赖覆盖层，整体忽略、不上传
        └── robot/
            ├── fpga_gateway       # UDP 网关与 FPGA 模拟器
            ├── robot_bringup      # 通信启动与闭环测试
            ├── robot_navigation   # 本仓库 C++：融合、SLAM、Nav2 与激光仿真
            ├── vision_bridge      # 本仓库 C++：单目标观测与合成相机
            └── mission_control    # 本仓库 C++：搜索、对准、接近、停车任务
```

当前自动化与主机仿真验证覆盖 FPGA 协议、传感器前端、电机控制、绕障、独立路径、模式仲裁，以及 ROS 构建、本机 UDP、融合、SLAM、Nav2 和红色目标任务闭环。红色阈值识别是可替换的算法基线，不是 TensorRT/神经网络性能证明；Gowin 离线实现现已提供内部时钟布局布线与 STA 证据，但仍不代表 PHY/PCB I/O 时序、真实电机/传感器或实车验证。当前状态和下一步门禁见 [`docs/handoff.md`](docs/handoff.md)。

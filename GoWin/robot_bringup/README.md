# robot_bringup

`robot_bringup` 集中维护 FPGA 网关的参数、主机仿真入口、硬件启动入口和 ROS 2 launch 测试。运行时依赖独立仓库 [`fpga_gateway`](https://github.com/FPGAGowin2026/fpga_gateway)。

## 获取与构建

把两个功能包放在同一个 ROS 2 工作区中：

```bash
mkdir -p ~/fpga_robot_ws/src
cd ~/fpga_robot_ws/src
git clone https://github.com/FPGAGowin2026/fpga_gateway.git
git clone https://github.com/FPGAGowin2026/robot_bringup.git
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to robot_bringup
source install/setup.bash
```

## 主机仿真

```bash
ros2 launch robot_bringup fpga_sim.launch.py
```

仿真默认使用本机回环地址和 UDP `15000/15001` 端口。需要并行运行时可覆盖端口：

```bash
ros2 launch robot_bringup fpga_sim.launch.py \
  control_port:=16000 telemetry_port:=16001
```

## FPGA 硬件入口

确认 Jetson 网卡和 FPGA 地址后运行：

```bash
ros2 launch robot_bringup fpga_hardware.launch.py \
  fpga_ip:=192.168.50.2 control_port:=5000 telemetry_port:=5001
```

共享的频率、超时、轮式里程计标定和坐标系参数位于 `config/fpga_gateway.yaml`。硬件 launch 只启动真实网关，不启动 `fake_fpga_node`。

## 测试

```bash
colcon test --packages-select robot_bringup
colcon test-result --verbose
```

launch 测试在本机 UDP 回环和独立 ROS domain 中验证遥测节拍、速度命令、命令超时、软件急停、安全恢复和模拟链路断开。

## 验证边界

仿真 launch 和自动化测试不会访问 FPGA 或电机。测试通过只证明当前 ROS 主机闭环，不能证明 FPGA 看门狗、RGMII 时序、PWM 关闭或实车安全。

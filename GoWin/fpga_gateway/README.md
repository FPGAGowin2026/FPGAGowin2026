# fpga_gateway

`fpga_gateway` 是 ROS 2 Humble 下的 Jetson—FPGA UDP 通信包，包含真实网关节点、固定二进制协议实现和用于主机闭环验证的 FPGA 模拟节点。

## 节点

- `fpga_gateway_node`：接收 ROS 速度与急停命令，周期发送 UDP 控制报文，并把 FPGA 遥测转换为 ROS 消息。
- `fake_fpga_node`：仅用于本机开发，模拟控制接收、遥测、状态、命令看门狗、丢包和 CRC 损坏。

## ROS 接口

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 订阅 | `/emergency_stop` | `std_msgs/msg/Bool` |
| 发布 | `/wheel/odom` | `nav_msgs/msg/Odometry` |
| 发布 | `/imu/data_raw` | `sensor_msgs/msg/Imu` |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 发布 | `/range/front`、`/range/left`、`/range/right` | `sensor_msgs/msg/Range` |
| 发布 | `/fpga/status` | `diagnostic_msgs/msg/DiagnosticArray` |

默认硬件端点为 FPGA `192.168.50.2:5000` 和 Jetson 遥测监听端口 `5001`。控制频率默认 100 Hz，ROS 速度命令超时默认 200 ms，链路诊断超时默认 300 ms。完整协议见主项目的 [`docs/udp_protocol.md`](https://github.com/FPGAGowin2026/FPGAGowin2026/blob/main/docs/udp_protocol.md)。

## 构建

```bash
mkdir -p ~/fpga_robot_ws/src
cd ~/fpga_robot_ws/src
git clone https://github.com/FPGAGowin2026/fpga_gateway.git
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fpga_gateway
source install/setup.bash
```

## 主机闭环运行

分别在两个已加载工作区环境的终端中运行：

```bash
ros2 run fpga_gateway fake_fpga_node
```

```bash
ros2 run fpga_gateway fpga_gateway_node --ros-args \
  -p fpga_ip:=127.0.0.1 -p bind_ip:=127.0.0.1 \
  -p control_port:=15000 -p telemetry_port:=15001
```

实际部署推荐使用 `robot_bringup` 中统一维护的参数和 launch 文件。

## 测试

```bash
colcon test --packages-select fpga_gateway
colcon test-result --verbose
```

协议 GTest 覆盖黄金报文、三类报文往返、异常报文拒绝和序号回绕。

## 安全与验证边界

网关在 ROS 命令超时后发送零速度，软件急停解除后也要求新的 `/cmd_vel` 才恢复运动。FPGA 侧仍必须独立实现 PHY、控制心跳和 PWM 关闭等最终安全措施；主机模拟器通过不能作为 FPGA 实板或实车安全证明。

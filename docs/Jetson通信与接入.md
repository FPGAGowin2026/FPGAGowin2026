# Jetson 通信与 ROS 接入

当前 ROS 源码位于 `ugv_ws/src/fpga_gateway`，启动包为 `ugv_ws/src/car/car_bringup`。本文适用于 BX72 UDP 版及独立 ACG720；BX72 蓝牙版使用 [网页控制器与串口协议](../FPGA_ws/boards/BX72_138K_robot/docs/bluetooth_ekf_telemetry.md)。

## ROS 工作区与遥控

Jetson/Linux、ROS 2 Humble、C++17；网关依赖 OpenCV 和 nlohmann_json。先按 ugv_ws 的依赖说明准备环境：

```bash
cd ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup
source install/setup.bash
ros2 launch robot_bringup mission.launch.py mode:=teleop bind_ip:=192.168.0.4 fpga_ip:=192.168.0.2
```

网关 `jetson_teleop_bridge` 接收 PC 控制，转发三轴速度、ACK 和视频；它不订阅 `/cmd_vel`。Windows PC 在 FPGA_ws 根目录运行 `python Scripts/pc_teleop.py --jetson <Jetson无线IP>`。操作、网络邻居配置和停止规则见 [遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)。

## 数据契约

| 通道 | 端口与格式 |
| --- | --- |
| 控制/诊断下行 | FPGA `192.168.0.2:5000`，32 字节 BX72 v1 |
| 入队回执 | Jetson `192.168.0.4:6103`；诊断 type 1/2、手动速度 3/4、自动模式 5/6 |
| 视频/状态 | Jetson 6102；800×480 RGB565 RTP 和 ACG6 状态 |
| 传感器 | Jetson 6104；BX72 v1 五类遥测，或独立 ACG720 BXT1 |
| PC 网关 | Jetson 6200；控制 JSON、状态及 BXJV/JPEG |

控制 vx/vy 单位 mm/s、wz 单位 mrad/s；新会话必须零速解锁，校验 token、递增序号和 CRC16。ACK 只表示入队。自动模式默认关闭，切换先停车、200 ms 续租超时退出；底层电机另有 300 ms 看门狗。详细字节布局见 [单通信协议](../FPGA_ws/boards/BX72_138K_robot/docs/single_comm_20261017.md)。

网关独占 6102/6103/6104；记录器独占 6102/6104；传感器节点占 6104。不要同时启动争用端口的程序。ROS Header 为接收时间，timestamp_us 为 FPGA 提交时间；尚未完成统一时间映射，接收节点不发布 odom/TF。

## BX72 双向 UDP 联调

FPGA 没有 ARP/ping 回复；Jetson 有线接口配置 `192.168.0.4/24`，按本机网口名设置 FPGA 永久邻居：

```bash
sudo ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 nud permanent dev enP8p1s0
ss -lunp | grep -E ':6102|:6103|:6104|:6200'
```

接口名和 FPGA MAC 以当前顶层配置为准。排查 RX 相位训练、坏包和无回执见 [接收诊断](../FPGA_ws/boards/BX72_138K_robot/docs/bidirectional_udp.md)。自动模式命令会让机器人尝试避障，使用方式见 [单通信版本](../FPGA_ws/boards/BX72_138K_robot/docs/single_comm_20261017.md#udp-自动模式)。

## BX72 传感器接入

加载工作区环境后，选择只读记录器或 ROS 节点之一：

```bash
ros2 launch robot_bringup mission.launch.py mode:=recording bind_ip:=192.168.0.4 output:=/tmp/bx72-data duration:=30
# 或使用 ROS 传感器节点；不要与上面的记录器同时运行
ros2 launch robot_bringup mission.launch.py mode:=sensors bind_ip:=192.168.0.4
```

ROS 入口为 `bx72_round5_node`。线格式、序号和丢包规则见 [五类遥测](../FPGA_ws/boards/BX72_138K_robot/docs/jetson_udp_telemetry.md)，视频组帧见 [视频说明](../FPGA_ws/boards/BX72_138K_robot/docs/video_jetson.md)。

## ACG720 传感器接入

ACG720 使用 BXT1，不接收 BX72 运动协议；`bx72_sensor_node` 的历史命名不改变其 BXT1 协议用途：

```bash
ros2 launch robot_bringup mission.launch.py mode:=sensors board:=acg720 bind_ip:=192.168.0.4
```

本板资源和 DDR 时序尚未通过，不能继承 BX72 验收。协议见 [ACG720 遥测](../FPGA_ws/boards/ACG720_60K_robot/docs/sensor_telemetry.md)。部分 FPGA 验证脚本仍引用旧 ros_ws 路径；ugv_ws 接入、C++ 构建与实板发布订阅的验证边界见 [项目状态](handoff.md)。

# Jetson 通信与 ROS 接入

当前 ROS 源码位于 `ugv_ws/src/fpga_gateway`，唯一启动文件为 `ugv_ws/src/car/car_bringup/launch/mission.launch.py`，包名与目录名均为 `car_bringup`。当前工作区仅保留 BX72 UDP 接入；BX72 蓝牙版使用 [网页控制器与串口协议](../FPGA_ws/boards/BX72_138K_robot/docs/bluetooth_ekf_telemetry.md)。

`codex/bx72-ddr-mapping@911a158` 先提供蓝牙 DDR3 雷达 SLAM，本轮融合继续将同款建图扩展到 UDP FPGA 工程，与相机视频共享 DDR3。两版尚未定义地图上行协议，也未接入本 UDP 网关、ROS `OccupancyGrid` 或 TF 发布；FPGA 内部支持建图不能证明 Jetson 已能读取地图或执行基于该图的导航。

## ROS 工作区与遥控

Jetson/Linux、ROS 2 Humble、C++17；网关依赖 OpenCV 和 nlohmann_json。先按 ugv_ws 的依赖说明准备环境：

```bash
cd ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to car_bringup
source install/setup.bash
ros2 launch car_bringup mission.launch.py
```

入口固定启动 `jetson_teleop_bridge`，默认监听 `0.0.0.0` 并连接 FPGA `192.168.0.2`，不再提供 launch 模式。未接网线也能启动，实板收发仍需配置下文的有线地址与静态邻居。网关等待网页经 PC 服务下发控制和自动模式请求，转发三轴速度、ACK、五类遥测和视频；它不订阅 `/cmd_vel`。Windows PC 按 [网络网页手册](../FPGA_ws/boards/BX72_138K_robot/docs/network_dashboard.md) 启动本机服务并连接 Jetson 无线 IP。Python/Tk 客户端仍可独立使用，见 [遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)。

## 数据契约

| 通道 | 端口与格式 |
| --- | --- |
| 控制/诊断下行 | FPGA `192.168.0.2:5000`，32 字节 BX72 v1 |
| 入队回执 | Jetson `192.168.0.4:6103`；诊断 type 1/2、手动速度 3/4、自动模式 5/6 |
| 视频/状态 | Jetson 6102；800×480 RGB565 RTP 和 ACG6 状态 |
| 传感器 | Jetson 6104；BX72 v1 五类遥测 |
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

独立调试时先停止统一入口，再选择只读记录器或 ROS 节点之一：

```bash
ros2 run fpga_gateway jetson_telemetry_receiver --bind 192.168.0.4 --source 192.168.0.2 --output /tmp/bx72-data --duration 30
# 或使用 ROS 传感器节点；不要与上面的记录器同时运行
ros2 run fpga_gateway bx72_round5_node --ros-args -p bind_ip:=192.168.0.4 -p fpga_ip:=192.168.0.2
```

ROS 入口为 `bx72_round5_node`。线格式、序号和丢包规则见 [五类遥测](../FPGA_ws/boards/BX72_138K_robot/docs/jetson_udp_telemetry.md)，视频组帧见 [视频说明](../FPGA_ws/boards/BX72_138K_robot/docs/video_jetson.md)。

导航参数与 C++ 任务源码保留在 `car_navigation`、`car_mission`，均与目录同名，不由统一入口启动。网页当前自动模式请求控制 FPGA 自动避障；Nav2 目标导航仍需补齐网页任务接口、ROS 里程计、完整 LaserScan 和速度控制适配。FPGA 的 CSM、七状态 EKF、打滑融合和自动避障独立执行。

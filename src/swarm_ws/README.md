# 协同工作区

[swarm_cooperation](src/swarm_cooperation/README.md) 负责 BX72 小车与 ACG720 六足的地图对齐、共享地图和目标调度。部署前填写[设备名册](config/robots.json)，核对各自相机的[标定参数](calibration/)。双机模式由 [Jetson 启动脚本](../ugv_ws/Scripts/README.md) 启动。

[swarm_interfaces](src/swarm_interfaces/README.md)、[swarm_manager](src/swarm_manager/README.md) 和 [swarm_bringup](src/swarm_bringup/README.md) 提供历史目标、中继消息与启动入口，供[无人机工作区](../uav_ws/README.md)等接口实验使用。

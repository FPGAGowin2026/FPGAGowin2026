# 无人机启动入口

`launch/` 包含飞控、VIO、EGO-Swarm 和集群中继的组合入口；`sim/uav_bringup/launch/` 保存 MAVROS SITL 启动文件。`dds.launch.py` 默认不启动自动飞行任务，DDS 后端的默认配置为只读。

协同中继的依赖与启动参数见[中继任务说明](../../docs/swarm_relay.md)。飞控、定位、传感器与坐标系均需在现场核对；自动任务入口可能请求解锁和起飞。

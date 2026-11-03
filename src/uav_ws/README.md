# 无人机工作区

包含 PX4/MAVROS 与 DDS 控制接口、VIO、无人机任务和 EGO-Swarm 传感器桥。协同中继需要同级 `../swarm_ws`，并需单独准备 PX4、定位、深度相机和 EGO-Swarm。代码及依赖准备见 [中继任务说明](docs/swarm_relay.md) 与 [EGO-Swarm 来源](third_party/README.md)。

本仓库中的无人机部分为扩展实现；现有地面小车的航点实测不代表无人机仿真或实飞已经通过。

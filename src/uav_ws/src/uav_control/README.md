# 无人机控制接口

`uav_control` 提供统一的目标、状态和命令接口，后端可选择 MAVROS 或 PX4 DDS。后端配置位于 `config/backend.yaml`、`config/mavros.yaml` 与 `config/dds.yaml`；DDS 默认只读。命令被接受与飞控实际完成状态变化分别报告。

`uav_interfaces` 位于本包子目录，colcon 构建时须显式加入 `src/uav_control/uav_interfaces`。任务节点与控制节点使用同一份消息定义；中继构建见[中继任务说明](../../docs/swarm_relay.md)。实际通信、机载定位融合和飞行仍需单独验证。

# ACG720 板载与建图软件

现役协同成员为 BX72 小车与 ACG720，ACG720 使用独立的 `/acg720` 话题。

| 包 | 内容 |
| --- | --- |
| `legged_bridge` | ACG720 UDP/HTTP/ROS 接入、传感器解析与相机标定元数据 |
| `legged_bringup` | ACG720 雷达里程计、建图和相机标定启动 |

ACG720 RTL 位于[板级工程](../../../FPGA_ws/boards/ACG720_60K_robot/README.md)。`swarm_ws` 提供成员对齐、共享地图与任务调度，ACG720 单机接入不依赖该工作区。

在 `ugv_ws` 中加载 ROS 2 后，增量构建 `colcon build --packages-select legged_bridge legged_bringup --executor sequential`，再加载 `install/setup.bash`。系统依赖通过已安装的 ROS 2 包查找。

# 无人机集群中继任务

本工作区在已有 PX4/MAVROS、VIO 和无人机飞行状态机上接入 EGO-Swarm 的三维局部规划。集群侧通过 `swarm_ws` 指定中继目标，无人机侧自行规划并经已有 `uav_control` 发布 MAVROS 位置设定点。原单机 `mission_node` 流程仍保留；协同入口使用同一状态机的 `relay_mission_node`，不能与另一套向 `/uav_control/target` 写入的任务同时运行。

数据链路为：`/swarm/uav/relay_target` → `RelayGoal` → 无人机本地坐标变换 → EGO 规划和深度避障 → `PositionCommand` 的位置与航向 → `/uav_control/target` → MAVROS。无人机按实际位置报告任务状态，`/swarm/uav/relay_status` 可区分接受、规划、执行、到达和失败。集群现有的视觉及雷达候选不会自动成为无人机目标。

## 准备与构建

需要 ROS 2 Humble、`mavros_msgs`、MAVROS、深度相机驱动及 VIO，并安装上游规划器所需的 Eigen、PCL、OpenCV、`cv_bridge`、`message_filters` 和 `pcl_conversions`。从本工作区获取固定版本 EGO-Swarm 和项目补丁；来源、许可和补丁说明见 [third_party](../third_party/README.md)。示例假定 `uav_ws` 和 `swarm_ws` 同属一个顶层目录：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/uav_ws
bash Scripts/fetch_ego_swarm.sh
colcon build --base-paths src/uav_control/uav_interfaces src ../swarm_ws/src \
  third_party/ego-planner-swarm/src/planner/traj_utils \
  third_party/ego-planner-swarm/src/planner/plan_env \
  third_party/ego-planner-swarm/src/planner/path_searching \
  third_party/ego-planner-swarm/src/planner/bspline_opt \
  third_party/ego-planner-swarm/src/planner/plan_manage \
  third_party/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs \
  --packages-up-to swarm_bringup uav_bringup
source install/setup.bash
```

分别部署时，集群端和无人机端需使用同一版本的 `swarm_interfaces`；只在集群端运行时可单独构建 `swarm_ws`。机器之间需要可用的 ROS 2 DDS 通信和同步时钟。

## 运行依赖

- PX4、MAVROS、VIO 的上游节点和深度相机驱动单独启动。默认使用 `/mavros/local_position/odom`，VIO 桥将现有位姿送到 MAVROS。
- 提供相机驱动发布的深度话题、光学帧和实际内参 `fx/fy/cx/cy`，以及 16 位深度图每米计数 `depth_scaling_factor`。32 位浮点深度图按米解释；深度标定和话题设置见 [EGO 输入适配](../src/uav_ego_bridge/README.md)。
- 提供实际标定的 `base_link` 到相机光学帧 TF。目标坐标系若不同于无人机本机 `map`，还需提供两者的真实 TF 及适用于目标时标的变换；不能仅修改 frame 名称来对齐坐标。
- 飞机端启动 `ros2 launch uav_bringup swarm_relay.launch.py`，传入上述 `depth_topic`、`camera_frame`、`fx`、`fy`、`cx`、`cy`、`depth_scaling_factor`。该入口启动现有控制器和协同任务，任务会按配置尝试解锁和起飞；参数 `robot_id` 默认为 `uav_01`。若 VIO 桥已在别处运行，设置 `start_vio_bridge:=false`。
- 集群端运行 `ros2 launch swarm_bringup relay.launch.py robot_id:=uav_01`，再向 `/swarm/uav/relay_target` 发布带真实 frame 和当前 ROS 时间戳的三维 `geometry_msgs/PoseStamped`；监听 `/swarm/uav/relay_status` 查看结果。目标只发布一次，应先启动无人机端并完成 DDS 发现。有效期默认 60 秒，仅约束接收和开始规划。

当前规划器复用 EGO-Swarm 的局部占据地图和 B 样条；现有控制接口只执行轨迹的位置与航向，未向 PX4 传入规划速度和加速度。`drone_id` 用于 EGO 机间轨迹标识，`robot_id` 用于集群任务寻址；多机需要独立话题重映射、坐标与时钟对齐，并尚待实际联调。这里的无人机轨迹共享不表示已经实现与地面机器人之间的碰撞预测。

上游规划参数中的地图范围、障碍膨胀和高度限制沿用演示值，需要按飞机和场地配置。本入口只支持系统时间，`use_sim_time=true` 会被拒绝。代码构建或消息测试不代表真实飞行、相机标定、定位一致性或无线网络已验证。

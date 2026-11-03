# EGO 输入适配

本包只适配传感器，不解锁飞机、不发送 MAVROS setpoint。启动 `uav_bringup/ego_swarm.launch.py` 会启动本桥、上游规划器和轨迹服务器；不会启动官方模拟器、飞控或无人机任务。

输入为 `nav_msgs/Odometry`（默认 `/mavros/local_position/odom`）和显式指定的 `sensor_msgs/Image` 深度图，均使用 SensorDataQoS 订阅。深度图必须使用标定相机光学坐标系，配置 `camera_frame` 后与消息 frame 精确一致；`16UC1` 数值按给定每米计数比例解释，`32FC1` 以米为单位。相机内参 `fx/fy/cx/cy` 与 `depth_scaling_factor` 由调用方提供，不能使用上游模拟相机参数冒充实测值。

桥缓存真实里程计，在图像采集时标上用前后机体位姿插值，不做位姿外推；再查该时标下 `body_frame ← camera_frame` TF 计算相机位姿。若里程计世界系不同于 `planning_frame`，还需提供 `planning_frame ← odom.header.frame_id` TF。桥不发布全局 TF，MAVROS 不需要开启 TF 广播。缺少外参、位姿夹逼、时标不新鲜或 TF 不可用时，不转发该图像，`/uav_ego/sensor_ready` 超时或失败后为 false。

输出均为 reliable、volatile：

| 话题 | 类型与含义 |
| --- | --- |
| `/uav_ego/odom` | `nav_msgs/Odometry`；位置、朝向及线/角速度转换到规划世界系，保留采集时标。此专用输出的 twist 刻意采用 EGO 的世界系语义，与标准 Odometry 的 child frame 速度约定不同，不应作为通用里程计转发。 |
| `/uav_ego/depth` | `sensor_msgs/Image`；原始深度数据与采集时标。 |
| `/uav_ego/camera_pose` | `geometry_msgs/PoseStamped`；相机光学系在规划系中的姿态，时标与深度图完全一致。 |
| `/uav_ego/sensor_ready` | `std_msgs/Bool`；10 Hz，里程计、成功配对深度和外参均新鲜才为 true；任务侧只需检查此状态及其接收新鲜度。 |

默认最大输入源年龄和接收年龄均为 0.5 秒，图像前后位姿间隔最大 0.2 秒；可在节点参数中调整。深度超时后，规划器等到新的配对观测完成融合才允许恢复；任务侧会为恢复后的目标重新规划。本桥不验证物理标定准确性、坐标对齐、机体遮挡或相机视场覆盖。

规划输入 `/uav_ego/goal` 使用规划坐标系的 `PoseStamped`，其非零源时间戳是目标代次标识。每条实际生成并发布的 `/uav_ego/planning/bspline` 会伴随 `/uav_ego/trajectory_goal`（`std_msgs/UInt64MultiArray`，`[goal_stamp_ns, trajectory_id]`）；不同话题之间不保证到达顺序。重复相同目标代次和坐标不会重新规划，已生成轨迹时会重发最近一条关联；同一代次传入不同坐标会被忽略。`/uav_ego/position_cmd` 是上游 `quadrotor_msgs/PositionCommand`，任务侧负责目标授权和控制源仲裁。轨迹 ID 在规划进程重启后会重置，不是跨进程唯一标识。

## 获取、构建与启动

```bash
cd uav_ws
bash Scripts/fetch_ego_swarm.sh
source /opt/ros/humble/setup.bash
colcon build --base-paths src/uav_control/uav_interfaces src \
  third_party/ego-planner-swarm/src/planner/traj_utils \
  third_party/ego-planner-swarm/src/planner/plan_env \
  third_party/ego-planner-swarm/src/planner/path_searching \
  third_party/ego-planner-swarm/src/planner/bspline_opt \
  third_party/ego-planner-swarm/src/planner/plan_manage \
  third_party/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs \
  --packages-up-to uav_ego_bridge ego_planner uav_bringup
source install/setup.bash
ros2 launch uav_bringup ego_swarm.launch.py \
  depth_topic:="$EGO_DEPTH_TOPIC" camera_frame:="$EGO_CAMERA_FRAME" \
  fx:="$EGO_FX" fy:="$EGO_FY" cx:="$EGO_CX" cy:="$EGO_CY" \
  depth_scaling_factor:="$EGO_DEPTH_COUNTS_PER_METER"
```

需要 ROS 2 Humble、MAVROS、Eigen、PCL、OpenCV、`cv_bridge`、`message_filters` 和 `pcl_conversions`。参数 `planning_frame` 默认 `map`，`body_frame` 默认 `base_link`；`odom_topic`、`drone_id`、`max_vel`、`max_acc` 和 `planning_horizon` 可覆盖。地图尺度、膨胀和高度限制目前沿用上游演示参数，部署前须按场地与飞机调整；默认值不代表实机安全范围。`use_sim_time=true` 明确拒绝，因为上游轨迹与规划时钟尚未统一。多机编号、共享地图和跨机轨迹/时钟链路也尚未在本启动入口验证。

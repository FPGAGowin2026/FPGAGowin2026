# 轮式搜索结果交接候选

`handoff_candidate` 订阅小车现有的 `/mission/state`（`std_msgs/String`）和 `/mission/selected_target`（`vision_msgs/Detection2DArray`）。只有状态为 `FOUND`、检测帧新鲜且含一个有效目标时，才在 `/swarm/target_candidate` 继续发布原始二维检测；该输出保留相机坐标系、源时标和会话内跟踪 ID。

目标为空、内容无效、状态离开 `FOUND`，或源时间/接收时间超时后，节点发布空检测数组。默认超时为 0.25 秒，可通过 `observation_timeout_s` 按传感器频率调整；用单调时钟检查接收超时，并拒绝重复、乱序和超前超过 50 ms 的帧。ROS 时钟仍需同步。目标类别可缺省，不影响有效检测框的传递。`/swarm/handoff_status` 使用 `diagnostic_msgs/DiagnosticArray` 报告状态原因。

`FOUND` 只代表轮式节点请求了零速。候选不是目标的三维坐标、四足任务分配、轮式停稳证明或到达证明；节点不会发送运动命令。目标身份仅在轮式跟踪会话内有效。四足与目标的位置关系、停稳反馈及新增断链停车机制仍需实物验证。

在已有 ROS 2 Humble 的机器上：

```bash
source /opt/ros/humble/setup.bash
cd /home/aa/FPGAGowin2026/swarm_ws
colcon build --packages-up-to swarm_manager --symlink-install
source install/local_setup.bash
ros2 run swarm_manager handoff_candidate
```

轮式的 `car_mission` 需在同一 ROS 域运行 `search_only` 任务。另开终端观察 `/swarm/target_candidate` 和 `/swarm/handoff_status`。该节点不自动启动搜索。

## 雷达位置候选

可另外运行 `ros2 run swarm_manager target_projection`：它订阅候选检测、轮式 `/scan` 和 `/camera/camera_info`，并持续发布 `/swarm/target_position_candidate`（`swarm_interfaces/TargetPositionCandidate`）。没有外参文件时只输出 `valid=false`、`reason=calibration_missing`。要启用投影，须提供含 `camera_frame`、`lidar_frame` 和 `camera_from_lidar` 的 JSON 文件，并以 `--ros-args -p extrinsic_file:=/绝对路径/标定.json` 指定；矩阵为 **3×4 行优先，轮式雷达米坐标到相机光学坐标**，旋转必须是正交且行列式为 1 的真实刚体旋转。相机内参与 5 项 `plumb_bob` 畸变从实测 `CameraInfo` 获得；帧名必须与输入完全一致。轮式当前没有实测外参，不提供示例数值。

算法将雷达平面返波投影到原始图像检测框中，只在单一相邻返波簇、至少 3 点且距离跨度不超过 0.3 m 时输出代表性点。相机观测与雷达帧默认各须在 0.25 秒内，时标差默认不超过 0.15 秒；可分别用 `observation_timeout_s`、`sync_tolerance_s` 调整。丢失、超时、帧不匹配、多簇或配置无效时不输出有效位置。轮式现有雷达无逐束时间，时标差检查并不能消除扫描运动畸变。输出中的有效点只表示“投影落入框内的雷达返波”：背景、遮挡或目标没有与雷达平面相交仍可能造成误关联，因此不能当成已确认的目标位置或驱动四足的依据。

`ros2 run swarm_manager target_to_legged` 订阅轮式位置候选，按其扫描时标通过 TF 转换到 `legged_laser`，并输出 `/swarm/legged_target_candidate`。上游已判断检测与雷达匹配及返波质量，转换节点只检查候选有效性、扫描时标新鲜度及变换；它的 `observation_timeout_s` 与上游使用相同默认值。必须事先由实测或可信定位结果提供连接轮式雷达与四足雷达的完整 TF 链；没有变换、观测过期、上游失效或变换不合法时，该输出为 `valid=false`。默认不创建两机器人之间的变换，不发送运动命令；若任一地图重置，旧的坐标对齐须重新建立。

回归测试：`ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 colcon test --packages-select swarm_manager`，再运行 `colcon test-result --verbose`。测试覆盖从 FOUND 到四足坐标候选、LOST 清除、标定与时间检查；所用检测、雷达和变换均为合成数据，不是实机联调。

## 无人机中继目标入口

`relay_goal_gateway` 将明确提供的 `/swarm/uav/relay_target`（`geometry_msgs/PoseStamped`）转发为 `/swarm/uav/relay_goal`（`swarm_interfaces/RelayGoal`），并订阅 `/swarm/uav/relay_status`，在状态变化时输出任务状态。它不订阅上述视觉/雷达候选，不自行选择中继位置、启动飞机或控制电机。运行 `ros2 run swarm_manager relay_goal_gateway`，或独立启动 `ros2 launch swarm_bringup relay.launch.py`；参数 `robot_id` 默认为 `uav_01`，`valid_for_s` 默认为 60 秒，可按需配置。

输入需携带目标的原始 ROS 时标和实际坐标系，以及有限三维位置和非零四元数；入口会将四元数归一化，过期、超前超过 50 ms 或无效位姿会被忽略。入口保留原始 header，只可靠发布一次，不缓存或重放运动目标；无人机端须在线并先完成发现。有效期从目标生成时刻起算，限制接收和启动，已启动任务不会因超过该时限而强制结束；到达与否以执行状态为准。跨设备时钟和目标坐标到飞控本机 `map` 的变换需另行建立；仅发布目标不表示接受、执行或到达。入口进程重启后编号会从 1 重新开始，不能仅以编号区分跨重启的任务。

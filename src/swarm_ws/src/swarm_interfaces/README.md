# 协同消息

`TargetPositionCandidate` 表示轮式 LiDAR 返波在相机检测框内的**候选**位置。`valid=true` 时，`header.stamp` 是轮式雷达扫描时标，`header.frame_id` 是 `position` 所在坐标系；`detection_stamp` 是相机检测时标，`target_id` 是轮式跟踪会话内 ID。`position` 是代表性雷达返波，不是目标中心；`supporting_points` 和 `range_spread_m` 描述支持该候选的返波。`valid=false` 时必须忽略位置，并检查 `reason`。

消息本身不建立相机外参或两机器人之间的变换，也不表示任务下发或到达确认；不得直接用于驱动机器人。坐标系以各条消息的 `header.frame_id` 为准。

`RelayGoal` 是明确指定给无人机的三维中继目标：`header.frame_id` 和 `header.stamp` 保留目标的参考系和生成时刻，`valid_for` 自该时刻起限制接收和启动，`task_id` 在目标入口进程内非零递增，`robot_id` 标识执行端。已启动任务不因有效期届满而强制结束，其结果由状态报告。

`RelayStatus` 以相同的 `task_id` 和 `robot_id` 报告 IDLE、ACCEPTED、PLANNING、EXECUTING、ARRIVED、FAILED 或 CANCELED；`pose_valid` 为真时 `current_pose` 才有意义。接受不等于到达。该消息不自动建立跨设备时钟或坐标变换；`task_id` 不是跨入口进程重启的全局唯一 ID。

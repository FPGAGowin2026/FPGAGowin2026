# ACG720 雷达建图

常用脚本入口与小车的 `bx72_run.sh` 放在同一目录。在树莓派执行：

```bash
cd ugv_ws  # 从项目根目录执行
bash Scripts/acg720_run.sh
```

在 Jetson 启动小车、六足建图、协同台和网页。

```bash
cd ugv_ws  # 从项目根目录执行
bash Scripts/bx72_run.sh
```

只需要六足本地建图时，在 Jetson 运行 `bash Scripts/bx72_run.sh mapping`。脚本复用现有桥接、建图和协同进程，不重复启动；建图使用 ROS 环境中的 RTAB-Map，并加载 `swarm_ws/calibration` 中保存的内参和安装参数。自行启动的前台进程按 `Ctrl+C` 退出；提示已有进程时，仅复用，不接管它们的生命周期。桥接与建图启动均不自动发送行走指令。


树莓派上继续运行 `acg720_bridge` 接收 FPGA；在具有 `rtabmap_odom`、`slam_toolbox` 的计算机上启动：

```bash
ros2 launch legged_bringup acg720_mapping.launch.py
```

此入口复用 RTAB-Map 的 `icp_odometry` 从 `/acg720/scan` 估计二维雷达里程计，再交给 `slam_toolbox` 建图。输出 `/acg720/odom`、`/acg720/map`，TF 为 `acg720_map -> acg720_odom -> acg720_lidar`。它不启动树莓派桥接器、运动节点或自动任务，适用于手控移动建图。雷达扫描匹配在缺少结构的场景中可能退化；实际精度仍须实地验证。

这里的地图参考点是雷达，尚未假定雷达与机身或 IMU 的安装位置。未标定的 IMU 不参与融合；目前 UDP 快照没有逐束时间戳，因此不启用扫描去畸变。配置位于 `config/acg720_mapping.yaml`，可用 `mapping_config_file:=<路径>` 覆盖。

每次启动重新建图，SLAM 启动参数最后将 `map_file_name` 强制设为空字符串，即使自定义配置指定旧图也不会恢复；历史地图归档仍保留。

AprilTag 协同对齐还需要实际相机内参和相机到雷达的安装变换。文件准备好后使用：

```bash
ros2 launch legged_bringup acg720_mapping.launch.py \
  camera_info_file:=/absolute/path/camera_info_candidate.yaml \
  camera_mount_file:=/absolute/path/camera_mount.yaml
```

`camera_info_file` 是匹配 FPGA **800×480** 图像的标准 ROS CameraInfo YAML；`camera_mount_file` 描述 `acg720_lidar -> acg720_camera_optical`，字段为 `parent_frame`、`child_frame`、`translation: {x, y, z}`（米）及 `rotation_rpy: {roll, pitch, yaw}`（弧度，固定轴 RPY）。光学坐标系使用 x 向右、y 向下、z 向前。该变换将相机坐标中的点变换到雷达坐标中，不能直接填入反方向的测量值。

独立标定模式的元数据节点沿用预览 JPEG 的帧头发布 `/acg720/camera/camera_info`。协同模式下，协调器直接从 YAML 读取内参和初步标定标记，从独立的无损 PNG 话题取得对齐图像。PNG 保留桥接器首次观察到完整帧的时间戳。

两个文件都可为空：缺内参不发布 CameraInfo，缺安装变换不发布静态相机 TF，纯雷达建图可以先行。不能把这种状态当作已完成视觉对齐。此入口也支持 `tf_topic`、`tf_static_topic`；协同任务若采用隔离 TF 话题，须给对齐端配置相同话题。新地图会话须重新执行 AprilTag 对齐。

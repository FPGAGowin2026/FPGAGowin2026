# 小车统一启动

在 Jetson 已构建的 ROS 2 Humble 工作区，通过同一个终端手动启动小车和网页：

```bash
cd ugv_ws  # 从项目根目录执行
bash Scripts/bx72_run.sh car
```

脚本加载 ROS 环境，启动 `ugv_bridge`、FPGA 命令节点、视频节点、Jetson 全局任务、slam_toolbox 和 8765 端口的网页；默认同时启动 TensorRT 视觉识别。只测试航点时可运行 `bash Scripts/bx72_run.sh car enable_vision:=false`。按 `Ctrl+C` 结束小车和网页；任务运行时，应先在网页请求停车并确认车辆和四轮反馈已停止，再按 `Ctrl+C`。

节点网络、速度、视频和模型设置位于 `config/bringup.yaml`。启动参数支持 `enable_vision:=false`、`enable_command:=false`、`enable_mission:=false` 或 `global_map_enabled:=false`。各节点启动不等于机器人已完成真实场地导航验证。

每次启动重新建图，启动入口将 `map_file_name` 强制设为空字符串，即使自定义配置指定旧图也不会恢复；历史地图归档仍保留。

小车启动日志会打印 SLAM 运行库的模式和目录。默认 `slam_library_dir:=auto` 优先使用 `car_navigation` 安装前缀下 `lib/car_navigation/slam` 中已有的一对 `libkartoSlamToolbox.so` 和 `libtoolbox_common.so`，缺少其中任何一个时使用系统 Humble 的 slam_toolbox。可通过 `bash Scripts/bx72_run.sh slam_library_dir:=/实际库目录` 选择同一版本的已安装私有库，或传 `slam_library_dir:=system` 选择系统库。

系统库模式将 `minimum_travel_distance` 设为 0，让静止时收到的扫描也有机会发布 `/pose`。默认的 `minimum_time_interval` 为 0.4 秒，扫描仍需有效里程计变换。系统库会把成功处理的静止扫描加入位姿图，长时间原地建图会增加图节点。旧私有库的搬运后跨位置候选搜索、歧义检查与连续确认仅在加载那对库时可用，系统库模式无法复现这套恢复行为。[Humble 扫描筛选源码](https://github.com/SteveMacenski/slam_toolbox/blob/8293d21fe5c816d0405e0cc1f3eb46c70659c267/src/slam_toolbox_common.cpp#L502-L547)

开发时可在加载 `install/setup.bash` 后单独运行 `ros2 launch car_bringup car_bringup.launch.py`。网关使用现场验证过的 Fast DDS 异步发布配置。已有建图和接收节点时，可用 `bash Scripts/bx72_run.sh mission` 补启动命令节点、任务节点和网页。

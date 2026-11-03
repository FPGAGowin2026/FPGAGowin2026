# 协同候选链路启动

```bash
source /opt/ros/humble/setup.bash
cd /home/aa/FPGAGowin2026/swarm_ws
colcon build --packages-up-to swarm_bringup --symlink-install
source install/local_setup.bash
ros2 launch swarm_bringup handoff.launch.py
```

该启动文件只启动三个候选处理节点，不启动轮式搜索、四足雷达、串口或运动。默认没有外参，位置候选无效；获得实测标定后可用 `extrinsic_file:=/绝对路径/标定.json` 指定，格式和输入依赖见 [swarm_manager 说明](../swarm_manager/README.md)。`legged_frame` 默认为 `legged_laser`。真实坐标对齐、目标身份关联和停车反馈仍需验证。

交接链路共享 `observation_timeout_s`（默认 0.25 秒）；投影的 `sync_tolerance_s` 默认为 0.15 秒。可以在启动时统一覆盖，例如 `observation_timeout_s:=0.5 sync_tolerance_s:=0.2`。

`ros2 launch swarm_bringup relay.launch.py robot_id:=uav_01 valid_for_s:=60.0` 独立启动无人机中继目标入口，不启动或改变原交接链路，也不自动起飞。接口与有效期说明见 [swarm_manager](../swarm_manager/README.md)。

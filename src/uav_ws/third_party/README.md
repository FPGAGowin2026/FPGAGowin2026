# EGO-Swarm 外部源码

无人机规划器使用浙江大学 FAST Lab 的 [EGO-Planner-Swarm ROS 2 分支](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)，固定提交 `23a8d5a191711dd65633df689bd00f55d4dea8f9`。运行 `../Scripts/fetch_ego_swarm.sh` 获取到本目录的 `ego-planner-swarm/` 并应用 `ego_swarm_humble.patch`；完整上游源码不纳入本仓库。

上游仓库根目录提供 GPL-3.0 许可文本，部分包的 `package.xml` 仍写有 `TODO: License declaration`；原版许可和来源保存在获取的上游工作树。若分发上游或修改版，应保留相应许可、补丁和源码来源，并核对各组件的许可。

补丁保留上游消息类型，修复手动目标高度、深度投影取样和图像缓冲容量，并让深度超时后的规划在新观测成功融合后恢复；在目标成功切换后，为每条实际生成的 B 样条发布目标时标与轨迹 ID 关联；轨迹服务器可配置输出坐标系、丢弃未来起点导致的无效时间输出，并在退出时释放 ROS 发布器。上游原有算法及网络消息格式仍在，补丁不代表已经完成仿真或飞行验证。

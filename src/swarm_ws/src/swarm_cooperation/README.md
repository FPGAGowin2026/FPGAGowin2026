# 小车与 ACG720 协同建图

在 `swarm_ws/config/robots.json` 中，将小车 `car` 的 `host` 及六足 `acg720` 的 `host`、`bridge_url` 从示例地址改为部署地址。

本包维护 AprilGrid 对齐、地图融合、共享地图和协同 HTTP 接口。ACG720 保留现有手控，只参加建图，不接收 Nav2 或自动探索目标。小车目标仍经 `/swarm/car/goal` 交给现役 `car_navigation`，停止仍使用 `/robot/mission/stop`。

在各自的 `ugv_ws` 目录中，Jetson 运行 `bash Scripts/bx72_run.sh`，六足树莓派运行 `bash Scripts/acg720_run.sh`。两台机器各用一个入口。[设备与启动选项](../../../ugv_ws/Scripts/README.md)

栅格融合、坐标变换、前沿选点及 AprilGrid 检测、位姿估计、内参标定使用 C++17。ROS 消息与 YAML 读写、协同调度和 HTTP 接口使用 Python。包通过 ament_cmake 构建，依赖 pybind11、Python 开发头文件和 OpenCV 开发库。AprilTag 检测需要支持 ArUco 的 OpenCV。单独开发本包时，从 `swarm_ws` 目录加载小车工作区后增量构建。

```bash
source /opt/ros/humble/setup.bash
source ../ugv_ws/install/setup.bash
colcon build --packages-select swarm_cooperation --executor sequential
source install/setup.bash
```

共同入口默认选择 `car,acg720`，启动小车建图和协同台，不启动运动任务：

```bash
ros2 launch swarm_cooperation cooperative_mapping.launch.py \
  car_camera_info_file:=<小车内参文件>
```

小车建图已经运行时设 `start_car_mapping:=false`。ACG720 雷达建图由 `legged_bringup/acg720_mapping.launch.py` 提供；需要在运行本入口的 Linux 主机上启动它时，设 `start_acg720_mapping:=true`。已有 ACG720 建图节点时保持默认 `false`，避免重复启动。本入口不远程启动树莓派桥，也不发送运动命令。

`participants` 也接受仅 `car`、仅 `acg720` 或 `acg720,car`；未选择的成员不会被启动。`anchor` 默认 `auto`，首次跨机对齐前优先选择有内参和相机 TF 的本地图成员，首次成功后锁定；也可显式指定已选择成员。同一地图会话中，参考成员临时离线不重新选择坐标系，不清除已建立的对齐。

小车和六足每次启动 SLAM 都从空图开始，标准启动不恢复历史地图或共同对齐。真实 SLAM 地图发布者变化或收到明确空图时，协同端清除该成员旧地图、合并图和共享叠加，撤销旧共同坐标下的探索目标与记录，等待当前地图重新观察 AprilTag。相同地图会话内的静止地图保留真实源时间戳，不因源时间较旧而丢弃。历史归档不删除；`swarm_console --alignment-state-file` 仅保留为显式离线恢复接口，不用于当前空图启动。

| 输入 | 小车 | ACG720 |
| --- | --- | --- |
| 地图 | `/robot/observation_map` | `/acg720/map` |
| 地图源身份 | `/map` | `/acg720/map` |
| 图像 | `/camera/image_raw` | `/acg720/camera/alignment_image/compressed`，无损 PNG |
| 内参 | `/camera/camera_info` | `/acg720/camera/camera_info` |
| 地图 / 定位参考 | `map` / `base_link` | `acg720_map` / `acg720_lidar` |
| 默认 TF 话题 | `/tf`、`/tf_static` | `/tf`、`/tf_static`，使用 `acg720_*` 独立 frame 名称 |

ACG720 建图默认使用全局 `/tf`、`/tf_static`，共同启动入口不覆盖该默认。需要独立话题时，可在 ACG720 建图入口显式传入 `tf_topic:=/acg720/tf tf_static_topic:=/acg720/tf_static` 进行 remap；协同端在接收全局 TF 的同时，也额外接收这两个独立话题。

ACG720 的定位参考是雷达，不作为机身原点。对齐使用图像 header 时间查询相机到本地图的 TF；ACG720 时间戳是主机首次观察到完整帧版本的时间，不代表曝光同步，初始对齐应保持静止。共同 AprilGrid、相机内参和实测相机安装变换仍是跨机对齐所需数据。缺标定时保留本地图，状态显示待标定，不生成零外参。

内参文件参数为 `car_camera_info_file`、`acg_camera_info_file`；ACG720 安装文件为 `acg_camera_mount_file`。未测量的值留空。安装变换的字段与单位见[标定模板](../../config/acg720_camera_mount.template.yaml)及 [ACG720 建图说明](../../../ugv_ws/src/legged/legged_bringup/README.md)。即使已有 `CameraInfo`，仍可传入同一内参文件，以保留消息无法表达的初步标定标记。

`GET /api/state` 返回按成员 ID 索引的 `robots/alignments`、锁定的 `anchor/frame`、`contributors`、`mapping_status/mapping_readiness`。各成员分别报告地图版本、在线、内参、相机 TF、对齐和本地/共同位姿。`GET /api/map?view=car|acg720` 返回本地图及版本，网页按版本更新底图并独立刷新位姿。`view=common|aligned` 融合已对齐且有地图的成员。未对齐时返回一份可用本地图，带 `local=true` 和真实 frame。

共同图内容变化时经 `/swarm/map` 发布。ACG720 地图通过 `/swarm/car/peer_map` 换算到小车 `map`，保留原始栅格、分辨率和源时间戳。两条话题均保留最近一份消息。未完成对齐或重新观察时，共享图通道发布空图以清除旧叠加。协同入口自带小车只读建图时使用 `/map`，复用带 Mission 的小车进程时使用 `/robot/observation_map`，也可通过 `car_map_topic` 指定输入。

小车探索仍需明确开始。同一地图会话完成初始对齐后，地图延迟、相机 TF 临时缺失、成员离线或共享图回执不增加协同停车门限；使用最后有效地图和变换。地图会话切换或显式重新观察清除共同对齐及旧坐标下的协同目标，重新对齐后仍需明确开始探索。ACG720 始终使用自己的手控接口。

保留参数：`car_radius=0.21` m、`sensing=0.5` m、`separation=0.8` m；半径沿用先前现场值，不表示本轮重新实测。`max_position_variance=0.01`、`max_yaw_variance=0.04` 继续保留参数兼容，不作为新增运行门限。

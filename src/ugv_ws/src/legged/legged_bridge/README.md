# ACG720 接入


`acg720_bridge` 复用唯一的 `acg720_receiver` 解析 `ACV1/ACV2`、`IMU1`、`LDR1`、`HXS1`，统一提供 UDP 收发、HTTP 和 ROS 2 入口。沿用小车的源地址绑定、来源筛选和状态反馈方式；HEX1 与 BX72 的编码和 ACK 不兼容，因此保留本板现有协议，不复制小车网关或新增一套接收器。

默认有线端点使用以下示例地址。部署时请配置主机和板端的实际地址。

| 方向 | 源 | 目标 | 负载 |
| --- | --- | --- | --- |
| FPGA → Linux | `192.0.2.10:5000` | `192.0.2.111:6102` | 视频、IMU、雷达、步态状态 |
| Linux → FPGA | `192.0.2.111:临时端口` | `192.0.2.10:6103` | 4 字节 `HEX1` + 1 字节命令码 |

FPGA MAC 为 `02:00:00:00:07:20`，默认主机 MAC 为 `02:00:00:00:00:6F`。板端顶层参数同时贯通收发地址和端口，见 [板端说明](../../../../FPGA_ws/boards/ACG720_60K_robot/README.md)。更换地址需两端一致；应用参数不配置网卡、主机路由或邻居表。FPGA 尚无 ARP 应答，后续实机接入仍需准备这些网络配置。

加载 ROS 2 Humble 后，在 `ugv_ws` 中增量构建并手动运行：

```bash
colcon build --packages-select legged_bridge --executor sequential
source install/setup.bash
ros2 run legged_bridge acg720_bridge \
  --bind 0.0.0.0 --port 6102 \
  --control-source 192.0.2.111 --control-source-port 0 \
  --control-destination 192.0.2.10 --control-port 6103 \
  --telemetry-source-port 5000 --http-bind 0.0.0.0 --http-port 8766
```

`--control-destination` 也可写为 `--fpga-address`，接收端仅接受该 IP 的包；`--telemetry-source-port 0` 可关闭源端口筛选。遥测和 HTTP 可分别绑定地址；支持标准 `--ros-args` 和话题重映射。`--http-only` 可在不加载 ROS 的情况下运行同一个 UDP/HTTP 实现。启动不发送控制命令，也不配置网络、烧录或启用系统服务。

| 接口 | 作用 |
| --- | --- |
| HTTP `:8766/api/status`、`/frame.jpg` | 状态和 JPEG，与集群代理保持兼容 |
| HTTP POST `:8766/api/control` | JSON 动作指令 |
| ROS `/acg720/command`，`std_msgs/String` | 同一 JSON 动作指令，复用 HTTP 的控制、租约和所有权逻辑 |
| ROS `/acg720/command_result`，`std_msgs/String` | 每个 ROS 请求的发送结果或拒绝原因 |
| ROS `/acg720/status` | HXS1 观测、收发计数、来源过滤、实际接收缓冲和错误 |
| ROS `/acg720/imu`、`/acg720/scan`、`/acg720/camera/image/compressed` | 原有传感器发布 |

HTTP 和 ROS 控制均接受 `action`、`pressed`、`client_id`、`lease_ms`。方向动作为前进、后退、左转、右转、左移与右移；`stop` 冻结、`release` 关闭 PWM、`stand`/`home` 回低姿态、`tripod`/`ripple` 选择模式。运动需新鲜 HXS1 状态，按住时每 100 ms 续发；租约默认 500 ms，FPGA 自身还有约 300 ms 看门狗。模式切换保留当前方向和原租约；租约到期或状态断流时停止续发。停止请求即使发送失败也会清除旧方向，不会在连接恢复后重新续发。

例如 ROS 停止请求为 `ros2 topic pub --once /acg720/command std_msgs/msg/String '{data: "{\"action\":\"stop\",\"client_id\":\"manual\"}"}'`。连续运动须持续提交请求续租，`--once` 只运行一次租约。

成功返回中的 `delivery="sent"` 仅表示主机 `sendto` 成功，`acknowledged=false`：当前 HEX1 没有逐命令 ACK。设备是否接收及当前动作由独立 HXS1 状态观察，`teleop_status_version` 是状态包版本，`rx_debug` 是四个 6 位 Gray 计数解码，不能将它们当作请求序号或执行确认。

千兆板端联调时，服务手动运行，开机启用仍关闭。现场以非阻塞方式排空 UDP 队列，仅在空队列等待；Linux 接收长度使用最大合法 1288 字节加一，仍拒绝超长包。雷达 CRC8 保留 0x4D 多项式，改为查表减少 CPU 消耗。FPGA 默认包间隔 480 µs 适配当前 Python/ROS 接收能力，PHY 保持千兆；含相机订阅的 20.05 秒观察达到 2.59 完整帧/秒、socket 丢包增量 0。网页手控入口在控制台的 `/#remote`，需明确选择 ACG720。HTTP、ROS 及网站代理的模式切换和 HXS1 回传已验证，未执行行走测试。

IMU 只做单位换算，不生成姿态解算；雷达转为固定 720 格逆时针扫描，缺测保留 NaN，扫描时间使用主机收齐时刻，未伪造逐束时戳。三个传感器保留独立坐标系；未测量机身安装外参、未接入里程计和地图，因此不发布机身 TF，也不声明自主导航就绪。协议解析来源为 `fpga_ws` 的板端配套接收代码，当前接收器在本包维护。

现场已确认机身部件遮挡雷达扫描平面。手动启动脚本 `Scripts/acg720_run.sh` 使用 `--ros-args -p lidar_self_radius_m:=0.20`，将同一角格最近的 20 cm 内回波置为 NaN；该角格不暴露更远回波，也不标为空闲。通用节点参数默认仍为 0，`range_min` 保持 0.02 m；原始 UDP 接收数据不变，ICP 与 SLAM 共用过滤后的 `/acg720/scan`。参数在节点初始化时读取，修改启动配置后须重启桥接服务。该遮罩只处理后续扫描，历史地图中的假障碍和几何错误需另行处理。

桥接器启动后，可在运行它的主机上执行 `curl http://127.0.0.1:8766/api/status`，查看遥测、传感器与控制链路状态。

## ACG720 标定与协同图像

ACG720 的现有 JPEG 显示接口保持原样。ROS 另提供 `/acg720/camera/alignment_image/compressed`：仅有订阅者时，将最新一张完整 RGB565 帧转换成无损 PNG，最多 2 Hz；不使用丢包修补后的预览，未收到新完整帧时不重复发布。此接口用于标定采集和 AprilTag 对齐。

在计算端可独立运行元数据节点，无需在树莓派上处理标定：

```bash
ros2 run legged_bridge acg720_camera_calibration --ros-args \
  -p camera_info_file:=/absolute/path/camera_info_candidate.yaml \
  -p camera_mount_file:=/absolute/path/camera_mount.yaml
```

`camera_info_file` 使用与 800×480 图像对应的标准 ROS CameraInfo YAML。独立标定启用该参数时，节点从预览 JPEG 取得帧头并发布 `/acg720/camera/camera_info`。`camera_mount_file` 使用实测 `acg720_lidar -> acg720_camera_optical` 变换（`translation` 米，`rotation_rpy` 弧度）。建图端的 `acg720_mapping.launch.py` 接受同名参数。协同模式由协调器直接读内参文件，标定节点负责发布安装变换。


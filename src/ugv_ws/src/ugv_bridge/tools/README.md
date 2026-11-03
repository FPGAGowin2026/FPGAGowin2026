# BX72 传感器采集与标定调试工具

这些工具只订阅现有 ROS 话题，不接管 FPGA 的 UDP 端口、不发送运动命令、不修改 TF 或运行中的参数。使用 Jetson 原有工作区：

`calibration_common.py` 统一IMU常量和序号连续性统计。离线工具按本目录运行或保留同目录依赖；两个既有标定报告的字段名保持兼容。

```sh
cd /home/aa/FPGAGowin2026/ugv_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
mkdir -p build/calibration_20261028_live
```

## 联合采集

`record_joint_calibration.py` 将现有 `/camera/image_raw`、`/fpga/imu_raw`、`/fpga/n10_packets` 和电机/融合/健康/导航遥测同时保存到同一目录。它只订阅 ROS，记录原始传感器轴、包序号、FPGA源时标、ROS头时标、订阅时间，以及限速保存的原始BGR8帧；不声称硬件同步。终端显示各流接收年龄、Tag检测、丢帧和电机状态；输入 `mark 标签` 可记下人工动作，输入 `q` 结束。检测到非零目标轮速或PWM时停止录制，不发送控制命令。

```sh
python3 src/ugv_bridge/tools/record_joint_calibration.py \
  --output build/calibration_20261028_live/joint_new --duration 180 --camera-hz 10
```

输出目录必须不存在；10 Hz 原始画面约 415 MB/分钟，启动前需核对 Jetson 可用空间。图像文件按 metadata 中的 `width`、`height`、`step` 和 `bgr8` 格式解释。此工具已在 Jetson 使用本轮实车话题采集；采集有接收序号缺口，必须逐段检查，不能把一次采集成功当作全部数据连续。

尽量固定一次目标，并用连续录制覆盖水平旋转及可安全完成的非水平倾斜，采集过程中逐项提示人工动作。AprilGrid 与雷达命中面须处于已知同一平面，并且雷达扫描平面确实相交；必要时才增加共面的硬质背板。相机—IMU 与雷达—相机可共用数据，雷达—IMU可由已验证变换组合得到。目标平面姿态、惯性激励及时间链路不足时仍不能宣称完整六自由度解。

`extract_joint_camera.py` 可从原始帧提取逐帧 AprilGrid PnP；`--motion-only` 只分析记录的 IMU 运动、事件附近和定期间隔帧，原始图像不变。它记录临时内参的来源和哈希。`solve_joint_imu_rotation.py` 使用原始 IMU 与这些 PnP 姿态，分训练和时间留出集诊断旋转及有效时标偏移；它不估计平移，不把发布或接收时刻当曝光时刻，并拒绝跨 IMU 序号缺口插值。`solve_joint_lidar_camera.py` 使用显式标注的静止时间窗、原始扇区和距离区间诊断雷达点到板平面的拟合；输出始终标为未验证。各工具的命令行参数和数据格式可通过 `--help` 查看；不要把候选关联或低训练误差当作标定验收。

## AprilGrid 相机

`calibrate_camera_aprilgrid.py` 从 `/camera/image_raw` 取得 640×360 BGR8。调试终端每秒显示帧年龄、Tag ID、角点数、像素覆盖范围和几何检查，超时显示 `STALE`。每次确认新姿态后按 `s` 加回车采集一帧，按 `q` 退出：

```sh
python3 src/ugv_bridge/tools/calibrate_camera_aprilgrid.py capture \
  --dataset build/calibration_20261028_live/aprilgrid_nominal_pending --role train
```

需要独立验证时将 `--role train` 换为 `--role holdout`。每张样本保存原分辨率 PNG、头时标、哈希、识别角点及近期稳定性检查用的 PNG；几何或稳定性检查未通过的样本仍留存但不会参与拟合。`candidate` 用于保存暂不能确认稳定性的帧，不参与拟合。标板的 ID 和角点顺序由完整画面检查并写入数据集；标称边长0.025 m、间隔比例0.3。用户确认实物符合 YAML，独立尺量误差尚未提供。

多姿态训练和未参与拟合的留出样本齐备后：

```sh
python3 src/ugv_bridge/tools/calibrate_camera_aprilgrid.py fit \
  --dataset build/calibration_20261028_live/aprilgrid_nominal_pending
```

该命令给出五系数 `plumb_bob`、训练和固定内参下逐张留出的重投影误差，以及视角、距离与画面覆盖提示。应检查实际覆盖、误差、畸变形状与留出集后再决定是否将结果填入带当前相机分辨率的网桥参数；仅得到一组输出不代表通过。网桥相机头时标生成于发布时，不是已验证的曝光时间。

## IMU

静止且无需假定轴方向时：

```sh
python3 src/ugv_bridge/tools/calibrate_imu.py capture \
  --label level_unconfirmed --duration 20 \
  --output build/calibration_20261028_live/imu_level_new.jsonl
python3 src/ugv_bridge/tools/calibrate_imu.py analyze \
  build/calibration_20261028_live/imu_level_new.jsonl
```

输出文件必须是新路径。工具记录传感器原始轴、序号、状态、FPGA时标及接收时标；静止均值仅为该窗口的零偏候选。多面拟合需要人工确认各姿态的已知传感器轴比力方向，用 `--expected-accel-g` 填写，例如 `0,0,-1`；不确定方向时不要填写。只在六个正负面均齐全、稳定、数据连续且方向一致时估算每轴偏置和尺度。一个静止姿态不能确认车头、偏航或 Rx(pi) 的全部轴向，结果不会自动应用。

## N10

只记录现场数据时：

```sh
python3 src/ugv_bridge/tools/calibrate_n10.py --duration 10 \
  --output build/calibration_20261028_live/n10_passive_new.jsonl
```

可增加 `--sector left:80:100` 统计**原始雷达角度系**的扇区；可用 `--target name:distance_mm:body_bearing_deg:raw_start_deg:raw_end_deg` 附记实际尺量和人工确定的原始扇区。工具不会假定雷达角度零点等于车头，也不会自动将车体方位映射成原始扇区。无回波保留为未知；测距分布和残差只作观察，需多个已测距离、方位和参考点才能判断安装外参。

N10 原始时间是 FPGA 包快照微秒时间，消息头和订阅时间另记；`/scan` 没有逐束时间。网页地图、ROS TF 和板内导航/独立近障保护各自消费雷达数据，必须分别核对应用位置。当前没有完整可观测的相机—IMU、雷达—相机或雷达—IMU 六自由度及时间偏移结果。

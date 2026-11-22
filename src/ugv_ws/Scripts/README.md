# 小车与六足手动启动

两台机器都在各自项目下的 `ugv_ws` 目录运行。Jetson 需要已构建的 `ugv_ws`、`swarm_ws`，以及六足建图所用的 RTAB-Map 和 slam_toolbox。树莓派需要已安装的 `legged_bridge`。

| 设备 | 手动命令 | 默认启动内容 |
| --- | --- | --- |
| Jetson（BX72） | `bash Scripts/bx72_run.sh` | 小车 FPGA 桥、视频、命令、视觉、SLAM、航点，六足雷达建图，协同台，网页 |
| 六足树莓派（ACG720） | `bash Scripts/acg720_run.sh` | 六足 FPGA 接收与手控接口，ROS 传感器话题和 HTTP 8766 |

在浏览器打开 Jetson 的 8765 端口（例如 `http://192.0.2.103:8765/`），选择机器人、提交航点或开始探索。两条命令只启动服务，运动任务由操作员在网页发起。

六足只需 UDP 和网页预览时，同一入口支持 `bash Scripts/acg720_run.sh --http-only`，直接加载桥接源码。双机建图使用默认模式，由 ROS 发布雷达和相机。

## Jetson 的可选模式

| 命令 | 用途 |
| --- | --- |
| `bash Scripts/bx72_run.sh car` | 只运行小车和网页 |
| `bash Scripts/bx72_run.sh car enable_vision:=false` | 运行小车航点和网页，关闭检测与视觉任务 |
| `bash Scripts/bx72_run.sh mission` | 复用已有的小车建图、遥测和视频进程，补启动航点、命令及网页 |
| `bash Scripts/bx72_run.sh mapping` | 只在 Jetson 运行六足雷达里程计与建图 |

`all` 和 `cooperative` 都表示默认的双机模式。小车参数保存在 `src/car/car_bringup/config/bringup.yaml`。用 `camera_info_file:=<绝对路径>` 指定小车视觉测距内参。协同端的内参分别通过 `car_camera_info_file`、`acg_camera_info_file` 设置，这两个参数也会传给各自机器的相机节点。六足安装变换使用 `acg_camera_mount_file`。专用的 `mapping` 模式直接接受 `acg720_mapping.launch.py` 的参数。

Jetson 复用发现的 ROS 节点和网页，树莓派默认复用已运行的桥接或已安装的 `acg720-bridge.service`。有任务运行时先在网页停车并确认四轮反馈，再按 Ctrl+C 结束当前手动会话。已有节点及由 systemd 管理的服务继续由原入口管理。

## 开发子命令

视觉节点需要 Jetson 的 TensorRT、CUDA 和 OpenCV。协同台与 ROS Humble 的 `cv_bridge` 在同一进程，需使用兼容的 OpenCV。构建脚本优先使用本地另行安装的 `thirdparty/opencv_humble_sdk/cmake/OpenCVConfig.cmake`，否则由 CMake 查找 OpenCV。六足建图通过 ROS 环境查找 RTAB-Map；使用单独安装的运行库时，可将 `ACG720_RTABMAP_ENV` 设为其环境脚本路径。

`bash Scripts/bx72_run.sh build` 增量构建 Jetson 的小车、六足建图和协同包。`bash Scripts/acg720_run.sh build` 只构建树莓派的桥接包。`bash Scripts/bx72_run.sh test` 调用 colcon 运行小车包在本次构建中注册的测试。`car_mission/test/test_frame_processing_schedule.cpp` 仍在源码中，当前 CMake 未注册该用例。这三个子命令都需要显式选择。

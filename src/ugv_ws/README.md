# Jetson / UGV 工作区

`src/ugv_bridge` 连接 FPGA 的 UDP 控制、遥测与视频；`src/car/car_navigation` 负责全局 SLAM 适配、A* 和航点调度，并为速度型固件提供兼容局部执行器；`src/car/car_mission` 提供视觉任务。构建时需要同级的 `../FPGA_ws`，以复用 FPGA 的局部路径实现。

在准备好 ROS 2 Humble、CUDA、TensorRT、OpenCV 和配置的模型文件后：

```bash
cd src/ugv_ws
bash Scripts/bx72_build.sh
bash Scripts/bx72_run.sh
```

设备地址、传感器外参和模型路径见 [bringup.yaml](src/car/car_bringup/config/bringup.yaml)；全局地图和坐标域说明见 [global_slam.md](docs/global_slam.md)。

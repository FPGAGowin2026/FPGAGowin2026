# BX72 与 ACG720 地面机器人工作区

在已构建的 Jetson 工作区启动小车导航、六足建图、协同台和网页：

```bash
cd ugv_ws  # 从项目根目录执行
bash Scripts/bx72_run.sh
```

只运行小车时使用 `bash Scripts/bx72_run.sh car`。六足树莓派在自己的工作区执行 `bash Scripts/acg720_run.sh`。[启动模式和参数](Scripts/README.md)

小车的 C++ 节点 `robot_mission` 按顺序执行网页提交的航点，使用 SLAM 地图、实时激光和 FPGA 局部图规划路线。已知航段支持 Nav2 MPPI 跟踪。航段受阻时，节点会检查绕行和原地转向的空间，保留当前航点，通路恢复后从实际位置重新规划。[航点、避障与恢复](src/car/car_navigation/README.md)

[小车启动与建图](src/car/car_bringup/README.md) · [FPGA 网桥](src/ugv_bridge/README.md) · [ACG720 接入](src/legged/README.md) · [网页控制台](../website/README.md)

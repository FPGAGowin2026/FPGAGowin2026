# 小车导航

本包包含全局 SLAM 的扫描适配、全局地图接口、Jetson A* 与航点调度，以及速度型 FPGA 的局部兼容执行器。支持 `nav_grid_v1` 的 FPGA 自行执行局部规划；兼容执行器复用同级 FPGA 工程的 `local_path.c`，因此构建时需要保留 `FPGA_ws` 和 `ugv_ws` 的相对位置。

接口、地图保存及坐标域切换见[全局建图说明](../../../docs/global_slam.md)；导航实现细节见 [docs](docs/)。

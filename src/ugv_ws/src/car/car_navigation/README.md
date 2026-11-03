# 小车导航

本包包含全局 SLAM 的扫描适配、全局地图接口、Jetson A* 与航点调度，以及速度型 FPGA 的局部兼容执行器。支持 `nav_grid_v1` 的 FPGA 自行执行局部规划；兼容执行器复用同级 FPGA 工程的 `local_path.c`，因此构建时需要保留 `FPGA_ws` 和 `ugv_ws` 的相对位置。

全局规划、航点调度和坐标关系见 [Jetson 全局导航说明](docs/jetson_global_navigation.md)；其他导航实现细节见 [docs](docs/)。

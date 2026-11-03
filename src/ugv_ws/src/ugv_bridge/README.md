# FPGA / Jetson 网桥

网桥将 FPGA 控制、确认、视频与传感器遥测接入 ROS 2，并向网页服务提供状态和任务接口。多个观察端可同时接收数据；运动控制由明确申请的单一来源持有。状态和实际设备反馈用于区分请求接收、执行、停止与到达。

IP、端口、传感器坐标、视频格式与模型关联参数位于 [bringup.yaml](../car/car_bringup/config/bringup.yaml)。调试与标定工具在 [tools](tools/)；运行与构建方式见[工作区说明](../../README.md)。

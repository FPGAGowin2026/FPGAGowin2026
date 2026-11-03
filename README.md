# 基于 FPGA 的机器人实时感知与自主导航

这是一个以 BX72 四轮麦克纳姆轮机器人为主体的工程作品。FPGA 负责实时传感器处理、局部导航与运动控制；Jetson 负责全局建图、路线规划和任务调度；网页展示画面、地图与状态，并支持选择航点。

项目也包含无人机和集群协同接口的源码，用于探索 PX4/MAVROS、VIO、深度相机和 EGO-Swarm 的组合。**已完成的实车反馈目前仅针对地面小车航点任务；无人机和集群部分不能据此视为已完成实飞。**

## 系统组成

```mermaid
flowchart LR
    Sensors[雷达 · IMU · 编码器 · 相机] --> FPGA["FPGA / GoRV32<br/>传感器处理 · 定位 · 局部规划 · 轮速闭环"]
    FPGA <-->|UDP 控制与遥测| Jetson["Jetson / ROS 2<br/>SLAM · 全局 A* · 航点调度 · 视觉"]
    Jetson <-->|目标与状态| Web["网页控制台<br/>地图 · 视频 · 选点"]
    Jetson -. 协同接口 .-> Swarm[集群消息与无人机接口]
    Swarm -. 实验功能 .-> UAV[PX4 · VIO · 深度相机 · EGO-Swarm]
```

全局目标由 Jetson 在 SLAM 地图中规划并分段调度；支持 `nav_grid_v1` 的 FPGA 在本地栅格中规划和执行短段。网页负责选点、提交和展示，不执行全局 A*。FPGA、Jetson 与网页通过各自的边界检查和设备状态协调，停车请求与实际停车反馈分别报告。

| 部分 | 主要内容 |
| --- | --- |
| [FPGA 工程](src/FPGA_ws/) | GW5AT-138C RTL、GoRV32 固件、局部栅格与 A*、传感器、视频和电机控制 |
| [Jetson 工作区](src/ugv_ws/) | UDP/ROS 网桥、slam_toolbox 适配、全局规划、航点调度和视觉任务 |
| [网页控制台](src/website/) | 摄像头画面、地图和设备状态展示，航点选取与提交 |
| [无人机工作区](src/uav_ws/) | PX4 控制、VIO、深度输入与 EGO-Swarm 适配，属于待进一步验证的扩展 |
| [集群工作区](src/swarm_ws/) | 机器人间目标、中继与状态消息，属于待进一步验证的扩展 |

## 已验证情况

- 2026-11-04，局部导航 candidate 08 已写入 FPGA 外部 Flash，烧录工具的写入与校验通过；重载后的 UserCode 为 `0x0000C25A`。
- 用户随后进行了航点实地测试，反馈“效果可以”。本次反馈未提供航点数量、到点误差或障碍物测试条件。
- FPGA 综合与完整布局布线结果：Logic **47,292**、Register **41,762**、BSRAM **236**、DSP **49**；现有约束下最差 setup 裕量 **+0.393 ns**。
- 动态避障、侧后盲区、物理断电冷启动、板级接口时序及相机源帧完整性仍需进一步验证；相机曾出现源帧不完整，原因尚未确定。

具体范围和记录见 [验证与部署说明](docs/验证与部署.md)。源码已按本次候选整理，位流、EDA 软件、模型、构建产物和运行日志未纳入仓库。

## 开始使用

地面机器人需要 FPGA 板卡、对应传感器、Jetson、ROS 2 Humble 和网络连接；完整视觉节点还需要 CUDA、TensorRT、OpenCV 及对应的推理引擎文件。设备 IP、雷达/相机外参和模型路径在 [bringup 配置](src/ugv_ws/src/car/car_bringup/config/bringup.yaml)中按现场设置。

```bash
# 在 Jetson 上构建并启动小车节点
cd src/ugv_ws
bash Scripts/bx72_build.sh
bash Scripts/bx72_run.sh
```

```bash
# 在电脑或 Jetson 上启动网页服务，Node.js 需为 22 或更新版本
cd src/website
npm ci
node server.mjs --jetson 192.168.1.103 --port 6200
```

将示例地址改为实际 Jetson 地址，在运行网页服务的机器上访问 `http://127.0.0.1:8765`。如果 FPGA 重载或会话变化导致坐标域变化，需重新启动全局建图并重新选点；旧地图不会自动与新坐标域拼接。FPGA 构建入口和工具要求见 [FPGA 说明](src/FPGA_ws/README.md)。

## 第三方组件

[ByteTrack](src/ugv_ws/src/thirdparty/bytetrack/README.md) 使用其随附的 MIT 许可。无人机适配保留了 [EGO-Swarm 来源、许可与补丁说明](src/uav_ws/third_party/README.md)；上游规划器需单独获取。FPGA 工程含 Gowin 加密 IP，构建需使用兼容的 Gowin 工具及相应许可。

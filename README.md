# BX72 小车与 ACG720 六足协同建图

BX72 小车由 FPGA 采集雷达、IMU、编码器与相机数据，执行轮速闭环和局部避障。Jetson 运行 ROS 2，负责小车建图、全局路线和任务调度。ACG720 六足通过树莓派接入，向 Jetson 提供雷达、相机和运动状态。操作员可以在同一个网页中查看两台机器的地图、视频和反馈，选择机器人并提交航点。

地面机器人源码位于 `src/`。各模块 README 中的“项目根目录”指 `src/`。下方命令从仓库根目录运行。仓库另保留了 [无人机工作区](src/uav_ws/README.md)及其[消息接口](src/swarm_ws/src/swarm_interfaces/README.md)。

## 源码入口

| 模块 | 工程或文档 |
| --- | --- |
| BX72 小车 FPGA 与 GoRV32 固件 | [Gowin 工程](src/FPGA_ws/boards/BX72_138K_robot/BX72_138K_robot.gprj) · [固件构建](src/FPGA_ws/boards/BX72_138K_robot/src/firmware/README.md) |
| ACG720 六足 FPGA | [Gowin 工程](src/FPGA_ws/boards/ACG720_60K_robot/ACG720_60K_robot.gprj) · [接线与网络参数](src/FPGA_ws/boards/ACG720_60K_robot/README.md) |
| 小车 ROS 2 工作区 | [遥测与控制](src/ugv_ws/src/ugv_bridge/README.md) · [导航与任务](src/ugv_ws/src/car/car_navigation/README.md) · [建图启动](src/ugv_ws/src/car/car_bringup/README.md) |
| 六足 ROS 2 接入 | [树莓派桥接](src/ugv_ws/src/legged/legged_bridge/README.md) · [Jetson 雷达建图](src/ugv_ws/src/legged/legged_bringup/README.md) |
| 双机协同与网页 | [地图对齐和目标调度](src/swarm_ws/src/swarm_cooperation/README.md) · [网页控制台](src/website/README.md) |

## 部署前配置

- **板卡与网络。** 在 Gowin IDE 中打开对应的 `.gprj`。工程中的 IP 和主机 MAC 是示例值。按实际网卡设置板端 RTL、树莓派桥接参数和[小车 ROS 配置](src/ugv_ws/src/car/car_bringup/config/bringup.yaml)，再构建位流。修改 BX72 固件 C 源码后，先在板级目录运行 `python3 tools/build_firmware.py` 重新生成 ROM。
- **ROS 与设备依赖。** Jetson 和树莓派使用 ROS 2 Humble，配置相同的 `ROS_DOMAIN_ID` 并启用跨机通信。小车建图使用 slam_toolbox，六足建图还需要 RTAB-Map 的 `icp_odometry`。Jetson 视觉节点需要 CUDA、TensorRT、OpenCV 和单独部署的推理引擎。[软件依赖](src/ugv_ws/src/thirdparty/README.md)
- **协同与网页。** 在[设备名册](src/swarm_ws/config/robots.json)中填写实际主机地址和六足桥接 URL，并将 Jetson 的 `ACG720_UPSTREAM` 设为同一桥接 URL，默认端口为 `8766`。使用 systemd 用户服务时，在服务中设置该环境变量和实际安装目录。[网页部署](src/website/README.md)

## 构建与启动

两台机器分别进入自己的 `ugv_ws` 目录。首次部署或源码更新后运行对应的 `build` 子命令。日常启动只需执行每组的最后一行。

**树莓派**接收六足 FPGA 遥测并提供控制接口。在当前终端将 `PI_WIRED_IP` 和 `ACG_FPGA_IP` 分别设为树莓派直连网卡和六足 FPGA 的实际地址。

```bash
cd src/ugv_ws
bash Scripts/acg720_run.sh build
bash Scripts/acg720_run.sh --control-source "$PI_WIRED_IP" --fpga-address "$ACG_FPGA_IP"
```

**Jetson** 构建小车和协同工作区，启动建图、任务调度和网页服务。

```bash
cd src/ugv_ws
bash Scripts/bx72_run.sh build
bash Scripts/bx72_run.sh
```

在浏览器打开 Jetson 的主机名及端口 `8765`。只运行小车时使用 `bash Scripts/bx72_run.sh car`。每次启动会重新建图，开始双机协同前需让两台相机观测同一个 AprilTag，重新对齐地图。[启动模式与参数](src/ugv_ws/Scripts/README.md)

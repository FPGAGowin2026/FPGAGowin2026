# 第三方源码

本目录统一存放仓库维护的第三方源码。`bytetrack` 用于 TensorRT 检测节点，来源及许可证见其 `README.md` 和 `LICENSE`。

小车建图使用已安装的 slam_toolbox。Jetson 上已有的配套私有库可通过 `car_bringup` 的 `slam_library_dir` 选择。[建图启动说明](../car/car_bringup/README.md)

ROS、OpenCV、CUDA 和 TensorRT 等通过系统包或现有 SDK 安装路径查找；不复制二进制和系统安装目录到机器人业务包。

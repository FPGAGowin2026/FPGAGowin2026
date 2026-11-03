# VIO 适配桥

将 OpenVINS 里程计及其健康信息转换为 MAVROS / PX4 可用的视觉里程计，并检查时标、坐标与外参。默认配置位于 `config/uav_vio_bridge.yaml`，`enabled` 默认为 `false`；启用前需确认实际时钟、坐标变换与标定。

OpenVINS 的本地补丁位于 [patches](../../patches/open_vins-v2.7-vio.patch)。飞控接收与视觉定位融合效果仍需在对应设备上验证。

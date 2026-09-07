# SCC8660 DVP 例程边界

本工程在新的高电平 VSYNC 后采集 HREF 有效期间的 RGB565 流，高字节在前；行末半个像素会丢弃并通过 `partial_pixel_error` 报告。配置口按本地逐飞驱动格式发送 `0xA5, command, value[15:8], value[7:0]` 四字节 UART 帧，默认 9600 8N1。

工程**不包含**完整初始化表、相机应答接收、自动重试、帧缓存、YUV/RGB 格式转换或白平衡算法。`cfg_done` 仅证明发送完成；调用方仍需按模组固件版本发送正确命令并检查相机返回值。像素流处于 `camera_pclk` 域，连接系统处理流水线前需要明确的 CDC/异步 FIFO。

生产顶层通过 `debug_select` 将 RGB565、X 或 Y 复用到 `debug_data[15:0]`，并用 `capture_status[4:0]` 汇总采集事件，从而满足 MG121 的 I/O 数量限制。测试台直接实例化完整 core，仍检查所有像素、坐标及错误输出。

# OV7725 DVP 例程边界

本工程提供两部分可综合逻辑：RGB565 DVP 字节流采集，以及一次一个寄存器的 SCCB 写事务（器件写地址 `0x42`，并检查三个 ACK）。采集端必须先观察到新的高电平 VSYNC，随后才在 HREF 高电平期间按“高字节在前”拼接 RGB565；行末的奇数字节会丢弃并报告 `partial_pixel_error`。

工程**不包含** OV7725 的完整上电寄存器表、读寄存器事务、帧缓存、跨时钟 FIFO、图像缩放或颜色转换。上层控制器需要按实际模组和时钟源逐条提交 `cfg_addr/cfg_data`，并检查 `cfg_done/cfg_error`。像素输出属于 `camera_pclk` 时钟域，接入系统时钟域前必须增加异步 FIFO 或等价 CDC。

为适配 MG121 封装，生产顶层没有把全部像素和坐标同时引出，而用 `debug_select` 在 RGB565、X、Y 之间选择 `debug_data[15:0]`，其余事件压缩在 `capture_status[4:0]`。测试台直接实例化采集与 SCCB core，因此仍验证全部内部输出。

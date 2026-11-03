# 一个通过原串口累加校验的 16 点分组，不是完整扫描。
# header.stamp 为 ROS 接收时刻；timestamp_us 为 FPGA 快照提交时间。
# timestamp_us 保留 BX72 v1 原始 32 位微秒时间，尚未映射到 ROS 时钟。
# 保留解析器原始角度方向/零点；frame_id 不声明车体外参。
std_msgs/Header header
uint32 sequence
uint64 timestamp_us
uint32 dropped
uint32 crc_errors
uint32 length_errors
uint32 framing_errors
# BX72 v1 雷达包没有该计数；此时值为 0 且 available=false，不能解释为无错误。
bool framing_errors_available
# 原始角度 0.01deg，距离 mm，0 距离表示没有有效测距；强度为原始无量纲字节。
uint16[16] angle_cdeg
uint16[16] distance_mm
uint8[16] strength

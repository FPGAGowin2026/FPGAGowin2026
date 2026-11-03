# 四个物理通道 CH0/CH1/CH2/CH3，未映射轮位，未应用方向反相。
# header.stamp 为 ROS 接收时刻；timestamp_us 为 FPGA 快照时间。
# interval_us 是相邻接收快照的 FPGA 时间差，首包为 0。
std_msgs/Header header
uint32 sequence
uint64 timestamp_us
uint32 dropped
uint32 interval_us
# 有符号四倍频累计计数，32 位二补码自然回绕；不是弧度或轮速。
int32[4] ticks
# 本采样窗内非法 AB 双位跳变，bit0..3 对应 CH0..3；非法跳变不计数。
uint8 invalid_mask
# 表示完整电机快照已解码，没有独立硬件 ready 字段。
bool ready

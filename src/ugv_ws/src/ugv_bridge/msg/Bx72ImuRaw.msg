# header.stamp 为 ROS 接收时刻；timestamp_us 为 FPGA 遥测提交时刻，32 位微秒回绕。
# frame_id 表示芯片原始轴；安装到车体的变换尚未标定。
std_msgs/Header header
uint32 sequence
uint64 timestamp_us
uint32 dropped
uint16 reader_sequence
uint8 flags
uint8 who_am_i
# ±4g：0.122 mg/LSB；±500deg/s：17.5 mdps/LSB，顺序 X/Y/Z。
int16[3] accel_raw
int16[3] gyro_raw

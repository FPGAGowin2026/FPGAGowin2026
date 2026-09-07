# IMU660RA protocol basis and boundary

- Local vendor driver: `zf_device_imu660ra.[ch]` and `imu660ra_config_file配置数组.txt` under `C:\Users\32035\Desktop\Embedded_System`; Seekfree product page: <https://www.seekfree.com.cn/产品资料/传感器/姿态传感器/imu660ra六轴模块/>.
- Seekfree identifies the underlying device as BMI270: Mode 0 at 10 MHz, `CHIP_ID(0x00)=0x24`, one dummy byte on every SPI read, and little-endian output at `0x0C`/`0x12`.
- A fully initialized BMI270 requires the vendor 8192-byte feature configuration upload. This small FPGA example deliberately implements the verified transport, identity/dummy-byte validation and physical raw-frame capture boundary. `configuration_required` stays high: a soft core or host must upload the unmodified vendor blob before treating samples as calibrated sensor data.

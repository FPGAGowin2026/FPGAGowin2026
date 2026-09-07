# IMU963RA protocol basis and boundary

- Local vendor driver: `zf_device_imu963ra.[ch]` from the Seekfree TC264 library under `C:\Users\32035\Desktop\Embedded_System`.
- The accel/gyro die follows the verified ST-style map: Mode 0 at 10 MHz, `WHO_AM_I(0x0F)=0x6B`, little-endian gyro at `0x22` and acceleration at `0x28`.
- The module's magnetometer is behind the IMU sensor hub and needs a long, module-specific hub script. This direct-FPGA example implements verified accel/gyro initialization and capture, while exposing `mag_configuration_required=1`; it does not fabricate direct SPI magnetometer registers.

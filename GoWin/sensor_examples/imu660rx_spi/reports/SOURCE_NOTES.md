# IMU660RX protocol basis and boundary

- Local vendor driver: `zf_device_imu660rx.[ch]` plus its included IMU660RA/IMU660RB drivers under `C:\Users\32035\Desktop\Embedded_System`.
- Mode 0 at 10 MHz. The module may contain the RA/BMI270 branch (`0x00=0x24`, one read dummy byte) or RB/ST branch (`0x0F=0x6B`, no dummy byte); the core probes both without guessing.
- The example publishes the detected variant and captures correctly ordered raw frames. For the RA variant, `configuration_required` documents the mandatory 8192-byte BMI270 feature-configuration upload boundary.

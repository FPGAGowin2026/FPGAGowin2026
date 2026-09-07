# ICM20602 protocol basis

- Local vendor driver: `zf_device_icm20602.[ch]` from the Seekfree TC264 library under `C:\Users\32035\Desktop\Embedded_System`.
- SPI Mode 0, 10 MHz; read flag `0x80`; `WHO_AM_I(0x75)=0x12`; reset/config registers `0x6B/0x6C/0x1A/0x19/0x1B/0x1C/0x1D`.
- Output burst starts at `ACCEL_XOUT_H(0x3B)` and uses high-byte-first signed 16-bit axes. The core polls `INT_STATUS(0x3A).DATA_RDY` before publishing.

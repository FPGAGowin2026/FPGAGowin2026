# MENC15A protocol basis

- Local vendor driver: `zf_device_menc15a.[ch]` from the Seekfree TC264 library under `C:\Users\32035\Desktop\Embedded_System`.
- The driver configures SPI Mode 1 at 20 MHz and performs a 32-clock operation: command `0x8021`, then a 16-bit response. Position is response bits `[14:0]`; command `0x8032` similarly reads signed speed.
- The local vendor interface exposes no immutable identity or checksum field. The FPGA rejects the all-ones disconnected-bus signature and exposes the raw status bit instead of assigning it an undocumented meaning.

# MT6816 protocol basis

- Primary reference: MagnTek `MT6816 Rev.2.1`, sections 8.6.3 and 8.6.5; vendor product page: <https://www.magntek.com.cn/136/105>.
- Four-wire SPI idles SCK high, changes data on falling edges and samples on rising edges (Mode 3); maximum SCK is 16 MHz.
- Angle is `0x03[7:0] || 0x04[7:2]`; `0x04[1]` is weak-magnet warning and `0x04[0]` makes the 16 received bits even parity. `0x05[3]` reports overspeed.
- MT6816 exposes no immutable identity register in the cited public map. This project therefore validates parity and protocol framing rather than inventing an ID check.

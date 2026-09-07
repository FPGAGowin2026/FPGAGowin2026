# Normalized compatibility PnR results

| Chapter | Project | PnR | Timing | Setup endpoints | Hold endpoints | SHA-256 |
|---:|---|---|---|---:|---:|---|
| 48 | eth_udp_loopback_rgmii | PASS | CLEAN | 0 | 0 | `6793fc278d6b5db9ef7981c9e95328a1c780388e3fc7cf0bcb5ed01d14a7515e` |
| 54 | udp_send_rgmii | PASS | CLEAN | 0 | 0 | `30e1d35defcd882f23ddf065d2f353316868e45170a867ddc82905c6c1ab5e85` |
| 56 | eth_udp_rx_rgmii | PASS | VIOLATED | 11 | 0 | `dcd604e553c7adb41a4682d2bf31844cb018a8cfd5a490e552fbcd4d448bf8c8` |
| 61 | eth_udp_rgmii_ddr3_tft800x480 | PASS | VIOLATED | 525 | 13 | `dbeb1885c982bf6661b9bd3f1661c7898a45377f48f4101df8bd2785c0fcacc0` |
| 62 | ad7606_ddr3_rgmii | PASS | VIOLATED | 101 | 27 | `a9922489cbe3d471fb13fa333825eb7acf79e2b49a5443c2786a87c58a009326` |
| 63 | acm1030_ddr3_rgmii | PASS | VIOLATED | 123 | 19 | `232ab6a1d04833da45c8050572361b3641f3c5edf648c580591182acc5d09bf2` |

PnR PASS means placement, routing, timing analysis and bitstream generation completed. Timing closure is reported separately.

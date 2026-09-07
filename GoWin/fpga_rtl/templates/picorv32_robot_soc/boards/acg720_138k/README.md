# ACG720 / GW5AT-138K 迁移骨架

目标器件固定为项目当前已确认的 `GW5AT-LV138PG484AC1/I0`。本目录是 25K 实测模板迁往正式赛题板的接口契约，不是可直接烧录的板级工程；在拿到并核对 ACG720 原理图、实际扩展板接线和完整管脚表前，不添加猜测 CST。

## 初始配置

- CPU：Gowin_PicoRV32，RV32IMC，50 MHz 服务域。
- ITCM/DTCM：建议先用 32 KiB / 16 KiB；固件超限后按 map 文件调整。
- 启用：counter、interrupt、timer、Simple UART、Open Wishbone。
- 默认关闭：Pico JTAG debug、内部 GPIO/I²C/SPI。传感器先通过自定义 Wishbone mailbox 接入；只有证明能减少逻辑和接口重复时才启用官方外设。
- 软核不能直接连接 PWM、方向、驱动使能和急停释放。

## 接入顺序

1. 复用现有 `acg720_robot_network.gprj` 的正式器件与官方 RGMII/PLL 基线。
2. 建立统一 50 MHz 管理域；若机器人实时核保持其他频率，只通过双口 BRAM、异步 FIFO 和 toggle/commit 握手跨域。
3. 接入 `robot_service_regs`、状态 snapshot、参数 shadow/commit、路径双 bank。
4. 接入 `softcore_service_watchdog`；超时锁存故障、禁止新 commit/start，并向纯 RTL 路径执行器产生 abort。
5. 将现有 `demo_path_rom` 保留为无 CPU 恢复/演示后备路径。
6. 加入 SCCB/I²C/SPI 传感器初始化固件，再迁移 UART CLI。
7. 最后才接入整车外设并按安全清单做架空轮验证。

`config/pico_define_138k.vh` 给出软核逻辑配置参考；固件路径和器件生成物应由最终 ACG720 工程生成，不能直接复用 25K 的器件相关网表。

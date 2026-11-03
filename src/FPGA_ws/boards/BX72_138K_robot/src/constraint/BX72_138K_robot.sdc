# Board input reference clock.
create_clock -name clk50m_in -period 20.000 [get_ports {clk50m}]

# PHY 提供独立的千兆 RX 时钟；HCLK 通路的采样相位须重新实测。
# 采样 PLL 动态扫描 16 个相位；独立内部输出时钟使用局部时间原点 {0 4}，
# 不声明其相对 PHY 引脚的外部相位，仍定义完整 8 ns、50% 占空比波形。
# 本工具 generated-clock 的 PLL 路径在 MPW 报告中出现近零高脉宽；
# 与其他 PLL 一样在输出端建立内部时钟，PHY 到 IDDR 的板级输入预算仍待验证。
create_clock -name eth_rx_pin_clk -period 8.000 [get_ports {rgmii_rx_clk_i}]
create_generated_clock -name eth_rx_hclk -source [get_ports {rgmii_rx_clk_i}] -divide_by 1 [get_pins {rx_sampling_pll/u_rx_hclk/CLKOUT}]
create_clock -name eth_rx_clk -period 8.000 -waveform {0.000 4.000} [get_pins {rx_sampling_pll/u_pll/PLL_inst/CLKOUT0}]

# OV5647 1080p30 RAW10: 437.5 Mb/s per lane, Gear 8 byte clock = 54.6875 MHz.
# B2P uses two pixels/clock at 43.75 MHz and crosses domains through its FIFO.
create_clock -name camera_hs_clk -period 4.571429 [get_ports {camera_mipi_clk_p}]
create_generated_clock -name camera_byte_clk -source [get_ports {camera_mipi_clk_p}] -divide_by 4 [get_pins {ov5647_capture/camera_dphy_rx/DPHY_RX_INST_u_idesx8_Inst3_CLKDIV/CLKOUT}]
create_clock -name camera_pixel_clk -period 22.857 [get_pins {ov5647_capture/camera_pixel_pll/u_pll/PLL_inst/CLKOUT0}]
set_clock_groups -asynchronous -group [get_clocks {camera_byte_clk}] -group [get_clocks {camera_pixel_clk}]

# DDR reference PLL outputs: 50 MHz, 200 MHz, and 400 MHz (400 MHz unused).
create_clock -name ddr_clk50m -period 20.000 [get_pins {ddr3_pll/u_pll/PLL_inst/CLKOUT0}]
create_clock -name ddr_clk200m -period 5.000 [get_pins {ddr3_pll/u_pll/PLL_inst/CLKOUT1}]

# DDR3 IP 为 200 MHz、1:4，用户时钟为 50 MHz；在 IP 实际分频输出创建时钟。
# 分组采用对应138K厂商DDR3 1:4 IP的时钟合同，用户请求在ddr_ui_clk同域分析。
create_clock -name ddr_ui_clk -period 20.000 [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]

# 机器人控制与CPU共用40MHz，采集为33.333MHz；50MHz板钟继续为PLL/相位训练参考。

# 视频 PLL 提供 RGMII TX 与传感器时钟。
create_clock -name eth_clk125m -period 8.000 [get_pins {Gowin_PLL/u_pll/PLL_inst/CLKOUT0}]
create_clock -name control_clk40m -period 25.000 -waveform {0.000 12.500} [get_pins {Gowin_PLL/u_pll/PLL_inst/CLKOUT1}]
create_clock -name sensor_clk33m -period 30.000 [get_pins {Gowin_PLL/u_pll/PLL_inst/CLKOUT3}]
# PLL输出分频25：CPU、MMIO与运动保护同域；50MHz参考域承载包总线和相位训练/状态同步。
# 控制域到50MHz公共包总线只有窄FIFO/同步器，保持20ns传输上界，不要求相位一致。
set_max_delay -from [get_clocks {control_clk40m}] -to [get_clocks {clk50m_in ddr_clk50m}] 20.000
set_max_delay -from [get_clocks {clk50m_in ddr_clk50m}] -to [get_clocks {control_clk40m}] 20.000
set_false_path -hold -from [get_clocks {control_clk40m}] -to [get_clocks {clk50m_in ddr_clk50m}]
set_false_path -hold -from [get_clocks {clk50m_in ddr_clk50m}] -to [get_clocks {control_clk40m}]
# 控制与33MHz采集之间只有异步FIFO、观测保持握手和Gray同步；遥测先在各自产生域字节化。
# PLL释放相位不作为数据采样关系；保持20ns最大传输界限，异步hold不适用。
set_max_delay -from [get_clocks {clk50m_in control_clk40m}] -to [get_clocks {sensor_clk33m}] 20.000
set_max_delay -from [get_clocks {sensor_clk33m}] -to [get_clocks {clk50m_in control_clk40m}] 20.000
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m}] -to [get_clocks {sensor_clk33m}]
set_false_path -hold -from [get_clocks {sensor_clk33m}] -to [get_clocks {clk50m_in control_clk40m}]
# 简单RAW降采样与相机像素域同频，100MHz ISP及其跨域通路已退出活动设计。
set_max_delay -from [get_clocks {eth_rx_clk}] -to [get_clocks {eth_clk125m}] 8.000
set_max_delay -from [get_clocks {eth_clk125m}] -to [get_clocks {eth_rx_clk}] 8.000
set_false_path -hold -from [get_clocks {eth_rx_clk}] -to [get_clocks {eth_clk125m}]
set_false_path -hold -from [get_clocks {eth_clk125m}] -to [get_clocks {eth_rx_clk}]

# 速度整帧 FIFO 新增 RX→40 MHz 控制域及满指针返回；最大路径仍按较短周期约束。
set_max_delay -from [get_clocks {eth_rx_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m}] 8.000
set_max_delay -from [get_clocks {clk50m_in control_clk40m ddr_clk50m}] -to [get_clocks {eth_rx_clk}] 8.000
set_false_path -hold -from [get_clocks {eth_rx_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m}]
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m ddr_clk50m}] -to [get_clocks {eth_rx_clk}]

# The DDR, video, camera byte and camera pixel clocks have separate phase relationships.
# The Gowin B2P IP crosses the byte and pixel clocks through its internal FIFO;
# these two clocks are grouped asynchronously above. The frame buffer crosses
# the camera pixel domain through its asynchronous frame FIFO.
# Ethernet and sensor rates are phase-related PLL outputs, but all signals between
# these domains use the async pixel FIFO or explicit status synchronizers.
# 自定义 CDC 以两个域中较短周期限制最大延迟，并仅豁免异步 hold。
# 除 B2P 厂商 FIFO 的 byte/pixel 时钟组外，不使用跨域 set_clock_groups
# 或全路径 false_path，避免覆盖自定义 CDC 的最大延迟。
# 数值为内部传输上界，不代表外部接口延迟，也不替代 CDC 结构检查。
set_max_delay -from [get_clocks {camera_pixel_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] 10.000
set_false_path -hold -from [get_clocks {camera_pixel_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}]
set_max_delay -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {camera_pixel_clk}] 10.000
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {camera_pixel_clk}]
set_max_delay -from [get_clocks {camera_pixel_clk}] -to [get_clocks {eth_clk125m}] 8.000
set_false_path -hold -from [get_clocks {camera_pixel_clk}] -to [get_clocks {eth_clk125m}]
set_max_delay -from [get_clocks {eth_clk125m}] -to [get_clocks {camera_pixel_clk}] 8.000
set_false_path -hold -from [get_clocks {eth_clk125m}] -to [get_clocks {camera_pixel_clk}]
set_max_delay -from [get_clocks {camera_pixel_clk}] -to [get_clocks {sensor_clk33m}] 10.000
set_false_path -hold -from [get_clocks {camera_pixel_clk}] -to [get_clocks {sensor_clk33m}]
set_max_delay -from [get_clocks {sensor_clk33m}] -to [get_clocks {camera_pixel_clk}] 10.000
set_false_path -hold -from [get_clocks {sensor_clk33m}] -to [get_clocks {camera_pixel_clk}]
# CSI 字节域到 50 MHz 的诊断灰码、初始化完成同步器：限制布线延迟而不要求固定相位。
set_max_delay -from [get_clocks {camera_byte_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m}] 10.000
set_false_path -hold -from [get_clocks {camera_byte_clk}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m}]
set_max_delay -from [get_clocks {clk50m_in control_clk40m ddr_clk50m}] -to [get_clocks {camera_byte_clk}] 10.000
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m ddr_clk50m}] -to [get_clocks {camera_byte_clk}]
# Camera training status/tap diagnostics terminate in two-stage observation
# synchronizers in the sensor telemetry domain; the domains have no fixed phase.
set_max_delay -from [get_clocks {camera_byte_clk}] -to [get_clocks {sensor_clk33m}] 10.000
set_false_path -hold -from [get_clocks {camera_byte_clk}] -to [get_clocks {sensor_clk33m}]
set_max_delay -from [get_clocks {eth_clk125m}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] 8.000
set_false_path -hold -from [get_clocks {eth_clk125m}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}]
set_max_delay -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {eth_clk125m}] 8.000
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {eth_clk125m}]
set_max_delay -from [get_clocks {eth_clk125m}] -to [get_clocks {sensor_clk33m}] 8.000
set_false_path -hold -from [get_clocks {eth_clk125m}] -to [get_clocks {sensor_clk33m}]
set_max_delay -from [get_clocks {sensor_clk33m}] -to [get_clocks {eth_clk125m}] 8.000
set_false_path -hold -from [get_clocks {sensor_clk33m}] -to [get_clocks {eth_clk125m}]
set_max_delay -from [get_clocks {sensor_clk33m}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] 20.000
set_false_path -hold -from [get_clocks {sensor_clk33m}] -to [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}]
set_max_delay -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {sensor_clk33m}] 20.000
set_false_path -hold -from [get_clocks {clk50m_in control_clk40m ddr_clk50m ddr_ui_clk}] -to [get_clocks {sensor_clk33m}]

# 电机 UART 与 AB 编码器为异步输入，仅豁免第一级同步器。
set_false_path -from [get_ports {encoder_a_i[*] encoder_b_i[*]}] -to [get_regs {robot/motor_control_u/control/wheel[0].encoder/sync_a_q_0_s0 robot/motor_control_u/control/wheel[0].encoder/sync_b_q_0_s0 robot/motor_control_u/control/wheel[1].encoder/sync_a_q_0_s0 robot/motor_control_u/control/wheel[1].encoder/sync_b_q_0_s0 robot/motor_control_u/control/wheel[2].encoder/sync_a_q_0_s0 robot/motor_control_u/control/wheel[2].encoder/sync_b_q_0_s0 robot/motor_control_u/control/wheel[3].encoder/sync_a_q_0_s0 robot/motor_control_u/control/wheel[3].encoder/sync_b_q_0_s0}]


# 高云官方 DDR3_MC_PHY_1vs4_5a138k 的 ddr3_1v4_hs.sdc 分别约束
# 参考、内存、用户时钟，并将三域相互设为异步。仅适用于此供应商 IP 模型；
# 用户 DDR 请求/数据仍全部在 ddr_ui_clk 下进行同域时序分析。
set_clock_groups -asynchronous -group [get_clocks {clk50m_in ddr_clk50m}] -group [get_clocks {ddr_clk200m}] -group [get_clocks {ddr_ui_clk}]

# N10 UART 为异步输入，仅切断进入第一级同步器的路径。
set_false_path -from [get_ports {n10_uart_rx}] -to [get_regs {robot/radar_uart_rx/rx_meta_s0}]

# PLL 锁定复位允许异步断言，两级释放链的异步控制脚不要求与目的时钟同相。
# RX PLL 锁定单独复位应答 FIFO；视频 TX 与诊断总线释放链综合共享为 rx_phase_reset_sync。
# 仅豁免各释放链异步控制脚，保持级间 D 路径及链输出到状态机的恢复/移除检查。
# 级间 D 路径及链输出到状态机的 recovery/removal 保持检查。
# 动态相位扫描仅豁免其两级异步断言/同步释放链控制脚。

# 仅豁免异步断言/同步释放链的异步控制脚，仍检查级间 D 路径。
set_false_path -to [get_pins {sensor_reset_sync_*/CLEAR camera_reset_sync_*/CLEAR init_reset_sync_*/CLEAR robot/reset33_*/CLEAR robot/reset_ctrl_*/CLEAR}]
set_false_path -to [get_pins {udp_request_receiver/tx_reset_sync_*/PRESET udp_request_receiver/rx_reset_sync_*/PRESET udp_request_receiver/control_reset_sync_*/PRESET udp_request_receiver/command_bus_reset_sync_*/PRESET udp_request_receiver/good_reset_sync_*/PRESET telemetry_packet_cdc_u/tx_reset_sync_*/PRESET rx_phase_reset_sync_*/PRESET control_link_down_sync_*/PRESET}]
# 校验和局部复位是RX释放链stage1的保留副本；仅豁免其异步断言脚，D路径和客户恢复检查保留。
set_false_path -to [get_pins {udp_request_receiver/receiver/g_ip_checksum_reset_split.ip_checksum_reset_stage1_q_*/PRESET}]
set_false_path -to [get_pins {rx_sampling_pll/u_phase_scan/reset_meta_s0/PRESET rx_sampling_pll/u_phase_scan/reset_sync_s0/PRESET}]
set_false_path -to [get_pins {rx_sampling_pll/pll_lock_reset_meta_s0/PRESET rx_sampling_pll/pll_lock_reset_sync_s0/PRESET}]

# 原始 HS 域两级复位释放链允许异步断言；仅豁免链本身的异步控制脚。
# 级间 D 路径以及 stage2 到 DPHY/IDES/CLKDIV 的 recovery/removal 仍检查。
set_false_path -to [get_pins {ov5647_capture/camera_dphy_rx/dphy_reset_release_meta_ff/CLEAR ov5647_capture/camera_dphy_rx/dphy_reset_release_sync_ff/CLEAR}]
set_false_path -to [get_pins {ov5647_capture/camera_dphy_rx/dphy_byte_reset_release_meta_ff/CLEAR ov5647_capture/camera_dphy_rx/dphy_byte_reset_release_sync_ff/CLEAR}]

# IDES/tap独立复位链仅五颗FF的异步PRESET脚豁免（共用stage1与各lane stage2）；Q至IDES恢复/移除仍检查。
set_false_path -to [get_pins {ov5647_capture/camera_dphy_rx/dphy_ides_reset_release_meta_ff/PRESET ov5647_capture/camera_dphy_rx/dphy_ides_reset_release_lane0_sync_ff/PRESET ov5647_capture/camera_dphy_rx/dphy_ides_reset_release_lane1_sync_ff/PRESET ov5647_capture/camera_dphy_rx/dphy_tap_reset_release_lane0_sync_ff/PRESET ov5647_capture/camera_dphy_rx/dphy_tap_reset_release_lane1_sync_ff/PRESET}]

# 详细覆盖相机、DDR、以太网和传感器时钟域；仅用于报告，不改变时序例外。
report_timing -setup -max_paths 50 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {control_clk40m}] -to_clock [get_clocks {control_clk40m}] -max_paths 20 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {camera_byte_clk}] -to_clock [get_clocks {camera_byte_clk}] -max_paths 5 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {camera_pixel_clk}] -to_clock [get_clocks {camera_pixel_clk}] -max_paths 5 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {ddr_ui_clk}] -to_clock [get_clocks {ddr_ui_clk}] -max_paths 20 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {eth_clk125m}] -to_clock [get_clocks {eth_clk125m}] -max_paths 5 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {eth_rx_clk}] -to_clock [get_clocks {eth_rx_clk}] -max_paths 5 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {sensor_clk33m}] -to_clock [get_clocks {sensor_clk33m}] -max_paths 50 -max_common_paths 1

# 枚举相机控制的全部同源路径；仅增加报告覆盖，不改变约束。
report_timing -setup -from_clock [get_clocks {camera_hs_clk}] -to_clock [get_clocks {camera_hs_clk}] -max_paths 100 -max_common_paths 8
report_timing -setup -from_clock [get_clocks {camera_byte_clk}] -to_clock [get_clocks {camera_hs_clk}] -max_paths 100 -max_common_paths 8
report_timing -recovery -from_clock [get_clocks {camera_hs_clk}] -to_clock [get_clocks {camera_hs_clk}] -max_paths 100 -max_common_paths 8

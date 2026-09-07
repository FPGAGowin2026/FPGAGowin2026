create_clock -name system_clk -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50m_i}]
# 无外设 DEMO wrapper 只导出 USB-UART，视频链在该 profile 中会被综合优化。
# 完整摄像头/DVI 时钟约束由 release profile 的 SDC 覆盖。

# Tang Primer 25K 板载时钟与 OV5640 最坏 PCLK 约束。
create_clock -name system_clk -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50m_i}]
create_clock -name camera_pclk -period 10.417 -waveform {0.000 5.208} [get_ports {ov_pclk_i}]

# PLL 目标：serial≈125.781MHz、pixel≈25.156MHz、camera_xclk≈23.958MHz。
# fractional PLL 的自动时钟在综合阶段会被命名为 default_gen_clk，但尚不能在
# PnR 解析 SDC 时按该内部名称查询；因此在顶层 pixel_clk 网显式声明精确 161/320 比例。
create_generated_clock -name pixel_clk -source [get_ports {clk_50m_i}] -master_clock system_clk -divide_by 320 -multiply_by 161 [get_nets {pixel_clk}]
# OV5640 的 PCLK 由相机内部生成，与板载 50MHz/同源像素时钟均无固定相位关系；
# 相关跨域只经过双缓冲握手与同步器。system/pixel 同源，仍保留彼此间的 STA。
set_clock_groups -asynchronous -group [get_clocks {camera_pclk}] -group [get_clocks {system_clk pixel_clk}]

report_max_frequency -mod_ins {u_clocks}

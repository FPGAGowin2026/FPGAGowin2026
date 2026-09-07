open_project acg720_robot_network_offline.gprj
set_option -top_module acg720_robot_network_offline_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
# 官方 ch58 约束将 U21/R19/P19 用作 RGMII；GW5AT 数据库要求显式释放
# 这些 CPU/SSPI 复用脚。不改变管脚位置，仅复现官方板级用法。
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
# 125 MHz 网络域余量较小，使用工具支持的 timing-priority placement/routing。
set_option -timing_driven 1
set_option -place_option 2
set_option -route_option 1
run pnr

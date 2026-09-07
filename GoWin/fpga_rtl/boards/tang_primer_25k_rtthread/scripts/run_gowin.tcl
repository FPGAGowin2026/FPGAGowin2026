set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
set project_file [file join $project_dir "tang_primer_25k_rtthread.gprj"]

cd $project_dir
open_project $project_file
set_option -top_module tang_primer_25k_rtthread_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
# E2 与板载 Flash 都属于双用途配置引脚，必须在用户模式释放。
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_cpu_as_gpio 1
# 50 MHz 下 GoRV32 的最差路径余量很窄，强制布局和布线优先满足时序。
set_option -timing_driven 1
set_option -place_option 2
set_option -route_option 1
run syn
run pnr

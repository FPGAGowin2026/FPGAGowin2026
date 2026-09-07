set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
set project_file [file join $project_dir "primer25k_uart_bringup.gprj"]

cd $project_dir
open_project $project_file
set_option -top_module tang_primer_25k_bringup_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
# E2 是核心板 50 MHz 时钟，同时也是芯片的 CPU/SSPI 复用脚。
# 释放双用途脚后才能按 Sipeed 官方原理图把它作为普通时钟输入。
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run syn
run pnr

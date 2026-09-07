set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
set project_file [file join $project_dir "tang_primer_25k_picorv32.gprj"]

cd $project_dir
open_project $project_file
set_option -top_module tang_primer_25k_picorv32_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
# E2 同时是芯片 CPU/SSPI 复用脚；必须释放后才能作为 50 MHz 时钟输入。
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run syn
run pnr

set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
cd $project_dir
open_project [file join $project_dir "choice3_robot_25k.gprj"]
set_option -top_module choice3_robot_25k_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_i2c_as_gpio 1
run syn
run pnr

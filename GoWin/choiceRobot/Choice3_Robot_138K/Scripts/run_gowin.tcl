set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
set project_file [file join $project_dir "choice3_robot_138k.gprj"]
cd $project_dir
open_project $project_file
set_option -top_module choice3_robot_138k_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
run syn

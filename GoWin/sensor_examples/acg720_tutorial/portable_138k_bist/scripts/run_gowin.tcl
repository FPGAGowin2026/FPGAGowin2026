set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ".."]]
set project_file [file join $project_dir "acg720_tutorial_138k_bist.gprj"]

cd $project_dir
open_project $project_file
set_option -top_module acg720_tutorial_138k_bist_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
run syn
run pnr

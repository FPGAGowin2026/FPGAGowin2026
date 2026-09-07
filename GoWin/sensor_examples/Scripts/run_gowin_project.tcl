if {$argc != 1} {
    error "usage: gw_sh run_gowin_project.tcl <sensor_example.gprj>"
}

set project_file [file normalize [lindex $argv 0]]
if {![file exists $project_file]} {
    error "project does not exist: $project_file"
}

cd [file dirname $project_file]
open_project $project_file
set_option -top_module sensor_example_top
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1
run syn
run pnr

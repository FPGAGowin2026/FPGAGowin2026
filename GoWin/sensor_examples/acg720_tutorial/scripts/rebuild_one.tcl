if {$argc < 2} {
    puts stderr "usage: gw_sh rebuild_one.tcl PROJECT.gprj syn|pnr ?TOP_MODULE?"
    exit 64
}

set project_file [file normalize [lindex $argv 0]]
set flow [string tolower [lindex $argv 1]]
set top_module ""
if {$argc >= 3} {
    set top_module [lindex $argv 2]
}

if {![file exists $project_file]} {
    puts stderr "PROJECT_NOT_FOUND $project_file"
    exit 66
}
if {$flow ne "syn" && $flow ne "pnr"} {
    puts stderr "INVALID_FLOW $flow"
    exit 64
}

set project_dir [file dirname $project_file]
cd $project_dir
puts "REBUILD_BEGIN project=$project_file flow=$flow top=$top_module"

if {[catch {open_project $project_file} open_result]} {
    puts stderr "OPEN_PROJECT_FAIL $open_result"
    exit 2
}
if {$top_module ne ""} {
    set_option -top_module $top_module
}
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1

if {[catch {run syn} syn_result]} {
    puts stderr "SYN_COMMAND_FAIL $syn_result"
    exit 3
}
puts "SYN_COMMAND_DONE result=$syn_result"

if {$flow eq "pnr"} {
    if {[catch {run pnr} pnr_result]} {
        puts stderr "PNR_COMMAND_FAIL $pnr_result"
        exit 4
    }
    puts "PNR_COMMAND_DONE result=$pnr_result"
}

puts "REBUILD_END project=$project_file flow=$flow"
exit 0

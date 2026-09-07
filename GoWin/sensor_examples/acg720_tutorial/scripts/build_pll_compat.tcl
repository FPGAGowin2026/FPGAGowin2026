if {$argc < 2} {
    puts stderr "usage: gw_sh build_pll_compat.tcl PROJECT.gprj TOP_MODULE"
    exit 64
}

set project_file [file normalize [lindex $argv 0]]
set top_module [lindex $argv 1]
cd [file dirname $project_file]
puts "PLL_COMPAT_BEGIN project=$project_file top=$top_module"

if {[catch {open_project $project_file} open_result]} {
    puts stderr "OPEN_PROJECT_FAIL $open_result"
    exit 2
}

set_option -top_module $top_module
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1

# These ACG720 projects intentionally use configuration-capable package pins.
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1

if {[catch {run syn} syn_result]} {
    puts stderr "SYN_COMMAND_FAIL $syn_result"
    exit 3
}
puts "SYN_COMMAND_DONE result=$syn_result"

if {[catch {run pnr} pnr_result]} {
    puts stderr "PNR_COMMAND_FAIL $pnr_result"
    exit 4
}
puts "PNR_COMMAND_DONE result=$pnr_result"
puts "PLL_COMPAT_END"
exit 0

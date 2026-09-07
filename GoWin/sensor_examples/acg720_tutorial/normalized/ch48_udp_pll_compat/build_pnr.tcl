set project_file [file normalize [file join [file dirname [info script]] eth_udp_loopback_rgmii.gprj]]
set project_dir [file dirname $project_file]

cd $project_dir
puts "CH48_COMPAT_BEGIN project=$project_file"

if {[catch {open_project $project_file} open_result]} {
    puts stderr "OPEN_PROJECT_FAIL $open_result"
    exit 2
}

set_option -top_module eth_udp_loopback_rgmii
set_option -verilog_std v2001
set_option -print_all_synthesis_warning 1

# ACG720 routes three RGMII signals through configuration-capable pins.
# These options reproduce the board project's original pin-reuse settings.
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
puts "CH48_COMPAT_END"
exit 0

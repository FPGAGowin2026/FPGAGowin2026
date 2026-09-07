if {[catch {
    create_project -name gorv32_probe -pn GW5A-LV25MG121NC1/I0 -device_version B -force
    create_ipc -name gowin_gorv32 -language Verilog -module_name gorv32_rt -file_name gorv32_rt.v
    generate_target [get_ips gorv32_rt]
} message options]} {
    puts stderr "GORV32_IP_GENERATION_ERROR=$message"
    puts stderr "GORV32_IP_GENERATION_ERRORINFO=[dict get $options -errorinfo]"
    exit 2
}
puts "GORV32_IP_GENERATION_PASS"

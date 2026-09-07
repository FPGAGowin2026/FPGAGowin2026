create_project -name gorv32_live -pn GW5A-LV25MG121NC1/I0 -device_version B
create_ipc -name gowin_gorv32 -language Verilog -module_name gorv32_rt -file_name gorv32_rt.v
generate_target [get_ips gorv32_rt ] 

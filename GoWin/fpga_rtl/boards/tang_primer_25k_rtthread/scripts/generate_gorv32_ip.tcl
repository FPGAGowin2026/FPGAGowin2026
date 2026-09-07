# 为 Tang Primer 25K 生成最小 GoRV32：64 KiB 片上 RAM + UART1。
# 使用显式 IPC 是为了避开 Gowin 1.9.12.03 中 create_ipc 默认对象无法写属性的问题。
set script_dir [file dirname [file normalize [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
set ipc_file [file normalize [file join $project_dir ip gorv32_rt.ipc]]

cd $project_dir
if {[catch {
    create_project -name tang_primer_25k_rtthread_ipgen \
        -dir $project_dir \
        -pn GW5A-LV25MG121NC1/I0 \
        -device_version B \
        -force
    set_device -name GW5A-25B -device_version B GW5A-LV25MG121NC1/I0
    read_ipc $ipc_file
    generate_target [get_ips gorv32_rt]
} message options]} {
    puts stderr "GORV32_IP_GENERATION_ERROR=<$message>"
    puts stderr "GORV32_IP_GENERATION_ERRORCODE=<[dict get $options -errorcode]>"
    puts stderr "GORV32_IP_GENERATION_ERRORINFO=<[dict get $options -errorinfo]>"
    exit 2
}
puts "GORV32_IP_GENERATION_PASS"

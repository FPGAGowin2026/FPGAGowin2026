onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /i2c_control_tb/Clk
add wave -noupdate /i2c_control_tb/Rst_n
add wave -noupdate /i2c_control_tb/wrreg_req
add wave -noupdate /i2c_control_tb/rdreg_req
add wave -noupdate /i2c_control_tb/addr
add wave -noupdate /i2c_control_tb/addr_mode
add wave -noupdate /i2c_control_tb/wrdata
add wave -noupdate /i2c_control_tb/rddata
add wave -noupdate /i2c_control_tb/device_id
add wave -noupdate /i2c_control_tb/RW_Done
add wave -noupdate /i2c_control_tb/ack
add wave -noupdate /i2c_control_tb/i2c_sclk
add wave -noupdate /i2c_control_tb/i2c_sdat
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {882441 ns}

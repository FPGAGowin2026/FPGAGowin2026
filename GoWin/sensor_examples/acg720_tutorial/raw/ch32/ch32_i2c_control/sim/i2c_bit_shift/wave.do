onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /i2c_bit_shift_tb/Clk
add wave -noupdate /i2c_bit_shift_tb/Rst_n
add wave -noupdate -radix binary /i2c_bit_shift_tb/Cmd
add wave -noupdate /i2c_bit_shift_tb/Go
add wave -noupdate /i2c_bit_shift_tb/Rx_DATA
add wave -noupdate /i2c_bit_shift_tb/Tx_DATA
add wave -noupdate /i2c_bit_shift_tb/Trans_Done
add wave -noupdate /i2c_bit_shift_tb/ack_o
add wave -noupdate /i2c_bit_shift_tb/i2c_sclk
add wave -noupdate /i2c_bit_shift_tb/i2c_sdat
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {74105805 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 70
configure wave -valuecolwidth 40
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
WaveRestoreZoom {0 ps} {123718256 ps}

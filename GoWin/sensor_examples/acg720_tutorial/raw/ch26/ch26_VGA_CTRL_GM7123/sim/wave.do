onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/Clk25M
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/Rst_n
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/data_in
add wave -noupdate -radix ufixed /VGA_CTRL_tb/VGA_CTRL/hcount
add wave -noupdate -radix ufixed /VGA_CTRL_tb/VGA_CTRL/vcount
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/VGA_RGB
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/VGA_HS
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/VGA_VS
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/VGA_BLK
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/VGA_CLK
add wave -noupdate -radix ufixed /VGA_CTRL_tb/VGA_CTRL/hcount_r
add wave -noupdate -radix ufixed /VGA_CTRL_tb/VGA_CTRL/vcount_r
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/hcount_ov
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/vcount_ov
add wave -noupdate /VGA_CTRL_tb/VGA_CTRL/dat_act
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {6688774 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 269
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
configure wave -timelineunits ns
update
WaveRestoreZoom {6687398 ns} {6688978 ns}

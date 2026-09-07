onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /key_filter_tb/clk
add wave -noupdate /key_filter_tb/reset_n
add wave -noupdate /key_filter_tb/key_press
add wave -noupdate /key_filter_tb/key
add wave -noupdate /key_filter_tb/key_flag
add wave -noupdate /key_filter_tb/key_state
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {27112637000 ps} 0}
quietly wave cursor active 1
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
WaveRestoreZoom {59062717054 ps} {66937745996 ps}

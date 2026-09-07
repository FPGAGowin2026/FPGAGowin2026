onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /shift_register_2taps_tb/clk
add wave -noupdate /shift_register_2taps_tb/reset_p
add wave -noupdate /shift_register_2taps_tb/shiftin
add wave -noupdate /shift_register_2taps_tb/shiftin_valid
add wave -noupdate /shift_register_2taps_tb/shiftout
add wave -noupdate /shift_register_2taps_tb/taps1x
add wave -noupdate /shift_register_2taps_tb/taps0x
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {34197866 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 40
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
WaveRestoreZoom {32962097 ps} {34981158 ps}

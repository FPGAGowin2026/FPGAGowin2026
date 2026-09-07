onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /ir_decode_tb/clk
add wave -noupdate /ir_decode_tb/reset_n
add wave -noupdate /ir_decode_tb/iIR
add wave -noupdate /ir_decode_tb/dec_done
add wave -noupdate /ir_decode_tb/ir_data
add wave -noupdate /ir_decode_tb/ir_addr
add wave -noupdate /ir_decode_tb/i
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
configure wave -namecolwidth 162
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
WaveRestoreZoom {0 ns} {253337911 ns}

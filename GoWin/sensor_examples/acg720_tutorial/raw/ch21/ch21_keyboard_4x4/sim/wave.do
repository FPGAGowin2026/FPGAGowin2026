onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /keyboaed_4x4_tb/clk
add wave -noupdate /keyboaed_4x4_tb/reset_n
add wave -noupdate /keyboaed_4x4_tb/press_key
add wave -noupdate /keyboaed_4x4_tb/press_key_value
add wave -noupdate /keyboaed_4x4_tb/keyboard_row_x4
add wave -noupdate /keyboaed_4x4_tb/keyboard_col_x4
add wave -noupdate /keyboaed_4x4_tb/key_flag
add wave -noupdate /keyboaed_4x4_tb/key_value
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
configure wave -namecolwidth 96
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
WaveRestoreZoom {0 ns} {157500274 ns}

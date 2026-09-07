onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /music_gen_tb/clk
add wave -noupdate /music_gen_tb/reset_n
add wave -noupdate /music_gen_tb/beep
add wave -noupdate /music_gen_tb/music_gen/counter_arr
add wave -noupdate /music_gen_tb/music_gen/counter_ccr
add wave -noupdate /music_gen_tb/music_gen/pitch_num
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 129
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
WaveRestoreZoom {0 ps} {1284433321500 ps}

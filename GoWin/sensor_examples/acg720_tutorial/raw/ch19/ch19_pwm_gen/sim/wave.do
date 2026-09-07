onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /pwm_gen_tb/clk
add wave -noupdate /pwm_gen_tb/reset_n
add wave -noupdate /pwm_gen_tb/pwm_gen_en
add wave -noupdate /pwm_gen_tb/counter_arr
add wave -noupdate /pwm_gen_tb/counter_ccr
add wave -noupdate /pwm_gen_tb/pwm_out
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
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
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ns} {316472 ns}

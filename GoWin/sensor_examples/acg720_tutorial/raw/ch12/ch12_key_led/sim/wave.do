onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /key_led_tb/clk
add wave -noupdate /key_led_tb/reset_n
add wave -noupdate /key_led_tb/press0
add wave -noupdate /key_led_tb/press1
add wave -noupdate /key_led_tb/press2
add wave -noupdate /key_led_tb/press3
add wave -noupdate /key_led_tb/key_in0
add wave -noupdate /key_led_tb/key_in1
add wave -noupdate /key_led_tb/key_in2
add wave -noupdate /key_led_tb/key_in3
add wave -noupdate -radix binary /key_led_tb/led
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 144
configure wave -valuecolwidth 74
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
WaveRestoreZoom {0 ns} {672000589 ns}

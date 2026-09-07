onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /hex8_tb/clk
add wave -noupdate /hex8_tb/reset_n
add wave -noupdate /hex8_tb/en
add wave -noupdate /hex8_tb/disp_data
add wave -noupdate -radix binary /hex8_tb/sel
add wave -noupdate -radix binary /hex8_tb/seg
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {47490109 ns} 0}
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
configure wave -timelineunits ns
update
WaveRestoreZoom {43178145 ns} {48761883 ns}

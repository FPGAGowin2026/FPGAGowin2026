onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /ram_tb/reset
add wave -noupdate /ram_tb/clk
add wave -noupdate /ram_tb/wea
add wave -noupdate /ram_tb/ad
add wave -noupdate /ram_tb/dina
add wave -noupdate /ram_tb/doutb
add wave -noupdate /ram_tb/i
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {200214 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 63
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
WaveRestoreZoom {203008 ps} {474248 ps}

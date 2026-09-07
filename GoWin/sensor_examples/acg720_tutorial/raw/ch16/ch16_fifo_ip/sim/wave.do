onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /fifo_ip_tb/Data
add wave -noupdate /fifo_ip_tb/Reset
add wave -noupdate /fifo_ip_tb/WrClk
add wave -noupdate /fifo_ip_tb/RdClk
add wave -noupdate /fifo_ip_tb/WrEn
add wave -noupdate /fifo_ip_tb/RdEn
add wave -noupdate /fifo_ip_tb/Wnum
add wave -noupdate /fifo_ip_tb/Rnum
add wave -noupdate /fifo_ip_tb/Almost_Empty
add wave -noupdate /fifo_ip_tb/Almost_Full
add wave -noupdate /fifo_ip_tb/Q
add wave -noupdate /fifo_ip_tb/Empty
add wave -noupdate /fifo_ip_tb/Full
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
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
WaveRestoreZoom {0 ps} {13211100 ps}

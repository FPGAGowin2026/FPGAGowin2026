onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /uart_byte_tx_tb/clk
add wave -noupdate /uart_byte_tx_tb/reset_n
add wave -noupdate /uart_byte_tx_tb/data_byte
add wave -noupdate /uart_byte_tx_tb/send_en
add wave -noupdate /uart_byte_tx_tb/baud_set
add wave -noupdate /uart_byte_tx_tb/uart_tx
add wave -noupdate /uart_byte_tx_tb/tx_done
add wave -noupdate /uart_byte_tx_tb/uart_state
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {8953 ns} 0}
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
WaveRestoreZoom {3770 ns} {157289 ns}

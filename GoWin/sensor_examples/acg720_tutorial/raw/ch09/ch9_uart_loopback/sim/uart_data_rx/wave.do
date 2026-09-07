onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /uart_data_rx_tb/Clk
add wave -noupdate /uart_data_rx_tb/Rst_n
add wave -noupdate /uart_data_rx_tb/data
add wave -noupdate /uart_data_rx_tb/rx_data
add wave -noupdate /uart_data_rx_tb/send_en
add wave -noupdate /uart_data_rx_tb/uart_tx
add wave -noupdate /uart_data_rx_tb/uart_rx
add wave -noupdate /uart_data_rx_tb/Tx_Done
add wave -noupdate /uart_data_rx_tb/uart_state
add wave -noupdate /uart_data_rx_tb/Rx_Done
add wave -noupdate /uart_data_rx_tb/timeout_flag
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
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
WaveRestoreZoom {0 ns} {3618070 ns}

onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /uart_adjust_time_tb/clk50M
add wave -noupdate /uart_adjust_time_tb/reset_n
add wave -noupdate /uart_adjust_time_tb/uart_tx_data
add wave -noupdate /uart_adjust_time_tb/uart_tx_en
add wave -noupdate /uart_adjust_time_tb/set_done
add wave -noupdate /uart_adjust_time_tb/uart_tx_done
add wave -noupdate /uart_adjust_time_tb/uart_rx
add wave -noupdate /uart_adjust_time_tb/uart_rx_data
add wave -noupdate /uart_adjust_time_tb/uart_data_valid
add wave -noupdate /uart_adjust_time_tb/set_time
add wave -noupdate /uart_adjust_time_tb/set_time_data
add wave -noupdate /uart_adjust_time_tb/set_date
add wave -noupdate /uart_adjust_time_tb/set_date_data
add wave -noupdate /uart_adjust_time_tb/adjust_cmd_data_reg
add wave -noupdate /uart_adjust_time_tb/i
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
configure wave -namecolwidth 327
configure wave -valuecolwidth 40
configure wave -justifyvalue left
configure wave -signalnamewidth 1
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
WaveRestoreZoom {0 ns} {27730015 ns}

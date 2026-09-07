onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /uart_rx_ctrl_led_tb/Clk
add wave -noupdate /uart_rx_ctrl_led_tb/Reset_n
add wave -noupdate /uart_rx_ctrl_led_tb/Led
add wave -noupdate /uart_rx_ctrl_led_tb/uart_rx
add wave -noupdate /uart_rx_ctrl_led_tb/delay_time
add wave -noupdate /uart_rx_ctrl_led_tb/uart_rx_ctrl_led/counter_led/counter2
add wave -noupdate /uart_rx_ctrl_led_tb/uart_rx_ctrl_led/uart_cmd/time_set
add wave -noupdate /uart_rx_ctrl_led_tb/uart_rx_ctrl_led/uart_cmd/ctrl
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
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
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {19031839050 ps}

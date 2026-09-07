onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /uart_eeprom_tb/uart_eeprom/clk50M
add wave -noupdate /uart_eeprom_tb/uart_eeprom/rst_n
add wave -noupdate /uart_eeprom_tb/uart_eeprom/i2c_sdat
add wave -noupdate /uart_eeprom_tb/uart_eeprom/i2c_sclk
add wave -noupdate /uart_eeprom_tb/uart_eeprom/convert/rddata
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_tx
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_rx
add wave -noupdate /uart_eeprom_tb/uart_eeprom/addr
add wave -noupdate /uart_eeprom_tb/uart_eeprom/ack
add wave -noupdate /uart_eeprom_tb/uart_eeprom/RW_Done
add wave -noupdate /uart_eeprom_tb/uart_eeprom/eeprom_rd_done
add wave -noupdate /uart_eeprom_tb/uart_eeprom/eeprom_wr_done
add wave -noupdate /uart_eeprom_tb/uart_eeprom/eeprom_rddata
add wave -noupdate /uart_eeprom_tb/uart_eeprom/wrreg_req
add wave -noupdate /uart_eeprom_tb/uart_eeprom/rdreg_req
add wave -noupdate /uart_eeprom_tb/uart_eeprom/wrdata
add wave -noupdate /uart_eeprom_tb/uart_eeprom/rddata
add wave -noupdate /uart_eeprom_tb/uart_eeprom/address
add wave -noupdate /uart_eeprom_tb/uart_eeprom/cmd_data
add wave -noupdate /uart_eeprom_tb/uart_eeprom/num_cmd
add wave -noupdate /uart_eeprom_tb/uart_eeprom/cmdvalid
add wave -noupdate /uart_eeprom_tb/uart_eeprom/rx_data
add wave -noupdate /uart_eeprom_tb/uart_eeprom/rx_done
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_data_tx/uart_byte_tx/data_byte
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_data_tx/uart_byte_tx/bps_cnt
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_byte_rx/Data
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_byte_rx/bps_cnt
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_cmd/address
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_cmd/data
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_cmd/cmdvalid
add wave -noupdate /uart_eeprom_tb/uart_eeprom/uart_cmd/num_cmd
add wave -noupdate /uart_eeprom_tb/uart_eeprom/convert/wrdata
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {4303708130 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 388
configure wave -valuecolwidth 100
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
configure wave -timelineunits ps
update
WaveRestoreZoom {1790478280 ps} {5542700620 ps}

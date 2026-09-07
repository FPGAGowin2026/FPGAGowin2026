onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /usb_stream_out_tb/source_ready
add wave -noupdate /usb_stream_out_tb/fx2_flagb
add wave -noupdate /usb_stream_out_tb/fx2_sloe
add wave -noupdate /usb_stream_out_tb/fx2_slrd
add wave -noupdate /usb_stream_out_tb/usb_data_valid
add wave -noupdate /usb_stream_out_tb/usb_data
add wave -noupdate /usb_stream_out_tb/fifo_empty
add wave -noupdate /usb_stream_out_tb/fifo_usedw
add wave -noupdate /usb_stream_out_tb/fx2_flagc
add wave -noupdate /usb_stream_out_tb/fx2_ifclk
add wave -noupdate /usb_stream_out_tb/reset_n
add wave -noupdate /usb_stream_out_tb/fifo_q
add wave -noupdate /usb_stream_out_tb/fx2_faddr
add wave -noupdate /usb_stream_out_tb/fx2_slwr
add wave -noupdate /usb_stream_out_tb/fifo_rdreq
add wave -noupdate /usb_stream_out_tb/cnt
add wave -noupdate /usb_stream_out_tb/fx2_fdata
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {50222907 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 134
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
WaveRestoreZoom {49993123 ps} {50610748 ps}

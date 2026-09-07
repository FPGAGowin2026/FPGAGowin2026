onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /image_stitche_x_tb/clk_image1_in
add wave -noupdate /image_stitche_x_tb/clk_image2_in
add wave -noupdate /image_stitche_x_tb/clk_image_out
add wave -noupdate /image_stitche_x_tb/reset_p
add wave -noupdate /image_stitche_x_tb/rst_busy_o
add wave -noupdate /image_stitche_x_tb/image1_data_pixel_i
add wave -noupdate /image_stitche_x_tb/image1_data_valid_i
add wave -noupdate /image_stitche_x_tb/image2_data_pixel_i
add wave -noupdate /image_stitche_x_tb/image2_data_valid_i
add wave -noupdate /image_stitche_x_tb/data_pixel_o
add wave -noupdate /image_stitche_x_tb/data_valid_o
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 40
configure wave -valuecolwidth 40
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
WaveRestoreZoom {0 ps} {13157550 ps}

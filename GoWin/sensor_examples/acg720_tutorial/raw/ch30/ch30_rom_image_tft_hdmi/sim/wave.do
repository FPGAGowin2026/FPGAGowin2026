onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/clk_ctrl
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/reset_n
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/img_disp_hbegin
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/img_disp_vbegin
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/disp_back_color
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/rom_addra
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/rom_data
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/Frame_Begin
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/disp_data_req
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/visible_hcount
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/visible_vcount
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/disp_data
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/h_exceed
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/v_exceed
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/img_h_disp
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/img_v_disp
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/img_disp
add wave -noupdate /rom_image_tft_tb/rom_image_tft_hdmi/image_extract/hcount_max
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
WaveRestoreZoom {0 ps} {17761151184 ps}

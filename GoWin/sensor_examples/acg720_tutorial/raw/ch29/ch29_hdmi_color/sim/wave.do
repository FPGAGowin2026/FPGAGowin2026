onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /hdmi_color_tb/hdmi_color/clk
add wave -noupdate /hdmi_color_tb/hdmi_color/rst_n
add wave -noupdate /hdmi_color_tb/hdmi_color/led
add wave -noupdate /hdmi_color_tb/hdmi_color/tmds_clk_p
add wave -noupdate /hdmi_color_tb/hdmi_color/tmds_clk_n
add wave -noupdate /hdmi_color_tb/hdmi_color/tmds_data_p
add wave -noupdate /hdmi_color_tb/hdmi_color/tmds_data_n
add wave -noupdate /hdmi_color_tb/hdmi_color/clk_vga
add wave -noupdate /hdmi_color_tb/hdmi_color/clk_vgax5
add wave -noupdate /hdmi_color_tb/hdmi_color/pll_lock
add wave -noupdate /hdmi_color_tb/hdmi_color/disp_data_req
add wave -noupdate /hdmi_color_tb/hdmi_color/hdmi4_rst_n
add wave -noupdate /hdmi_color_tb/hdmi_color/hcount
add wave -noupdate /hdmi_color_tb/hdmi_color/vcount
add wave -noupdate /hdmi_color_tb/hdmi_color/disp_data
add wave -noupdate /hdmi_color_tb/hdmi_color/video_hs
add wave -noupdate /hdmi_color_tb/hdmi_color/video_vs
add wave -noupdate /hdmi_color_tb/hdmi_color/video_de
add wave -noupdate /hdmi_color_tb/hdmi_color/video_r
add wave -noupdate /hdmi_color_tb/hdmi_color/video_g
add wave -noupdate /hdmi_color_tb/hdmi_color/video_b
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 96
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
WaveRestoreZoom {0 ps} {172627541134 ps}

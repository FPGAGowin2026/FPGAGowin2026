onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb/u_top/init_calib_complete
add wave -noupdate /tb/u_top/app_en
add wave -noupdate /tb/u_top/app_cmd
add wave -noupdate /tb/u_top/app_rdy
add wave -noupdate /tb/u_top/app_rd_data
add wave -noupdate /tb/u_top/app_rd_data_valid
add wave -noupdate /tb/u_top/app_wdf_rdy
add wave -noupdate /tb/u_top/app_wdf_data
add wave -noupdate /tb/u_top/app_wdf_wren
add wave -noupdate /tb/u_top/app_wdf_end
add wave -noupdate /tb/u_top/u_rd/c_s
add wave -noupdate /tb/u_top/u_rd/error
add wave -noupdate /tb/u_top/init_calib_complete
add wave -noupdate /tb/u_top/app_wdf_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 303
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
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {5099700400 ps}

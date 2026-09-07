onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /DDS_AD9767_tb/Clk
add wave -noupdate /DDS_AD9767_tb/Reset_n
add wave -noupdate /DDS_AD9767_tb/key_ch
add wave -noupdate /DDS_AD9767_tb/key_fre
add wave -noupdate /DDS_AD9767_tb/key_wave
add wave -noupdate /DDS_AD9767_tb/led
add wave -noupdate /DDS_AD9767_tb/sh_cp
add wave -noupdate /DDS_AD9767_tb/st_cp
add wave -noupdate /DDS_AD9767_tb/ds
add wave -noupdate /DDS_AD9767_tb/Clk0
add wave -noupdate /DDS_AD9767_tb/WRT0
add wave -noupdate /DDS_AD9767_tb/Data0
add wave -noupdate /DDS_AD9767_tb/Data1
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Mode_Sel_A
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Mode_Sel_B
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Fword_A
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Fword_B
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Pword_A
add wave -noupdate /DDS_AD9767_tb/dds_ad9767/Pword_B
add wave -noupdate /DDS_AD9767_tb/Clk1
add wave -noupdate /DDS_AD9767_tb/WRT1
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {49993525 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 262
configure wave -valuecolwidth 63
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
WaveRestoreZoom {49807496 ps} {50793750 ps}

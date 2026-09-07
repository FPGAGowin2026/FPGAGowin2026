onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tlv5618_tb/Clk
add wave -noupdate /tlv5618_tb/Rst_n
add wave -noupdate /tlv5618_tb/DAC_DATA
add wave -noupdate /tlv5618_tb/Start
add wave -noupdate /tlv5618_tb/Set_Done
add wave -noupdate /tlv5618_tb/DAC_CS_N
add wave -noupdate /tlv5618_tb/DAC_DIN
add wave -noupdate /tlv5618_tb/DAC_SCLK
add wave -noupdate /tlv5618_tb/tlv5618/r_DAC_DATA
add wave -noupdate /tlv5618_tb/tlv5618/DIV_CNT
add wave -noupdate /tlv5618_tb/tlv5618/SCLK2X
add wave -noupdate /tlv5618_tb/tlv5618/en
add wave -noupdate /tlv5618_tb/tlv5618/trans_done
add wave -noupdate -radix octal /tlv5618_tb/tlv5618/SCLK_GEN_CNT
add wave -noupdate /tlv5618_tb/tlv5618/fCLK
add wave -noupdate /tlv5618_tb/tlv5618/DIV_PARAM
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 2} {1800 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 125
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
configure wave -timelineunits ns
update
WaveRestoreZoom {317 ns} {1977 ns}

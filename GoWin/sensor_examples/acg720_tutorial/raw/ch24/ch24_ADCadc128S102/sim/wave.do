onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /adc128s022_tb/adc128s102/Clk
add wave -noupdate /adc128s022_tb/adc128s102/Rst_n
add wave -noupdate /adc128s022_tb/adc128s102/Channel
add wave -noupdate -format Analog-Step -height 100 -max 4095.0 -radix octal /adc128s022_tb/adc128s102/Data
add wave -noupdate /adc128s022_tb/adc128s102/En_Conv
add wave -noupdate /adc128s022_tb/adc128s102/Conv_Done
add wave -noupdate /adc128s022_tb/adc128s102/ADC_State
add wave -noupdate /adc128s022_tb/adc128s102/DIV_PARAM
add wave -noupdate /adc128s022_tb/adc128s102/ADC_SCLK
add wave -noupdate /adc128s022_tb/adc128s102/ADC_CS_N
add wave -noupdate /adc128s022_tb/adc128s102/ADC_DOUT
add wave -noupdate /adc128s022_tb/adc128s102/ADC_DIN
add wave -noupdate /adc128s022_tb/adc128s102/r_Channel
add wave -noupdate /adc128s022_tb/adc128s102/r_data
add wave -noupdate /adc128s022_tb/adc128s102/DIV_CNT
add wave -noupdate /adc128s022_tb/adc128s102/SCLK2X
add wave -noupdate /adc128s022_tb/adc128s102/SCLK_GEN_CNT
add wave -noupdate /adc128s022_tb/adc128s102/en
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {411380 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
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
WaveRestoreZoom {0 ns} {116888436 ns}

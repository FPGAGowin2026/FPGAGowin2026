onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /pll_led_tb/clk
add wave -noupdate /pll_led_tb/rst_n
add wave -noupdate /pll_led_tb/led
add wave -noupdate /pll_led_tb/clk_100m
add wave -noupdate /pll_led_tb/clk_100m_phase
add wave -noupdate /pll_led_tb/clk_200m
add wave -noupdate /pll_led_tb/clk_200m_duty
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 150
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
WaveRestoreZoom {0 ps} {73920 ns}

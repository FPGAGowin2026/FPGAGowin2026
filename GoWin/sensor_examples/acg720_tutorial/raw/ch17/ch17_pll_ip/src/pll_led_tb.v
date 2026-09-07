`timescale 1ns / 1ps
`define CLK_PERIOD 20

module pll_led_tb;
    reg clk;
    reg rst_n;
    wire [3:0]led;
    wire clk_100m;
    wire clk_100m_phase;
    wire clk_200m;
    wire clk_200m_duty;
    
    defparam pll_led.led0.CNT_MAX = 26'd500;
    defparam pll_led.led1.CNT_MAX = 26'd500;
    defparam pll_led.led2.CNT_MAX = 26'd500; 
    defparam pll_led.led3.CNT_MAX = 26'd500; 
           
    pll_led pll_led(
        .clk(clk),
        .rst_n(rst_n),
        .clk_100m(clk_100m),
        .clk_100m_phase(clk_100m_phase),
        .clk_200m(clk_200m),
        .clk_200m_duty(clk_200m_duty),
        .led(led)
    );
    
    initial clk = 1'b1;
    always #(`CLK_PERIOD/2) clk = ~clk;
    
    initial
        begin
        rst_n = 0;
        #201;
        rst_n = 1;
        #20000; 
        @(posedge pll_led.locked)
        #20000; 
        $stop;
    end
endmodule

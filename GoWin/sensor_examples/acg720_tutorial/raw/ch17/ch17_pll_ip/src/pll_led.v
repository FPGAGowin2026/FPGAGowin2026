module pll_led(
    clk,
    rst_n,
    clk_100m,
    clk_100m_phase,
    clk_200m,
    clk_200m_duty,
    led
);
    
    input clk;
    input rst_n;
    
    output clk_100m;
    output clk_100m_phase;
    output clk_200m;   
    output clk_200m_duty;
    output [3:0]led;
    
    wire locked;

    Gowin_PLL Gowin_PLL(
        .lock(locked), //output lock
        .clkout0(clk_100m), //output clkout0
        .clkout1(clk_100m_phase), //output clkout1
        .clkout2(clk_200m), //output clkout2
        .clkout3(clk_200m_duty), //output clkout3
        .clkin(clk), //input clkin
        .reset(~rst_n) //input reset
    );
    
    led_flip_ctrl #(
        .CNT_MAX(26'd50_000_000)
    )
    led0(
        .clk(clk_100m),
        .rst_n(locked),
        .led(led[0])
    );

    led_flip_ctrl #(
        .CNT_MAX(26'd50_000_000)
    )
    led1(
        .clk(clk_100m_phase),
        .rst_n(locked),
        .led(led[1])
    );

    led_flip_ctrl #(
        .CNT_MAX(26'd50_000_000)
    )
    led2(
        .clk(clk_200m),
        .rst_n(locked),
        .led(led[2])
    );

    led_flip_ctrl #(
        .CNT_MAX(26'd50_000_000)
    )
    led3(
        .clk(clk_200m_duty),
        .rst_n(locked),
        .led(led[3])
    );
    
endmodule

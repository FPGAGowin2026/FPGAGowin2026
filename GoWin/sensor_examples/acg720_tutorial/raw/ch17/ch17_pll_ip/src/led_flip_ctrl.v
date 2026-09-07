module led_flip_ctrl#(
    parameter CNT_MAX = 24'd10_000_000
)
(
    clk,
    rst_n,
    led
);
    
    input clk;
    input rst_n;
    output reg led;
    
    reg [27:0]cnt;
    
    always@(posedge clk or negedge rst_n)
    if(!rst_n)begin 
        cnt <= 'b0;
        led <= 'b0;
    end
    else if(cnt >= CNT_MAX - 1'b1)begin
        cnt <= 'b0;
        led <= ~led;
    end
    else
        cnt <= cnt + 1'b1;
    
endmodule

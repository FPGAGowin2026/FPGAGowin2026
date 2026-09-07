module bin_counter
(
    clk,
    reset_n,
    led
);
    input clk;
    input reset_n;
    output led;  
    reg led;

    parameter MCNT = 24_999_999;

    reg [24:0]cnt;  //定义计数器寄存器

    //计数器计数进程   
    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        cnt <= 25'd0;
    else if(cnt == MCNT)
        cnt <= 25'd0;
    else
        cnt <= cnt + 1'b1;

    //led输出控制进程
    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        led <= 1'b1;
    else if(cnt == MCNT)
        led <= ~led;
    else
        led <= led;


endmodule

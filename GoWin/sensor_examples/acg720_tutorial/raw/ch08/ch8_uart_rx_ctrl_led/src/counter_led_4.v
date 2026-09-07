`timescale 1ns/1ns

module counter_led_4(
    Clk,
    Reset_n,
    Ctrl,
    Time,
    Led
);
    input Clk;
    input Reset_n;
    input [7:0]Ctrl;
    input [31:0]Time;
    output reg Led;

    reg [31:0]counter;
    
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        counter <= #1 0;
    else if(counter >= Time - 1)
        counter <= #1 0;
    else
        counter <= #1 counter + 1'b1;
    
    reg [2:0]counter2;
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n) 
        counter2 <= #1 0; 
    else if(counter >= Time - 1)
        counter2 <= #1 counter2 + 1'b1;

    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        Led <= #1 0;
    else case(counter2)
        0:Led <= #1 Ctrl[0];
        1:Led <= #1 Ctrl[1];
        2:Led <= #1 Ctrl[2];
        3:Led <= #1 Ctrl[3];
        4:Led <= #1 Ctrl[4];
        5:Led <= #1 Ctrl[5];
        6:Led <= #1 Ctrl[6];
        7:Led <= #1 Ctrl[7];
        default:Led <= #1 Led;
    endcase
            
endmodule
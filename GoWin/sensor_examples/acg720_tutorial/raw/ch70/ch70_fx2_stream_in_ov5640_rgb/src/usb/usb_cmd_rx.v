`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/02/23 14:39:01
// Design Name: 
// Module Name: uart_cmd_rx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module usb_cmd_rx(
    clk,
    reset,
    cmdvalid,
    cmd_addr,
    cmd_data,
    start_sample
    );
    input clk;
    input reset;
    input [31:0]cmd_data;
    input cmdvalid;
    input [7:0]cmd_addr;
    output reg start_sample;  //Æô¶¯ 
    
      always@(posedge clk or posedge reset)
      if(reset)begin
        start_sample <= 1'b0;
      end
      else if(cmdvalid)begin
        case(cmd_addr)
            0: start_sample <= 1'b1;
          default:;
        endcase
      end
      else
        start_sample <= 1'b0;
endmodule

/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : uart_cmd
// Description   : 串口接收命令解析
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module usb_cmd(
    Clk,
    Reset_n,
    rx_data,
    rx_done,
    address,
    data,
    cmdvalid
);

    input Clk;
    input Reset_n;
    input [15:0]rx_data;
    input rx_done;
    output reg[7:0]address;
    output reg[31:0]data;
    output reg cmdvalid;

    reg [15:0] data_str [3:0];//data_str是4个数，每个数16位
    always@(posedge Clk)
    if(rx_done)begin
        data_str[3] <= #1 rx_data;
        data_str[2] <= #1 data_str[3];
        data_str[1] <= #1 data_str[2];
        data_str[0] <= #1 data_str[1];
//        data_str[3] <= #1 data_str[4];
//        data_str[2] <= #1 data_str[3];
//        data_str[1] <= #1 data_str[2];
//        data_str[0] <= #1 data_str[1];
    end

    reg r_rx_done;
    always@(posedge Clk)
        r_rx_done <= rx_done;

    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n) begin
        address <= #1 0;
        data <= #1 0;
        cmdvalid <= #1 0;
    end else if(r_rx_done)begin
        if((data_str[0][7:0] == 8'h55) && (data_str[0][15:8] == 8'hA5) && (data_str[3][15:8] == 8'hF0))begin
            data[7:0] <= #1 data_str[3][7:0];
            data[15:8] <= #1 data_str[2][15:8];
            data[23:16] <= #1 data_str[2][7:0];
            data[31:24] <= #1 data_str[1][15:8];
            address <= #1 data_str[1][7:0];
            cmdvalid <= #1 1;
        end
		else
			cmdvalid <= #1 0;//如果在r_rx_done为1的条件下，接收到错误指令，帧头或帧尾不符，则指令无效
    end
    else
        cmdvalid <= #1 0;

endmodule

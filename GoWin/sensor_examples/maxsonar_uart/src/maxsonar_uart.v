`timescale 1ns/1ps

// MaxBotix serial ASCII parser. Common devices emit 'R' followed by three or
// four decimal range digits and carriage return at 9600 8N1. The numeric unit
// is model-dependent (inches, centimetres, or millimetres), so the output name
// deliberately remains range_raw.
module maxsonar_uart #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(
    input wire clk,input wire rst_n,input wire uart_rx,
    output reg[15:0]range_raw,output reg range_valid,output reg format_error,output wire framing_error);
    wire[7:0]rx_byte;wire rx_valid;reg collecting_q;reg[2:0]digits_q;reg[15:0]value_q;
    uart_rx_8n1_maxsonar #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE))u_rx(.clk(clk),.rst_n(rst_n),.rx_i(uart_rx),.data_o(rx_byte),.valid_o(rx_valid),.framing_error_o(framing_error));
    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin collecting_q<=0;digits_q<=0;value_q<=0;range_raw<=0;range_valid<=0;format_error<=0;end
        else begin range_valid<=0;format_error<=0;if(framing_error)collecting_q<=0;
            if(rx_valid)begin
                if(rx_byte=="R")begin collecting_q<=1;digits_q<=0;value_q<=0;end
                else if(collecting_q && rx_byte>=8'h30&&rx_byte<=8'h39)begin
                    if(digits_q<4)begin
                        // 十进制乘十改写为同位宽移位；ASCII 数字低四位即 0..9。
                        value_q<=(value_q<<3)+(value_q<<1)+{12'd0,rx_byte[3:0]};
                        digits_q<=digits_q+1'b1;
                    end
                    else begin collecting_q<=0;format_error<=1;end
                end else if(collecting_q && rx_byte==8'h0D)begin
                    collecting_q<=0;if(digits_q>=1)begin range_raw<=value_q;range_valid<=1;end else format_error<=1;
                end else if(collecting_q)begin collecting_q<=0;format_error<=1;end
            end
        end
    end
endmodule

module uart_rx_8n1_maxsonar #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(input wire clk,input wire rst_n,input wire rx_i,output reg[7:0]data_o,output reg valid_o,output reg framing_error_o);
    localparam integer C0=CLK_FREQ_HZ/BAUD_RATE;localparam integer C=(C0<2)?2:C0;localparam integer H=C/2;localparam[1:0]I=0,S=1,D=2,P=3;reg m,s;reg[1:0]st;reg[31:0]n;reg[2:0]b;reg[7:0]sh;
    always@(posedge clk or negedge rst_n)if(!rst_n)begin m<=1;s<=1;st<=I;n<=0;b<=0;sh<=0;data_o<=0;valid_o<=0;framing_error_o<=0;end else begin m<=rx_i;s<=m;valid_o<=0;framing_error_o<=0;case(st)
        I:if(!s)begin st<=S;n<=0;end S:if(n==H-1)begin n<=0;if(!s)begin st<=D;b<=0;end else st<=I;end else n<=n+1'b1;
        D:if(n==C-1)begin n<=0;sh[b]<=s;if(b==7)st<=P;else b<=b+1'b1;end else n<=n+1'b1;
        P:if(n==C-1)begin n<=0;st<=I;if(s)begin data_o<=sh;valid_o<=1;end else framing_error_o<=1;end else n<=n+1'b1;default:st<=I;endcase end
endmodule

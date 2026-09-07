`timescale 1ns/1ps

// NMEA-0183 line validator. It accepts "$.....*HH" sentences, XORs every body
// byte between '$' and '*', and classifies GGA/RMC independently of talker ID.
// It intentionally exports validated metadata rather than floating-point
// latitude/longitude conversion, which is better done by a CPU or fixed-point block.
module gnss_nmea_uart #(
    parameter integer CLK_FREQ_HZ=27000000,
    parameter integer BAUD_RATE=9600,
    parameter integer MAX_BODY_CHARS=96
) (
    input wire clk,input wire rst_n,input wire uart_rx,
    output reg sentence_valid,output reg [1:0] sentence_type,
    output reg [7:0] body_length,output reg [7:0] checksum_value,
    output reg checksum_error,output reg format_error,output reg overflow_error,
    output wire framing_error
);
    wire[7:0]rx_byte;wire rx_valid;
    reg in_sentence_q,star_q,have_hi_q;
    reg[7:0]xor_q,length_q,checksum_hi_q;
    reg[2:0]header_index_q;
    reg[7:0]id2_q,id3_q,id4_q;
    function is_hex;input[7:0]c;begin is_hex=((c>="0"&&c<="9")||(c>="A"&&c<="F")||(c>="a"&&c<="f"));end endfunction
    // 调用点已经先用 is_hex() 验证。显式枚举既保持原语义，也避免综合器
    // 把 ASCII 算术扩展后再截断为 4 bit 的位宽告警。
    function[3:0]hex_value;input[7:0]c;begin case(c)
        8'h30:hex_value=4'h0;8'h31:hex_value=4'h1;8'h32:hex_value=4'h2;8'h33:hex_value=4'h3;
        8'h34:hex_value=4'h4;8'h35:hex_value=4'h5;8'h36:hex_value=4'h6;8'h37:hex_value=4'h7;
        8'h38:hex_value=4'h8;8'h39:hex_value=4'h9;8'h41,8'h61:hex_value=4'hA;
        8'h42,8'h62:hex_value=4'hB;8'h43,8'h63:hex_value=4'hC;8'h44,8'h64:hex_value=4'hD;
        8'h45,8'h65:hex_value=4'hE;default:hex_value=4'hF;
    endcase end endfunction
    uart_rx_8n1_gnss #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE))u_rx(.clk(clk),.rst_n(rst_n),.rx_i(uart_rx),.data_o(rx_byte),.valid_o(rx_valid),.framing_error_o(framing_error));
    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin in_sentence_q<=0;star_q<=0;have_hi_q<=0;xor_q<=0;length_q<=0;checksum_hi_q<=0;header_index_q<=0;id2_q<=0;id3_q<=0;id4_q<=0;sentence_valid<=0;sentence_type<=0;body_length<=0;checksum_value<=0;checksum_error<=0;format_error<=0;overflow_error<=0;end
        else begin
            sentence_valid<=0;checksum_error<=0;format_error<=0;overflow_error<=0;
            if(framing_error)in_sentence_q<=0;
            if(rx_valid)begin
                if(rx_byte==8'h24)begin in_sentence_q<=1;star_q<=0;have_hi_q<=0;xor_q<=0;length_q<=0;header_index_q<=0;id2_q<=0;id3_q<=0;id4_q<=0;end
                else if(in_sentence_q && !star_q)begin
                    if(rx_byte==8'h2A)begin star_q<=1;have_hi_q<=0;end
                    else if(rx_byte==8'h0D||rx_byte==8'h0A)begin in_sentence_q<=0;format_error<=1;end
                    else if(length_q>=MAX_BODY_CHARS)begin in_sentence_q<=0;overflow_error<=1;end
                    else begin
                        xor_q<=xor_q^rx_byte;length_q<=length_q+1'b1;
                        if(header_index_q==2)id2_q<=rx_byte;if(header_index_q==3)id3_q<=rx_byte;if(header_index_q==4)id4_q<=rx_byte;
                        if(header_index_q<5)header_index_q<=header_index_q+1'b1;
                    end
                end else if(in_sentence_q && star_q)begin
                    if(!is_hex(rx_byte))begin in_sentence_q<=0;format_error<=1;end
                    else if(!have_hi_q)begin checksum_hi_q<={hex_value(rx_byte),4'b0};have_hi_q<=1;end
                    else begin
                        in_sentence_q<=0;checksum_value<=xor_q;body_length<=length_q;
                        if((checksum_hi_q|hex_value(rx_byte))==xor_q)begin
                            sentence_valid<=1;
                            if(id2_q=="G"&&id3_q=="G"&&id4_q=="A")sentence_type<=1;
                            else if(id2_q=="R"&&id3_q=="M"&&id4_q=="C")sentence_type<=2;
                            else sentence_type<=0;
                        end else checksum_error<=1;
                    end
                end
            end
        end
    end
endmodule

module uart_rx_8n1_gnss #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(input wire clk,input wire rst_n,input wire rx_i,output reg[7:0]data_o,output reg valid_o,output reg framing_error_o);
    localparam integer C0=CLK_FREQ_HZ/BAUD_RATE;localparam integer C=(C0<2)?2:C0;localparam integer H=C/2;localparam[1:0]I=0,S=1,D=2,P=3;reg m,s;reg[1:0]st;reg[31:0]n;reg[2:0]b;reg[7:0]sh;
    always@(posedge clk or negedge rst_n)if(!rst_n)begin m<=1;s<=1;st<=I;n<=0;b<=0;sh<=0;data_o<=0;valid_o<=0;framing_error_o<=0;end else begin m<=rx_i;s<=m;valid_o<=0;framing_error_o<=0;case(st)
        I:if(!s)begin st<=S;n<=0;end S:if(n==H-1)begin n<=0;if(!s)begin st<=D;b<=0;end else st<=I;end else n<=n+1'b1;
        D:if(n==C-1)begin n<=0;sh[b]<=s;if(b==7)st<=P;else b<=b+1'b1;end else n<=n+1'b1;
        P:if(n==C-1)begin n<=0;st<=I;if(s)begin data_o<=sh;valid_o<=1;end else framing_error_o<=1;end else n<=n+1'b1;default:st<=I;endcase end
endmodule

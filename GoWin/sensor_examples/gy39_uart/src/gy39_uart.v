`timescale 1ns/1ps

// GY-39 v3 auto-output parser. Frame = 5A 5A TYPE LEN PAYLOAD CHECKSUM,
// checksum is the low eight bits of all preceding bytes. TYPE 15h carries
// four-byte big-endian illuminance (0.01 lux); TYPE 45h carries the BME frame.
module gy39_uart #(
    parameter integer CLK_FREQ_HZ=27000000,
    parameter integer BAUD_RATE=9600
) (
    input wire clk, input wire rst_n, input wire uart_rx,
    output reg [31:0] lux_centi_lux,
    output reg signed [15:0] temperature_centi_c,
    output reg [31:0] pressure_centi_pa,
    output reg [15:0] humidity_centi_rh,
    output reg signed [15:0] altitude_m,
    output reg lux_valid, output reg environment_valid,
    output reg checksum_error, output reg format_error,
    output wire framing_error
);
    wire [7:0] rx_byte; wire rx_valid;
    reg [2:0] state_q;
    localparam H0=0,H1=1,TYPE=2,LENGTH=3,PAYLOAD=4,CHECKSUM=5;
    reg [7:0] type_q, length_q, index_q, sum_q;
    reg [7:0] payload_q [0:9];
    uart_rx_8n1_gy39 #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_rx(
        .clk(clk),.rst_n(rst_n),.rx_i(uart_rx),.data_o(rx_byte),.valid_o(rx_valid),.framing_error_o(framing_error));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state_q<=H0; type_q<=0; length_q<=0; index_q<=0; sum_q<=0;
            lux_centi_lux<=0; temperature_centi_c<=0; pressure_centi_pa<=0;
            humidity_centi_rh<=0; altitude_m<=0; lux_valid<=0; environment_valid<=0;
            checksum_error<=0; format_error<=0;
        end else begin
            lux_valid<=0; environment_valid<=0; checksum_error<=0; format_error<=0;
            if(framing_error) state_q<=H0;
            if(rx_valid) case(state_q)
                H0: if(rx_byte==8'h5A) begin state_q<=H1; sum_q<=8'h5A; end
                H1: if(rx_byte==8'h5A) begin state_q<=TYPE; sum_q<=sum_q+rx_byte; end else state_q<=H0;
                TYPE: begin type_q<=rx_byte; sum_q<=sum_q+rx_byte; state_q<=LENGTH; end
                LENGTH: begin
                    length_q<=rx_byte; sum_q<=sum_q+rx_byte; index_q<=0;
                    if(rx_byte==0 || rx_byte>10) begin state_q<=H0; format_error<=1; end
                    else state_q<=PAYLOAD;
                end
                PAYLOAD: begin
                    payload_q[index_q]<=rx_byte; sum_q<=sum_q+rx_byte;
                    if(index_q+1'b1==length_q) state_q<=CHECKSUM;
                    else index_q<=index_q+1'b1;
                end
                CHECKSUM: begin
                    state_q<=H0;
                    if(rx_byte!=sum_q) checksum_error<=1;
                    else if(type_q==8'h15 && length_q==4) begin
                        lux_centi_lux<={payload_q[0],payload_q[1],payload_q[2],payload_q[3]}; lux_valid<=1;
                    end else if(type_q==8'h45 && length_q==10) begin
                        temperature_centi_c<={payload_q[0],payload_q[1]};
                        pressure_centi_pa<={payload_q[2],payload_q[3],payload_q[4],payload_q[5]};
                        humidity_centi_rh<={payload_q[6],payload_q[7]};
                        altitude_m<={payload_q[8],payload_q[9]}; environment_valid<=1;
                    end else format_error<=1;
                end
                default: state_q<=H0;
            endcase
        end
    end
endmodule

module uart_rx_8n1_gy39 #(
    parameter integer CLK_FREQ_HZ=27000000, parameter integer BAUD_RATE=9600
) (input wire clk,input wire rst_n,input wire rx_i,output reg [7:0] data_o,output reg valid_o,output reg framing_error_o);
    localparam integer C0=CLK_FREQ_HZ/BAUD_RATE; localparam integer C=(C0<2)?2:C0; localparam integer H=C/2;
    localparam [1:0] I=0,S=1,D=2,P=3; reg m,s; reg [1:0] st; reg [31:0] n; reg [2:0] b; reg [7:0] sh;
    always @(posedge clk or negedge rst_n) if(!rst_n) begin m<=1;s<=1;st<=I;n<=0;b<=0;sh<=0;data_o<=0;valid_o<=0;framing_error_o<=0; end else begin
        m<=rx_i;s<=m;valid_o<=0;framing_error_o<=0;
        case(st)
            I:if(!s)begin st<=S;n<=0;end
            S:if(n==H-1)begin n<=0;if(!s)begin st<=D;b<=0;end else st<=I;end else n<=n+1'b1;
            D:if(n==C-1)begin n<=0;sh[b]<=s;if(b==7)st<=P;else b<=b+1'b1;end else n<=n+1'b1;
            P:if(n==C-1)begin n<=0;st<=I;if(s)begin data_o<=sh;valid_o<=1;end else framing_error_o<=1;end else n<=n+1'b1;
            default:st<=I;
        endcase
    end
endmodule

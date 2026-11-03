`timescale 1ns/1ps
`default_nettype none
// 两条已字节化的遥测流在公共包时钟域逐包轮询；整包锁定，start不消耗数据。
module bx72_telemetry_mux(
    input wire clk,reset_i,
    input wire source0_valid_i,source0_length_nonzero_i,
    input wire [7:0] source0_length_i,source0_data_i,
    output wire source0_start_o,source0_read_o,
    input wire source1_valid_i,source1_length_nonzero_i,
    input wire [7:0] source1_length_i,source1_data_i,
    output wire source1_start_o,source1_read_o,
    output reg tx_valid_o,tx_length_nonzero_o,
    output reg [7:0] tx_length_o,
    output wire [7:0] tx_data_o,
    input wire tx_start_i,tx_read_i
);
    localparam [1:0] IDLE=0,OFFER=1,ACTIVE=2;
    reg [1:0] state;
    reg selected,prefer;
    reg [7:0] remaining;
    wire eligible0=source0_valid_i&&source0_length_nonzero_i;
    wire eligible1=source1_valid_i&&source1_length_nonzero_i;
    wire choose1=eligible1&&(!eligible0||prefer);
    wire start_fire=tx_valid_o&&tx_start_i&&!reset_i;
    wire read_fire=tx_read_i&&!reset_i&&((state==ACTIVE)||start_fire);
    assign tx_data_o=selected?source1_data_i:source0_data_i;
    assign source0_start_o=start_fire&&!selected;
    assign source1_start_o=start_fire&&selected;
    assign source0_read_o=read_fire&&!selected;
    assign source1_read_o=read_fire&&selected;
    always @(posedge clk) begin
        if(reset_i)begin
            state<=IDLE;selected<=0;prefer<=0;remaining<=0;
            tx_valid_o<=0;tx_length_o<=0;tx_length_nonzero_o<=0;
        end else case(state)
            IDLE:if(eligible0||eligible1)begin
                selected<=choose1;
                tx_length_o<=choose1?source1_length_i:source0_length_i;
                remaining<=choose1?source1_length_i:source0_length_i;
                tx_length_nonzero_o<=1;
                tx_valid_o<=1;
                state<=OFFER;
            end
            OFFER:if(start_fire)begin
                tx_valid_o<=0;
                if(tx_read_i)begin
                    remaining<=remaining-1'b1;
                    if(remaining==1)begin state<=IDLE;prefer<=!selected;end
                    else state<=ACTIVE;
                end else state<=ACTIVE;
            end
            ACTIVE:if(read_fire)begin
                remaining<=remaining-1'b1;
                if(remaining==1)begin state<=IDLE;prefer<=!selected;end
            end
            default:begin state<=IDLE;tx_valid_o<=0;tx_length_nonzero_o<=0;end
        endcase
    end
endmodule
`default_nettype wire

`timescale 1ns/1ps

// Eight independent digital IR/proximity channels. Inputs are synchronized and
// must hold the new state for DEBOUNCE_CYCLES clocks before publication.
module multi_ir_array #(
    parameter integer CHANNELS=8,
    parameter integer DEBOUNCE_CYCLES=27000,
    parameter ACTIVE_LOW=1'b1
) (
    input wire clk,input wire rst_n,input wire [CHANNELS-1:0] sensor_i,
    output reg [CHANNELS-1:0] active_mask_o,output reg change_valid_o
);
    localparam integer DB=(DEBOUNCE_CYCLES<1)?1:DEBOUNCE_CYCLES;
    reg [CHANNELS-1:0] sync1_q,sync2_q;
    reg [31:0] count_q [0:CHANNELS-1];
    integer k;
    wire [CHANNELS-1:0] sensed_active=ACTIVE_LOW ? ~sync2_q : sync2_q;
    always @(posedge clk or negedge rst_n) begin
        // 复位时将同步器初值设为对应极性的“未触发”原始电平。
        // 这样 ACTIVE_LOW=0 且 DB=1 时也不会在上电后伪造一次障碍事件。
        if(!rst_n) begin sync1_q<={CHANNELS{ACTIVE_LOW}};sync2_q<={CHANNELS{ACTIVE_LOW}};active_mask_o<=0;change_valid_o<=0;for(k=0;k<CHANNELS;k=k+1)count_q[k]<=0;end
        else begin
            sync1_q<=sensor_i;sync2_q<=sync1_q;change_valid_o<=0;
            for(k=0;k<CHANNELS;k=k+1) begin
                if(sensed_active[k]==active_mask_o[k]) count_q[k]<=0;
                else if(count_q[k]>=DB-1) begin active_mask_o[k]<=sensed_active[k];count_q[k]<=0;change_valid_o<=1;end
                else count_q[k]<=count_q[k]+1'b1;
            end
        end
    end
endmodule

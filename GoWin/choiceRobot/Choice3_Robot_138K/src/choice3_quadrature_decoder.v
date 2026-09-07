`timescale 1ns/1ps

// 带首次状态采集的四倍频正交解码器，避免复位时 AB 非 00 产生假步进。
module choice3_quadrature_decoder (
    input  wire clk_i, input wire rst_n_i,
    input  wire encoder_a_i, input wire encoder_b_i,
    output reg signed [31:0] position_ticks_o,
    output reg step_valid_o, output reg invalid_transition_o,
    output reg baseline_ready_o
);
    (* ASYNC_REG="TRUE" *) reg [1:0] sync_a_q, sync_b_q;
    reg [1:0] previous_ab_q;
    reg [1:0] startup_q;
    wire [1:0] current_ab = {sync_a_q[1], sync_b_q[1]};
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            sync_a_q<=0; sync_b_q<=0; previous_ab_q<=0;startup_q<=0;
            position_ticks_o<=0; step_valid_o<=0;
            invalid_transition_o<=0; baseline_ready_o<=0;
        end else begin
            sync_a_q <= {sync_a_q[0],encoder_a_i};
            sync_b_q <= {sync_b_q[0],encoder_b_i};
            step_valid_o<=0; invalid_transition_o<=0;
            if (!baseline_ready_o) begin
                // 等待两级同步器和额外一拍基线均稳定，避免任意上电相位产生假步进。
                previous_ab_q <= current_ab;
                startup_q <= {startup_q[0],1'b1};
                if(&startup_q)baseline_ready_o <= 1'b1;
            end else if (current_ab != previous_ab_q) begin
                case ({previous_ab_q,current_ab})
                    4'b0001,4'b0111,4'b1110,4'b1000: begin
                        position_ticks_o<=position_ticks_o+1; step_valid_o<=1;
                    end
                    4'b0010,4'b1011,4'b1101,4'b0100: begin
                        position_ticks_o<=position_ticks_o-1; step_valid_o<=1;
                    end
                    default: invalid_transition_o<=1;
                endcase
                previous_ab_q<=current_ab;
            end
        end
    end
endmodule


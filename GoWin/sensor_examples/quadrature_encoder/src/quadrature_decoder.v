`timescale 1ns/1ps

// AB 相两拍同步与四倍频正交解码。
module quadrature_decoder (
    input  wire              clk_i,
    input  wire              rst_n_i,
    input  wire              encoder_a_i,
    input  wire              encoder_b_i,
    output reg signed [31:0] position_ticks_o,
    output reg               step_valid_o,
    output reg               direction_o,
    output reg               invalid_transition_o
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_a_q;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_b_q;
    reg [1:0] previous_ab_q;
    reg [1:0] startup_q;
    reg       decoder_ready_q;
    wire [1:0] current_ab = {sync_a_q[1], sync_b_q[1]};

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            sync_a_q <= 2'b00;
            sync_b_q <= 2'b00;
            previous_ab_q <= 2'b00;
            startup_q <= 2'b00;
            decoder_ready_q <= 1'b0;
            position_ticks_o <= 32'sd0;
            step_valid_o <= 1'b0;
            direction_o <= 1'b0;
            invalid_transition_o <= 1'b0;
        end else begin
            sync_a_q <= {sync_a_q[0], encoder_a_i};
            sync_b_q <= {sync_b_q[0], encoder_b_i};
            step_valid_o <= 1'b0;
            invalid_transition_o <= 1'b0;
            if (!decoder_ready_q) begin
                // 编码器上电可能停在任意相位；同步器稳定后先建立基线，
                // 避免把 00 到实际初相位误计为一次运动。
                previous_ab_q <= current_ab;
                startup_q <= {startup_q[0], 1'b1};
                if (&startup_q)
                    decoder_ready_q <= 1'b1;
            end else if (current_ab != previous_ab_q) begin
                case ({previous_ab_q, current_ab})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                        position_ticks_o <= position_ticks_o + 32'sd1;
                        step_valid_o <= 1'b1;
                        direction_o <= 1'b1;
                    end
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                        position_ticks_o <= position_ticks_o - 32'sd1;
                        step_valid_o <= 1'b1;
                        direction_o <= 1'b0;
                    end
                    default: invalid_transition_o <= 1'b1;
                endcase
                previous_ab_q <= current_ab;
            end
        end
    end
endmodule

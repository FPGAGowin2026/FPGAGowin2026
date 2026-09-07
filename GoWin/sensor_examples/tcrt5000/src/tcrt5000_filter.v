`timescale 1ns/1ps

// Digital TCRT5000 line-sensor conditioner.
// A saturating up/down counter rejects short glitches and provides hysteresis:
// the output asserts only after FILTER_CYCLES active samples and clears only
// after the counter has returned to zero.
module tcrt5000_filter #(
    parameter integer ACTIVE_LOW    = 1,
    parameter integer FILTER_CYCLES = 16
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sensor_in,
    output reg         line_detected,
    output reg         line_rise,
    output reg         line_fall,
    output reg  [31:0] filter_level
);

    localparam integer FILTER_LIMIT =
        (FILTER_CYCLES < 1) ? 1 : FILTER_CYCLES;

    (* ASYNC_REG = "TRUE" *) reg sensor_meta;
    (* ASYNC_REG = "TRUE" *) reg sensor_sync;
    wire active_sample;

    assign active_sample = (ACTIVE_LOW != 0) ? ~sensor_sync : sensor_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_meta <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
            sensor_sync <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
        end else begin
            sensor_meta <= sensor_in;
            sensor_sync <= sensor_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_detected <= 1'b0;
            line_rise     <= 1'b0;
            line_fall     <= 1'b0;
            filter_level  <= 32'd0;
        end else begin
            line_rise <= 1'b0;
            line_fall <= 1'b0;

            if (active_sample) begin
                if (filter_level < FILTER_LIMIT)
                    filter_level <= filter_level + 1'b1;

                if (!line_detected &&
                    (filter_level >= (FILTER_LIMIT - 1))) begin
                    line_detected <= 1'b1;
                    line_rise     <= 1'b1;
                end
            end else begin
                if (filter_level != 0)
                    filter_level <= filter_level - 1'b1;

                if (line_detected && (filter_level <= 1)) begin
                    line_detected <= 1'b0;
                    line_fall     <= 1'b1;
                end
            end
        end
    end

endmodule

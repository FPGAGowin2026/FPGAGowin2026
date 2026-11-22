`timescale 1ns/1ps

// Keep the closest actual sample in each 2-degree interval. The independent
// obstacle guard still consumes every point before this reduction.
module lidar_bin_reduce (
    input wire clk, rst, sample_i, hit_i, error_i,
    input wire [15:0] angle_i, distance_i,
    input wire [7:0] strength_i,
    input wire [31:0] stamp_i,
    output reg valid_o,
    output reg [72:0] record_o
);
    reg active, best_hit;
    reg [7:0] bin_q;
    reg [15:0] best_angle, best_distance;
    reg [7:0] best_strength;
    reg [31:0] best_stamp;
    // Exact floor(angle/200) over the input range 0..35999 centidegrees.
    wire [34:0] scaled_angle = angle_i * 19'd335545;
    wire [7:0] incoming_bin = scaled_angle[33:26];
    wire incoming_hit = hit_i && distance_i >= 16'd50 && distance_i <= 16'd8000;
    always @(posedge clk) begin
        if (rst) begin
            active <= 0; best_hit <= 0; bin_q <= 0;
            best_angle <= 0; best_distance <= 0; best_strength <= 0;
            best_stamp <= 0; valid_o <= 0; record_o <= 0;
        end else begin
            valid_o <= 0;
            if (error_i) begin
                active <= 0;
                best_hit <= 0;
            end else if (sample_i) begin
                if (!active || incoming_bin != bin_q) begin
                    if (active) begin
                        record_o <= {best_stamp,best_hit,best_strength,best_angle,best_distance};
                        valid_o <= 1;
                    end
                    active <= 1;
                    bin_q <= incoming_bin;
                    best_hit <= incoming_hit;
                    best_angle <= angle_i;
                    best_distance <= incoming_hit ? distance_i : 16'd0;
                    best_strength <= strength_i;
                    best_stamp <= stamp_i;
                end else if (incoming_hit && (!best_hit || distance_i < best_distance)) begin
                    best_hit <= 1;
                    best_angle <= angle_i;
                    best_distance <= distance_i;
                    best_strength <= strength_i;
                    best_stamp <= stamp_i;
                end
            end
        end
    end
endmodule

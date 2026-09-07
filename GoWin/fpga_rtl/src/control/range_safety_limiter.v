`timescale 1ns/1ps

// 根据前向测距对运动目标做最后一级安全限制。
// 前方危险时只禁止正向直行，原地旋转和后退仍可用于脱困。
module range_safety_limiter #(
    parameter COMMAND_WIDTH = 32,
    parameter RANGE_WIDTH = 16,
    parameter [RANGE_WIDTH-1:0] STOP_DISTANCE = 16'd200,
    parameter [RANGE_WIDTH-1:0] SLOW_DISTANCE = 16'd500,
    parameter SLOW_SHIFT = 1
) (
    input  wire                            command_valid_i,
    input  wire signed [COMMAND_WIDTH-1:0] linear_i,
    input  wire signed [COMMAND_WIDTH-1:0] angular_i,
    input  wire                            front_valid_i,
    input  wire        [RANGE_WIDTH-1:0]   front_range_i,

    output reg                             command_valid_o,
    output reg signed [COMMAND_WIDTH-1:0]  linear_o,
    output reg signed [COMMAND_WIDTH-1:0]  angular_o,
    // 0：正常，1：减速，2：前方停车。
    output reg         [1:0]               safety_level_o,
    output wire                            forward_blocked_o
);

    localparam [1:0] LEVEL_NORMAL = 2'd0;
    localparam [1:0] LEVEL_SLOW   = 2'd1;
    localparam [1:0] LEVEL_STOP   = 2'd2;

    assign forward_blocked_o = !front_valid_i || front_range_i <= STOP_DISTANCE;

    always @(*) begin
        command_valid_o = command_valid_i;
        linear_o = {COMMAND_WIDTH{1'b0}};
        angular_o = {COMMAND_WIDTH{1'b0}};

        if (!front_valid_i || front_range_i <= STOP_DISTANCE)
            safety_level_o = LEVEL_STOP;
        else if (front_range_i <= SLOW_DISTANCE)
            safety_level_o = LEVEL_SLOW;
        else
            safety_level_o = LEVEL_NORMAL;

        if (command_valid_i) begin
            angular_o = angular_i;

            if (linear_i > 0) begin
                if (safety_level_o == LEVEL_STOP)
                    linear_o = {COMMAND_WIDTH{1'b0}};
                else if (safety_level_o == LEVEL_SLOW)
                    linear_o = linear_i >>> SLOW_SHIFT;
                else
                    linear_o = linear_i;
            end else begin
                linear_o = linear_i;
            end
        end
    end

endmodule

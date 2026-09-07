`timescale 1ns/1ps

// 两块 TB6612FNG 驱动四电机。四路 PWM/方向独立，共用 STBY。
// 方向翻转必须经历 DEADTIME_CYCLES 个完整系统时钟的全关断区间。
module choice3_tb6612_quad_bridge #(
    parameter integer PWM_PERIOD = 1000,
    parameter integer DEADTIME_CYCLES = 5000
) (
    input  wire        clk_i,
    input  wire        rst_n_i,
    input  wire        enable_i,
    input  wire [63:0] duty_flat_i,
    input  wire [3:0]  direction_i,
    output wire [3:0]  pwm_o,
    output wire [3:0]  in1_o,
    output wire [3:0]  in2_o,
    output wire        stby_o,
    output wire [3:0]  deadtime_active_o
);
    localparam integer PWM_W = (PWM_PERIOD < 2) ? 1 : $clog2(PWM_PERIOD);
    localparam integer DEAD_W = (DEADTIME_CYCLES < 2) ? 1 : $clog2(DEADTIME_CYCLES+1);
    reg [PWM_W-1:0] pwm_count_q;
    reg [3:0] direction_q;
    reg [DEAD_W-1:0] dead_count_q [0:3];
    wire [15:0] duty [0:3];
    wire [3:0] drive_on;
    genvar g;
    integer i;

    assign duty[0]=duty_flat_i[15:0];
    assign duty[1]=duty_flat_i[31:16];
    assign duty[2]=duty_flat_i[47:32];
    assign duty[3]=duty_flat_i[63:48];
    assign stby_o = enable_i;

    generate for (g=0;g<4;g=g+1) begin: g_drive
        assign deadtime_active_o[g] = dead_count_q[g] != 0;
        assign drive_on[g] = enable_i && (dead_count_q[g] == 0) &&
                             (direction_i[g] == direction_q[g]) &&
                             (duty[g] != 0) && (pwm_count_q < duty[g]);
        assign pwm_o[g] = drive_on[g];
        assign in1_o[g] = drive_on[g] && direction_q[g];
        assign in2_o[g] = drive_on[g] && !direction_q[g];
    end endgenerate

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            pwm_count_q <= 0;
            direction_q <= 4'b1111;
            for (i=0;i<4;i=i+1) dead_count_q[i] <= 0;
        end else begin
            if (pwm_count_q >= PWM_PERIOD-1) pwm_count_q <= 0;
            else pwm_count_q <= pwm_count_q + 1'b1;
            for (i=0;i<4;i=i+1) begin
                if (!enable_i || duty[i] == 0) begin
                    dead_count_q[i] <= 0;
                    direction_q[i] <= direction_i[i];
                end else if (dead_count_q[i] != 0) begin
                    if (dead_count_q[i] == 1) begin
                        dead_count_q[i] <= 0;
                        direction_q[i] <= direction_i[i];
                    end else begin
                        dead_count_q[i] <= dead_count_q[i] - 1'b1;
                    end
                end else if (direction_i[i] != direction_q[i]) begin
                    dead_count_q[i] <= DEADTIME_CYCLES;
                end
            end
        end
    end
endmodule

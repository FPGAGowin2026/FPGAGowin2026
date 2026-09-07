`timescale 1ns/1ps

// 编码器在线/堵转监视。静止时数字 AB 线无法证明机械连接存在，只有在
// 已下发运动命令后，连续多次采样未观察到边沿才判定堵转/缺失。
module choice3_encoder_health #(
    parameter integer STALL_SAMPLE_LIMIT = 100,
    parameter integer MIN_DUTY_FOR_CHECK = 80
) (
    input  wire       clk_i,
    input  wire       rst_n_i,
    input  wire       clear_fault_pulse_i,
    input  wire       motion_enabled_i,
    input  wire       sample_tick_i,
    input  wire [3:0] baseline_ready_i,
    input  wire [3:0] encoder_step_i,
    input  wire [63:0] pwm_duty_flat_i,
    output wire       health_ok_o,
    output reg  [3:0] stall_fault_o
);
    localparam integer COUNT_W = (STALL_SAMPLE_LIMIT < 2) ? 1 : $clog2(STALL_SAMPLE_LIMIT+1);
    reg [3:0] step_seen_q;
    reg [COUNT_W-1:0] stall_count_q [0:3];
    integer i;
    wire [15:0] duty0 = pwm_duty_flat_i[15:0];
    wire [15:0] duty1 = pwm_duty_flat_i[31:16];
    wire [15:0] duty2 = pwm_duty_flat_i[47:32];
    wire [15:0] duty3 = pwm_duty_flat_i[63:48];

    function duty_active;
        input integer index;
        begin
            case(index)
                0: duty_active = duty0 >= MIN_DUTY_FOR_CHECK;
                1: duty_active = duty1 >= MIN_DUTY_FOR_CHECK;
                2: duty_active = duty2 >= MIN_DUTY_FOR_CHECK;
                default: duty_active = duty3 >= MIN_DUTY_FOR_CHECK;
            endcase
        end
    endfunction

    assign health_ok_o = (&baseline_ready_i) && (stall_fault_o == 4'b0000);

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            step_seen_q <= 0;
            stall_fault_o <= 0;
            for (i=0;i<4;i=i+1) stall_count_q[i] <= 0;
        end else begin
            step_seen_q <= step_seen_q | encoder_step_i;
            if (clear_fault_pulse_i) begin
                stall_fault_o <= 0;
                for (i=0;i<4;i=i+1) stall_count_q[i] <= 0;
            end
            if (sample_tick_i) begin
                for (i=0;i<4;i=i+1) begin
                    if (!motion_enabled_i || !duty_active(i) || step_seen_q[i]) begin
                        stall_count_q[i] <= 0;
                    end else if (!stall_fault_o[i]) begin
                        if (stall_count_q[i] >= STALL_SAMPLE_LIMIT-1) begin
                            stall_fault_o[i] <= 1'b1;
                        end else begin
                            stall_count_q[i] <= stall_count_q[i] + 1'b1;
                        end
                    end
                end
                step_seen_q <= 0;
            end
        end
    end
endmodule

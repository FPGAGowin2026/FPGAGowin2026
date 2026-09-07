`timescale 1ns/1ps

// 在 FPGA 独立控制与 Jetson 协同控制之间执行唯一选源和安全门控。
// 模式编码与 UDP Status 报文一致：0 表示 FPGA 独立模式，1 表示 Jetson 协同模式。
// 本模块只负责仲裁，不负责输入同步、停止状态判定、速度限幅或物理急停断电。
module motion_mode_arbiter #(
    parameter COMMAND_WIDTH = 32
) (
    // 所有输入除异步低有效复位外，接入前都应同步到 clk_i 时钟域。
    input  wire                            clk_i,
    input  wire                            rst_n_i,

    // requested_mode_i 是模式请求；motion_stopped_i 是外部确认后的实际停车条件。
    input  wire                            requested_mode_i,
    input  wire                            motion_stopped_i,

    // 任一安全条件有效时立即禁止运动，不受当前模式或切换状态影响。
    input  wire                            estop_i,
    input  wire                            critical_fault_i,
    input  wire                            safety_stop_i,

    // FPGA 独立路径控制器给出的运动目标及有效标志。
    input  wire                            fpga_command_valid_i,
    input  wire signed [COMMAND_WIDTH-1:0] fpga_linear_i,
    input  wire signed [COMMAND_WIDTH-1:0] fpga_angular_i,

    // Jetson 网关给出的运动目标及心跳/报文有效标志。
    input  wire                            jetson_command_valid_i,
    input  wire signed [COMMAND_WIDTH-1:0] jetson_linear_i,
    input  wire signed [COMMAND_WIDTH-1:0] jetson_angular_i,

    // active_mode_o 表示实际生效模式；请求模式在安全切换完成前不会直接生效。
    output reg                             active_mode_o,
    output wire                            switch_pending_o,
    output wire                            motion_enabled_o,
    output reg signed [COMMAND_WIDTH-1:0]  linear_command_o,
    output reg signed [COMMAND_WIDTH-1:0]  angular_command_o
);

    localparam MODE_FPGA   = 1'b0;
    localparam MODE_JETSON = 1'b1;

    // 活动模式更新后继续封锁一拍，避免新旧来源在切换边沿直接相接。
    reg switch_guard_q;

    wire mode_change_requested;
    wire safety_blocked;
    wire selected_command_valid;

    assign mode_change_requested = requested_mode_i != active_mode_o;

    // 模式变化时先强制零目标；确认电机停止后切换，并再保持一拍零输出。
    assign switch_pending_o = mode_change_requested || switch_guard_q;

    // 这些保护条件不受当前模式影响。物理急停仍应直接切断驱动使能。
    assign safety_blocked = !rst_n_i || estop_i || critical_fault_i || safety_stop_i;

    assign selected_command_valid =
        (active_mode_o == MODE_FPGA) ? fpga_command_valid_i : jetson_command_valid_i;

    // 只有安全条件、模式切换和来源有效性三道门槛全部通过才允许运动。
    assign motion_enabled_o =
        !safety_blocked && !switch_pending_o && selected_command_valid;

    // 运动中出现模式请求时保持原活动模式，并靠零目标促使底盘停车。
    // 外部确认停车后才登记新模式，随后 switch_guard_q 再保护一个完整周期。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            active_mode_o <= MODE_FPGA;
            switch_guard_q <= 1'b0;
        end else begin
            switch_guard_q <= 1'b0;

            if (mode_change_requested && motion_stopped_i) begin
                active_mode_o <= requested_mode_i;
                switch_guard_q <= 1'b1;
            end
        end
    end

    // 默认输出零；只有 motion_enabled_o 有效时才转发当前模式对应的唯一来源。
    always @(*) begin
        linear_command_o  = {COMMAND_WIDTH{1'b0}};
        angular_command_o = {COMMAND_WIDTH{1'b0}};

        if (motion_enabled_o) begin
            if (active_mode_o == MODE_JETSON) begin
                linear_command_o  = jetson_linear_i;
                angular_command_o = jetson_angular_i;
            end else begin
                linear_command_o  = fpga_linear_i;
                angular_command_o = fpga_angular_i;
            end
        end
    end

endmodule

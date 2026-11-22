module mecanum_motor_closed_loop_core #(
    parameter integer ENABLE_DEBUG_UART = 1,
    // 仅供已在同一时钟域完成范围、时效与近障保护的板内调用。
    parameter integer TRUSTED_GUARDED_INPUT = 0,
    parameter integer CLK_HZ = motor_closed_loop_config::CLK_HZ,
    parameter integer BAUD = motor_closed_loop_config::BAUD,
    parameter integer SAMPLE_HZ = motor_closed_loop_config::SAMPLE_HZ,
    parameter integer PWM_PERIOD = motor_closed_loop_config::PWM_PERIOD,
    parameter integer JOG_CYCLES = CLK_HZ / 1000 * 300,
    parameter integer REMOTE_TIMEOUT_CYCLES = CLK_HZ / 1000 * 300,
    parameter integer REPORT_CYCLES = CLK_HZ / 10,
    parameter integer STARTUP_CYCLES = CLK_HZ / 100,
    parameter integer CALIBRATION_CYCLES = CLK_HZ,
    parameter signed [31:0] MM_PER_TICK_Q16 = motor_closed_loop_config::MM_PER_TICK_Q16,
    parameter [7:0] MOTOR_CHANNEL_MAP = motor_closed_loop_config::MOTOR_CHANNEL_MAP,
    parameter [7:0] ENCODER_CHANNEL_MAP = motor_closed_loop_config::ENCODER_CHANNEL_MAP,
    parameter [3:0] MOTOR_REVERSE = motor_closed_loop_config::MOTOR_REVERSE,
    parameter [3:0] ENCODER_REVERSE = motor_closed_loop_config::ENCODER_REVERSE
)(
    input wire clk_50m_i, user_reset_i, debug_uart_rx_i,
    input wire [3:0] encoder_a_i, encoder_b_i,
    input wire remote_valid_i, remote_link_down_i,
    input wire [47:0] remote_velocity_i,
    output wire debug_uart_tx_o,
    output wire [3:0] motor_in1_o, motor_in2_o,
    // 只导出已有测速，不改变速度环或电机串口协议。
    output wire odometry_valid_o,
    output wire signed [31:0] odometry_vx_o,odometry_vy_o,odometry_wz_o,
    output wire [127:0] odometry_ticks_o,
    output wire [127:0] odometry_wheels_o,
    output wire [3:0] odometry_invalid_o,
    output reg telemetry_valid_o,
    output reg [703:0] telemetry_payload_o
);
    reg [31:0] startup = 0;
    wire por = startup < STARTUP_CYCLES;
    always @(posedge clk_50m_i) if (por) startup <= startup + 1;
    (* ASYNC_REG="TRUE" *) reg [1:0] reset_sync = 2'b11;
    always @(posedge clk_50m_i or posedge user_reset_i)
        if (user_reset_i) reset_sync <= 2'b11;
        else reset_sync <= {reset_sync[0], por};
    wire rst = reset_sync[1];
    wire [7:0] rx_data;
    wire rx_valid;
    reg physical_report_mode;
    generate if(ENABLE_DEBUG_UART)begin : gen_debug_rx
    (* ASYNC_REG="TRUE" *) reg [1:0] rx_sync = 2'b11;
    always @(posedge clk_50m_i) rx_sync <= {rx_sync[0], debug_uart_rx_i};
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) rx(
        .clk_i(clk_50m_i), .rst_i(rst), .rx_i(rx_sync[1]), .data_o(rx_data), .valid_o(rx_valid));
    end else begin : gen_no_debug_rx
        assign rx_data=8'd0;
        assign rx_valid=1'b0;
    end endgenerate

    reg [31:0] remaining;
    // 按 remaining 的同一覆盖顺序同步保持非零状态，缩短控制使能扇出路径。
    reg remaining_active_q;
    reg [7:0] command;
    reg remote_mode, remote_armed;
    reg signed [31:0] remote_vx,remote_vy,remote_wz;
    // 再锁存保护器的速度/valid，范围检查独占下一拍，避免串到租约寄存器。
    // 零速旁路保留立即停车与重新解锁的语义。
    reg remote_pending_valid;
    reg [47:0] remote_pending_velocity;
    reg remote_checked_valid;
    reg [47:0] remote_checked_velocity;
    (* ASYNC_REG="TRUE" *) reg [1:0] remote_link_sync;
    always @(posedge clk_50m_i) remote_link_sync <= {remote_link_sync[0],remote_link_down_i};
    wire remote_zero_now=remote_velocity_i==48'd0;
    wire signed [15:0] pending_vx=remote_pending_velocity[47:32];
    wire signed [15:0] pending_vy=remote_pending_velocity[31:16];
    wire signed [15:0] pending_wz=remote_pending_velocity[15:0];
    wire pending_bounds=pending_vx>=-400 && pending_vx<=400 && pending_vy>=-400 && pending_vy<=400 &&
        pending_wz>=-1000 && pending_wz<=1000;
    wire signed [15:0] checked_vx=remote_checked_velocity[47:32];
    wire signed [15:0] checked_vy=remote_checked_velocity[31:16];
    wire signed [15:0] checked_wz=remote_checked_velocity[15:0];
    // 单字节 1/2=前后、3/4=左右、a/b=逆/顺时针；qwer/QWER 单独点动物理 CH0..3。
    // 0=立即停止。物理通道点动锁住原始计数报告模式，直到下一条整车运动命令。
    // 运动期间忽略新运动字符，不能靠连续发字符延长当前脉冲。
    wire motion_byte = rx_data == "1" || rx_data == "2" || rx_data == "3" ||
                       rx_data == "4" || rx_data == "a" || rx_data == "b";
    wire calibration_byte = rx_data == "q" || rx_data == "w" || rx_data == "e" || rx_data == "r" ||
                            rx_data == "Q" || rx_data == "W" || rx_data == "E" || rx_data == "R";
    generate if (TRUSTED_GUARDED_INPUT && !ENABLE_DEBUG_UART) begin: g_guarded_input
    // 板内命令已由 MMIO commit 与 guard 验收；仍保留独立租约和断链立即关断。
    // guard 持续重新检查当前有效命令，因此断链解除后不需要零速解锁或重放旧命令。
    always @(posedge clk_50m_i) begin
        if (rst) begin
            remaining<=0;remaining_active_q<=0;command<="0";physical_report_mode<=0;
            remote_mode<=1;remote_armed<=0;remote_vx<=0;remote_vy<=0;remote_wz<=0;
            remote_pending_valid<=0;remote_pending_velocity<=0;
            remote_checked_valid<=0;remote_checked_velocity<=0;
        end else begin
            if (remaining != 0) begin
                remaining<=remaining-1'b1;
                remaining_active_q<=remaining>1;
                if (remaining==1) begin command<="0";remote_armed<=0;end
            end
            if (remote_valid_i===1'b1) begin
                if (remote_zero_now) begin
                    remaining<=0;remaining_active_q<=0;command<="0";remote_armed<=1;
                    remote_vx<=0;remote_vy<=0;remote_wz<=0;
                end else if (!(remote_link_down_i===1'b1) && !(remote_link_sync[1]===1'b1)) begin
                    remaining<=REMOTE_TIMEOUT_CYCLES;remaining_active_q<=(REMOTE_TIMEOUT_CYCLES!=0);
                    command<="U";remote_armed<=1;
                    remote_vx<=$signed(remote_velocity_i[47:32]);
                    remote_vy<=$signed(remote_velocity_i[31:16]);
                    remote_wz<=$signed(remote_velocity_i[15:0]);
                end
            end
            if (remote_link_down_i===1'b1 || remote_link_sync[1]===1'b1) begin
                remaining<=0;remaining_active_q<=0;command<="0";remote_armed<=0;
                remote_vx<=0;remote_vy<=0;remote_wz<=0;
            end
        end
    end
    end else begin: g_external_input
    always @(posedge clk_50m_i) begin
        if (rst) begin
            remaining<=0;remaining_active_q<=1'b0;command<="0";physical_report_mode<=0;
            remote_mode<=0;remote_armed<=0;remote_vx<=0;remote_vy<=0;remote_wz<=0;
            remote_pending_valid<=1'b0;remote_pending_velocity<=48'd0;
            remote_checked_valid<=1'b0;remote_checked_velocity<=48'd0;
        end
        else begin
            remote_pending_valid <= (remote_valid_i===1'b1) && !remote_zero_now &&
                !(rx_valid && rx_data=="0") && !(remote_link_sync[1]===1'b1);
            if (remote_valid_i===1'b1 && !remote_zero_now)
                remote_pending_velocity <= remote_velocity_i;
            // 范围检查完成后，只有同批已检查命令能更新租约。
            remote_checked_valid <= remote_pending_valid && pending_bounds &&
                !(remote_valid_i===1'b1 && remote_zero_now) &&
                !(rx_valid && rx_data=="0") && !(remote_link_sync[1]===1'b1);
            if (remote_pending_valid) remote_checked_velocity <= remote_pending_velocity;
            if (remaining != 0) begin
                remaining <= remaining - 1;
                remaining_active_q <= remaining > 1;
            end
            if (rx_valid) begin
                if (rx_data == "0") begin remaining <= 0; remaining_active_q<=1'b0; command <= "0";remote_mode<=0;remote_armed<=0; end
                else if (!remote_mode && remaining == 0 && (motion_byte || calibration_byte)) begin
                    remaining <= calibration_byte ? CALIBRATION_CYCLES : JOG_CYCLES;
                    remaining_active_q <= calibration_byte ?
                        (CALIBRATION_CYCLES != 0) : (JOG_CYCLES != 0);
                    command <= rx_data;physical_report_mode<=calibration_byte;
                end
            end
            // 网络接管前先停车；续租可同时改变三轴速度。停机/超时后须先收到零速度重新解锁。
            if(remote_mode && remaining==1)begin remote_armed<=0;command<="0";end
            if(remote_valid_i===1'b1 && remote_zero_now && !(rx_valid && rx_data=="0")) begin
                // 独立保护器发出的故障/超时零速不延迟。
                remote_pending_valid<=1'b0;
                remote_checked_valid<=1'b0;
                remaining<=0;remaining_active_q<=1'b0;command<="0";remote_mode<=1;remote_armed<=1;physical_report_mode<=0;
                remote_vx<=0;remote_vy<=0;remote_wz<=0;
            end else if(remote_checked_valid && !(rx_valid && rx_data=="0") &&
                        !(remote_link_sync[1]===1'b1)) begin
                if(remote_armed)begin
                    remaining<=REMOTE_TIMEOUT_CYCLES;remaining_active_q<=(REMOTE_TIMEOUT_CYCLES != 0);command<="U";remote_mode<=1;remote_armed<=1;physical_report_mode<=0;
                    remote_vx<=checked_vx;remote_vy<=checked_vy;remote_wz<=checked_wz;
                end
            end
            if(remote_link_sync[1]===1'b1 && remote_mode)begin
                remote_pending_valid<=1'b0;remote_checked_valid<=1'b0;
                remaining<=0;remaining_active_q<=1'b0;command<="0";remote_armed<=0;
            end
        end
    end
    end endgenerate
    // 链路同步后的状态只进入控制运算；异步断链在最终驱动口直接切断输出。
    // 避免 PHY/PLL 状态经除法器使能的长组合路径传播到算术寄存器。
    wire enabled = remaining_active_q && !rst && !user_reset_i && !(remote_mode && remote_link_sync[1]===1'b1);
    wire calibration_active=enabled && (command=="q" || command=="w" || command=="e" || command=="r" ||
                                        command=="Q" || command=="W" || command=="E" || command=="R");
    wire controller_enabled=enabled && !calibration_active;
    reg signed [31:0] vx, vy, wz;
    always @* begin
        vx = 0; vy = 0; wz = 0;
        case (command)
            "1": vx = motor_closed_loop_config::JOG_MM_S;
            "2": vx = -motor_closed_loop_config::JOG_MM_S;
            "3": vy = motor_closed_loop_config::JOG_MM_S;
            "4": vy = -motor_closed_loop_config::JOG_MM_S;
            "a": wz = motor_closed_loop_config::JOG_MRAD_S;
            "b": wz = -motor_closed_loop_config::JOG_MRAD_S;
            default: begin end
        endcase
        if(remote_mode)begin vx=remote_vx;vy=remote_vy;wz=remote_wz;end
    end
    wire [3:0] wheel_a, wheel_b;
    wire [3:0] wheel_in1, wheel_in2, encoder_invalid;
    wire [3:0] mapped_motor_in1,mapped_motor_in2;
    reg [3:0] calibration_in1,calibration_in2;
    wire [127:0] position_ticks, measured_mm_s, target_mm_s;
    wire [127:0] physical_channel_ticks;
    reg [15:0] calibration_pwm_count;
    wire [15:0] calibration_duty=(PWM_PERIOD * motor_closed_loop_config::CALIBRATION_PWM_PERCENT) / 100;
    always @(posedge clk_50m_i) begin
        if(rst || calibration_pwm_count==PWM_PERIOD-1)calibration_pwm_count<=0;
        else calibration_pwm_count<=calibration_pwm_count+1'b1;
    end
    always @* begin
        calibration_in1=0;calibration_in2=0;
        if(calibration_active && calibration_pwm_count<calibration_duty) begin
            case(command)
                "q":calibration_in1[0]=1;"Q":calibration_in2[0]=1;
                "w":calibration_in1[1]=1;"W":calibration_in2[1]=1;
                "e":calibration_in1[2]=1;"E":calibration_in2[2]=1;
                "r":calibration_in1[3]=1;"R":calibration_in2[3]=1;
                default:begin end
            endcase
        end
    end
    wire force_output_stop=rst || user_reset_i ||
        ((TRUSTED_GUARDED_INPUT || remote_mode) && remote_link_down_i===1'b1);
    assign motor_in1_o=force_output_stop ? 4'd0 : calibration_active ? calibration_in1 : mapped_motor_in1;
    assign motor_in2_o=force_output_stop ? 4'd0 : calibration_active ? calibration_in2 : mapped_motor_in2;
    // 调试行在物理点动模式下始终按编码器物理 CH0..3 输出、不过软件方向反相。
    for(genvar report_wheel=0;report_wheel<4;report_wheel=report_wheel+1)begin: physical_report
        wire signed [31:0] logical_count=position_ticks[report_wheel*32+:32];
        wire signed [31:0] raw_count=ENCODER_REVERSE[report_wheel] ? -logical_count : logical_count;
        assign physical_channel_ticks[ENCODER_CHANNEL_MAP[report_wheel*2+:2]*32+:32]=raw_count;
    end
    wire body_valid;
    mecanum_motor_control #(
        .CLK_HZ(CLK_HZ), .SAMPLE_HZ(SAMPLE_HZ),
        .GEOMETRY_MM(motor_closed_loop_config::GEOMETRY_MM),
        .MAX_WHEEL_MM_S(motor_closed_loop_config::MAX_WHEEL_MM_S),
        .STEP_MM_S(motor_closed_loop_config::STEP_MM_S),
        .MM_PER_TICK_Q16(MM_PER_TICK_Q16), .PWM_PERIOD(PWM_PERIOD),
        .KP(motor_closed_loop_config::KP), .KD(motor_closed_loop_config::KD),
        .PWM_BRAKE_MODE(motor_closed_loop_config::PWM_BRAKE_MODE),
        .ENCODER_REVERSE(ENCODER_REVERSE),
        .MOTOR_REVERSE(MOTOR_REVERSE)
    ) control(
        .clk_i(clk_50m_i), .rst_n_i(!rst), .enable_i(controller_enabled),
        .vx_mm_s_i(vx), .vy_mm_s_i(vy), .wz_mrad_s_i(wz),
        .encoder_a_i(wheel_a), .encoder_b_i(wheel_b), .in1_o(wheel_in1), .in2_o(wheel_in2),
        .position_ticks_o(position_ticks), .measured_mm_s_o(measured_mm_s),
        .target_mm_s_o(target_mm_s), .encoder_invalid_o(encoder_invalid),
        .sample_valid_o(body_valid));
    reg [3:0] invalid_since_sample;
    reg [3:0] invalid_snapshot;
    assign odometry_invalid_o = invalid_snapshot;
    fusion_mecanum_odometry #(.GEOMETRY_MM(motor_closed_loop_config::GEOMETRY_MM)) fusion_odometry(
        .clk(clk_50m_i),.rst(rst),.sample_i(body_valid),.measured_i(measured_mm_s),
        .valid_o(odometry_valid_o),.vx_o(odometry_vx_o),.vy_o(odometry_vy_o),.wz_o(odometry_wz_o));
    // 将与本次测速相同采样时刻的四轮累计计数送给静止检测器。
    reg [127:0] odometry_ticks_snapshot,odometry_wheels_snapshot;
    assign odometry_ticks_o = odometry_ticks_snapshot;
    assign odometry_wheels_o = odometry_wheels_snapshot;
    always @(posedge clk_50m_i) begin
        if(rst) begin
            invalid_since_sample <= 0;invalid_snapshot<=0;odometry_ticks_snapshot<=0;odometry_wheels_snapshot<=0;
        end else if(body_valid) begin
            invalid_snapshot<=invalid_since_sample | encoder_invalid;
            invalid_since_sample <= 0;
            odometry_ticks_snapshot<=position_ticks;
            odometry_wheels_snapshot<=measured_mm_s;
        end else invalid_since_sample <= invalid_since_sample | encoder_invalid;
    end
    // 在测速发布拍锁存轮计数与命令；等待同组正运动学计算结束再发布遥测。
    reg [575:0] telemetry_wheel_snapshot;
    reg [3:0] active_physical_mask;
    integer active_wheel;
    always @* begin
        active_physical_mask=0;
        if(calibration_active) begin
            case(command)
                "q","Q":active_physical_mask=1;
                "w","W":active_physical_mask=2;
                "e","E":active_physical_mask=4;
                "r","R":active_physical_mask=8;
                default:active_physical_mask=0;
            endcase
        end else if(controller_enabled) begin
            for(active_wheel=0;active_wheel<4;active_wheel=active_wheel+1)
                if(target_mm_s[active_wheel*32+:32]!=0)
                    active_physical_mask[MOTOR_CHANNEL_MAP[active_wheel*2+:2]]=1;
        end
    end
    always @(posedge clk_50m_i) begin
        if(rst) begin
            telemetry_valid_o<=0;telemetry_payload_o<=0;telemetry_wheel_snapshot<=0;
        end else begin
            telemetry_valid_o<=0;
            if(body_valid) telemetry_wheel_snapshot<={
                MOTOR_CHANNEL_MAP,ENCODER_CHANNEL_MAP,4'd0,MOTOR_REVERSE,4'd0,ENCODER_REVERSE,
                (enabled ? command : 8'h30),
                (calibration_active ? 8'(motor_closed_loop_config::CALIBRATION_PWM_PERCENT) : 8'd0),
                4'd0,active_physical_mask,4'd0,(invalid_since_sample|encoder_invalid),
                position_ticks[0+:32],position_ticks[32+:32],position_ticks[64+:32],position_ticks[96+:32],
                physical_channel_ticks[0+:32],physical_channel_ticks[32+:32],
                physical_channel_ticks[64+:32],physical_channel_ticks[96+:32],
                measured_mm_s[0+:32],measured_mm_s[32+:32],measured_mm_s[64+:32],measured_mm_s[96+:32],
                target_mm_s[0+:32],target_mm_s[32+:32],target_mm_s[64+:32],target_mm_s[96+:32]};
            if(odometry_valid_o) begin
                telemetry_payload_o<={telemetry_wheel_snapshot,odometry_vx_o,odometry_vy_o,odometry_wz_o,32'd0};
                telemetry_valid_o<=1;
            end
        end
    end
    // 分别按集中配置重排编码器输入和驱动输出；位映射低两位对应 FL。
    genvar wheel_index;
    generate for (wheel_index=0; wheel_index<4; wheel_index=wheel_index+1) begin: channel_map
        assign wheel_a[wheel_index] = encoder_a_i[ENCODER_CHANNEL_MAP[wheel_index*2 +: 2]];
        assign wheel_b[wheel_index] = encoder_b_i[ENCODER_CHANNEL_MAP[wheel_index*2 +: 2]];
        assign mapped_motor_in1[MOTOR_CHANNEL_MAP[wheel_index*2 +: 2]] = wheel_in1[wheel_index];
        assign mapped_motor_in2[MOTOR_CHANNEL_MAP[wheel_index*2 +: 2]] = wheel_in2[wheel_index];
    end endgenerate

    function automatic [63:0] hex32(input [31:0] value);
        reg [3:0] nibble;
        begin
            for (integer k=0; k<8; k=k+1) begin
                nibble = value[31-k*4-:4];
                hex32[63-k*8-:8] = nibble < 10 ? 8'h30+nibble : 8'h41+nibble-10;
            end
        end
    endfunction
    // 输出39字节状态与四路32位计数帧，计数按 FL/FR/RL/RR 排列。
    wire [7:0] state_char = physical_report_mode ? (calibration_active ? command : "C") :
        (enabled ? command : "0");
    generate if(ENABLE_DEBUG_UART)begin : gen_debug_tx
    wire [127:0] report_ticks=physical_report_mode ? physical_channel_ticks : position_ticks;
    wire [311:0] frame = {state_char,8'h20,hex32(report_ticks[0+:32]),8'h20,
        hex32(report_ticks[32+:32]),8'h20,hex32(report_ticks[64+:32]),8'h20,
        hex32(report_ticks[96+:32]),16'h0d0a};
    reg [31:0] report_timer;
    wire frame_ready;
    wire send_frame = report_timer == REPORT_CYCLES-1 && frame_ready;
    always @(posedge clk_50m_i)
        if (rst || report_timer == REPORT_CYCLES-1) report_timer <= 0;
        else report_timer <= report_timer+1;
    wire [7:0] tx_data;
    wire tx_start, tx_busy;
    packet_uart_tx #(.PACKET_BYTES(39)) packet(.clk(clk_50m_i), .rst(rst),
        .packet_valid_i(send_frame), .packet_data_i(frame), .packet_ready_o(frame_ready),
        .uart_busy_i(tx_busy), .uart_data_o(tx_data), .uart_start_o(tx_start), .packet_pending_o());
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) tx(.clk_i(clk_50m_i), .rst_i(rst),
        .data_i(tx_data), .start_i(tx_start), .tx_o(debug_uart_tx_o), .busy_o(tx_busy));
    end else begin : gen_no_debug_tx
        assign debug_uart_tx_o=1'b1;
    end endgenerate
endmodule

// Integrated UART support: uart_rx.sv
`timescale 1ns/1ps

// 电机输入已由调用端同步；保留同步高有效复位和逐位数据输出。
module uart_rx #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer BAUD=115200
)(
    input wire clk_i,rst_i,rx_i,
    output wire [7:0] data_o,
    output wire valid_o
);
    localparam integer BIT_CYCLES=CLK_HZ/BAUD;
    localparam integer HALF_CYCLES=(BIT_CYCLES/2<1)?1:BIT_CYCLES/2;
    bx72_uart_rx #(.BIT_CYCLES(BIT_CYCLES),.HALF_CYCLES(HALF_CYCLES),
        .ASYNC_RESET(0),.DATA_ON_SAMPLE(1)) receiver(
        .clk(clk_i),.reset_n(!rst_i),.rx_i(rx_i),.data_o(data_o),
        .valid_o(valid_o),.framing_error_o());
endmodule

// Integrated UART support: uart_tx.sv
`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115200
) (
    input  wire       clk_i,
    input  wire       rst_i,
    input  wire [7:0] data_i,
    input  wire       start_i,
    output reg        tx_o,
    output reg        busy_o
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA = 2'd2;
    localparam [1:0] ST_STOP = 2'd3;

    reg [1:0] state;
    reg [31:0] count;
    reg [2:0] bit_index;
    reg [7:0] data_reg;

    always @(posedge clk_i) begin
        if (rst_i) begin
            state <= ST_IDLE;
            count <= 32'd0;
            bit_index <= 3'd0;
            data_reg <= 8'd0;
            tx_o <= 1'b1;
            busy_o <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx_o <= 1'b1;
                    busy_o <= 1'b0;
                    if (start_i) begin
                        data_reg <= data_i;
                        bit_index <= 3'd0;
                        count <= CLKS_PER_BIT - 1;
                        tx_o <= 1'b0;
                        busy_o <= 1'b1;
                        state <= ST_START;
                    end
                end

                ST_START: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        tx_o <= data_reg[0];
                        count <= CLKS_PER_BIT - 1;
                        state <= ST_DATA;
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                ST_DATA: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        count <= CLKS_PER_BIT - 1;
                        if (bit_index == 3'd7) begin
                            tx_o <= 1'b1;
                            state <= ST_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx_o <= data_reg[bit_index + 1'b1];
                        end
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                ST_STOP: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        tx_o <= 1'b1;
                        busy_o <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

// Integrated UART support: packet_uart_tx.v
`timescale 1ns/1ps

// 固定长度报文串行发送器；锁存完整帧后逐字节输出，忙期间不接收新帧。
module packet_uart_tx #(
    parameter integer PACKET_BYTES = 32
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         packet_valid_i,
    input  wire [PACKET_BYTES*8-1:0] packet_data_i,
    output wire         packet_ready_o,
    input  wire         uart_busy_i,
    output reg  [7:0]   uart_data_o,
    output reg          uart_start_o,
    output wire         packet_pending_o
);
    reg [PACKET_BYTES*8-1:0] shift_reg;
    localparam integer INDEX_WIDTH = (PACKET_BYTES > 1) ? $clog2(PACKET_BYTES) : 1;
    reg [INDEX_WIDTH-1:0] byte_index;
    reg pending;
    reg active;

    assign packet_ready_o = !pending && !active;
    assign packet_pending_o = pending || active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            shift_reg    <= {PACKET_BYTES*8{1'b0}};
            byte_index   <= {INDEX_WIDTH{1'b0}};
            pending      <= 1'b0;
            active       <= 1'b0;
            uart_data_o  <= 8'd0;
            uart_start_o <= 1'b0;
        end else begin
            uart_start_o <= 1'b0;
            if (!active) begin
                if (pending) begin
                    if (!uart_busy_i) begin
                        uart_data_o  <= shift_reg[PACKET_BYTES*8-1 -: 8];
                        uart_start_o <= 1'b1;
                        active       <= 1'b1;
                        pending      <= 1'b0;
                        byte_index   <= {INDEX_WIDTH{1'b0}};
                    end
                end else if (packet_valid_i) begin
                    shift_reg <= packet_data_i;
                    pending   <= 1'b1;
                end
            end else begin
                if (uart_start_o) begin
                    uart_start_o <= 1'b0;
                end else if (!uart_busy_i) begin
                    if (byte_index == PACKET_BYTES - 1) begin
                        active     <= 1'b0;
                        byte_index <= {INDEX_WIDTH{1'b0}};
                    end else begin
                        shift_reg    <= shift_reg << 8;
                        uart_data_o  <= (shift_reg << 8) >> (PACKET_BYTES*8-8);
                        uart_start_o <= 1'b1;
                        byte_index   <= byte_index + 1'b1;
                    end
                end
            end
        end
    end
endmodule

// ==================== bx72_command_control.sv ====================
`timescale 1ns/1ps
// 单一通信入口的手动/自动仲裁。模式切换先停车，自动租约超时后必须重新启用。
module bx72_command_control #(
    parameter integer LEASE_CYCLES=10000000,
    parameter integer ALLOW_LEGACY_AUTO=1
)(
    input wire clk,reset_n,
    input wire manual_valid_i,
    input wire [47:0] manual_velocity_i,
    input wire link_down_i,mode_valid_i,auto_enable_i,
    input wire planner_valid_i,planner_scan_valid_i,planner_fault_i,
    input wire [47:0] planner_velocity_i,
    input wire [63:0] now_us_i,
    input wire task_accepted_i,task_done_i,task_fault_i,
    input wire [7:0] task_mode_i,task_flags_i,
    input wire [63:0] task_expiry_us_i,
    input wire navigation_valid_i,input wire [47:0] navigation_velocity_i,
    input wire cancel_i,
    output wire navigation_cancel_o,
    output reg auto_mode_o,
    output wire valid_o,link_down_o,
    output wire [47:0] velocity_o
);
    localparam integer AGE_W=LEASE_CYCLES<2 ? 1 : $clog2(LEASE_CYCLES+1);
    reg [AGE_W-1:0] age;
    reg owned;
    reg task_owned;
    reg [7:0] task_mode,task_flags;
    reg [63:0] task_expiry;
    wire task_expired=task_owned && now_us_i>=task_expiry;
    wire task_link_stop=task_owned && link_down_i && task_mode!=3 && !task_flags[0];
    wire task_cancel=task_owned && (manual_valid_i || mode_valid_i || cancel_i===1'b1 ||
        task_fault_i===1'b1 || task_expired || task_link_stop);
    assign navigation_cancel_o=task_cancel;
    wire lease_expired=owned && !task_owned && age>=LEASE_CYCLES-1 && !manual_valid_i && !mode_valid_i;
    wire mode_change=mode_valid_i && (!auto_enable_i || !auto_mode_o);
    wire manual_takeover=manual_valid_i && auto_mode_o;
    wire stop_now=mode_change || manual_takeover || lease_expired || task_cancel || (task_owned && task_done_i===1'b1);
    assign link_down_o=task_owned ? (task_cancel || task_fault_i===1'b1) :
        (link_down_i || (auto_mode_o && (!planner_scan_valid_i || planner_fault_i || lease_expired)));
    assign valid_o=stop_now || (task_owned ? navigation_valid_i : (auto_mode_o ? planner_valid_i : manual_valid_i));
    assign velocity_o=(stop_now || link_down_o) ? 48'd0 :
        task_owned ? navigation_velocity_i : (auto_mode_o ? planner_velocity_i : manual_velocity_i);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin auto_mode_o<=0;owned<=0;age<=0;end
        else if(task_accepted_i===1'b1 || link_down_i || lease_expired)begin auto_mode_o<=0;owned<=0;age<=0;end
        else if(mode_valid_i)begin auto_mode_o<=auto_enable_i && ALLOW_LEGACY_AUTO!=0;owned<=auto_enable_i && ALLOW_LEGACY_AUTO!=0;age<=0;end
        else if(manual_valid_i)begin auto_mode_o<=0;owned<=1;age<=0;end
        else if(owned)age<=age+1'b1;
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin task_owned<=0;task_mode<=0;task_flags<=0;task_expiry<=0;end
        else if(task_cancel)task_owned<=0;
        else if(task_accepted_i===1'b1)begin
            task_owned<=1;task_mode<=task_mode_i;task_flags<=task_flags_i;task_expiry<=task_expiry_us_i;
        end
        else if(task_done_i===1'b1)task_owned<=0;
    end
endmodule

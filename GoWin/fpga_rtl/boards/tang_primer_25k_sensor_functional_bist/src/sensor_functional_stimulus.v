`timescale 1ns/1ps

// 完全可综合的片上激励发生器。
// 它只验证传感器接口 RTL 在 FPGA 内的正常路径与关键边界，不等价于
// 外部传感器、连线、电平转换、模拟前端或光机系统的实物测试。
module sensor_functional_stimulus #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer SENSOR_UART_BAUD_RATE = 3125000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        hc_trigger_i,
    input  wire        tsl_adc_start_i,
    input  wire [6:0]  tsl_pixel_index_i,
    output reg         digital_ir_n_o,
    output wire [4:0]  sensor_uart_o,
    output reg         hc_echo_o,
    output reg         itr_pulse_o,
    output reg         bumper_left_n_o,
    output reg         bumper_right_n_o,
    output reg         bumper_clear_o,
    output reg         camera_vsync_o,
    output reg         camera_href_o,
    output reg [7:0]   mt9v03x_data_o,
    output reg [7:0]   ov5640_data_o,
    output reg [7:0]   ov7725_data_o,
    output reg [7:0]   scc8660_data_o,
    output reg [7:0]   multi_ir_n_o,
    output reg         encoder_a_o,
    output reg         encoder_b_o,
    output reg         tcrt5000_n_o,
    output reg [11:0]  tsl_adc_data_o,
    output reg         tsl_adc_valid_o,
    output reg [4:0]   uart_channel_done_o,
    output reg         timeline_done_o
);
    localparam integer SAFE_SENSOR_BAUD =
        (SENSOR_UART_BAUD_RATE < 1) ? 1 : SENSOR_UART_BAUD_RATE;
    localparam integer RAW_SENSOR_CPB = CLK_FREQ_HZ / SAFE_SENSOR_BAUD;
    localparam integer SENSOR_CPB = (RAW_SENSOR_CPB < 4) ? 4 : RAW_SENSOR_CPB;

    reg [15:0] timeline_q;

    // 五路片上 UART 激励依次发送，未选中的接收线始终保持空闲高电平。
    // 通道：0 GNSS、1 GY-39、2 JY61P、3 MaxSonar、4 TF-Luna。
    reg        uart_active_q;
    reg        uart_all_done_q;
    reg        uart_serial_bit_q;
    reg [2:0]  uart_channel_q;
    reg [5:0]  uart_byte_index_q;
    reg [3:0]  uart_bit_index_q;
    reg [31:0] uart_clock_count_q;
    reg [31:0] uart_gap_count_q;
    reg [7:0]  uart_byte_q;

    assign sensor_uart_o[0] =
        (uart_active_q && uart_channel_q == 0) ? uart_serial_bit_q : 1'b1;
    assign sensor_uart_o[1] =
        (uart_active_q && uart_channel_q == 1) ? uart_serial_bit_q : 1'b1;
    assign sensor_uart_o[2] =
        (uart_active_q && uart_channel_q == 2) ? uart_serial_bit_q : 1'b1;
    assign sensor_uart_o[3] =
        (uart_active_q && uart_channel_q == 3) ? uart_serial_bit_q : 1'b1;
    assign sensor_uart_o[4] =
        (uart_active_q && uart_channel_q == 4) ? uart_serial_bit_q : 1'b1;

    function [5:0] uart_stream_length;
        input [2:0] channel;
        begin
            case (channel)
                3'd0: uart_stream_length = 6'd17;
                3'd1: uart_stream_length = 6'd9;
                3'd2: uart_stream_length = 6'd11;
                3'd3: uart_stream_length = 6'd6;
                default: uart_stream_length = 6'd9;
            endcase
        end
    endfunction

    // 黄金字节序列独立写明，不从被测解析器推导。
    function [7:0] uart_stream_byte;
        input [2:0] channel;
        input [5:0] index;
        begin
            uart_stream_byte = 8'h00;
            case (channel)
                3'd0: begin
                    case (index)
                        0:uart_stream_byte="$";  1:uart_stream_byte="G";
                        2:uart_stream_byte="P";  3:uart_stream_byte="G";
                        4:uart_stream_byte="G";  5:uart_stream_byte="A";
                        6:uart_stream_byte=",";  7:uart_stream_byte="1";
                        8:uart_stream_byte=",";  9:uart_stream_byte="2";
                        10:uart_stream_byte=","; 11:uart_stream_byte="3";
                        12:uart_stream_byte="*";13:uart_stream_byte="4";
                        14:uart_stream_byte="A";15:uart_stream_byte=8'h0d;
                        default:uart_stream_byte=8'h0a;
                    endcase
                end
                3'd1: begin
                    case (index)
                        0:uart_stream_byte=8'h5a;1:uart_stream_byte=8'h5a;
                        2:uart_stream_byte=8'h15;3:uart_stream_byte=8'h04;
                        4:uart_stream_byte=8'h00;5:uart_stream_byte=8'h00;
                        6:uart_stream_byte=8'h30;7:uart_stream_byte=8'h39;
                        default:uart_stream_byte=8'h36;
                    endcase
                end
                3'd2: begin
                    case (index)
                        0:uart_stream_byte=8'h55;1:uart_stream_byte=8'h53;
                        2:uart_stream_byte=8'h34;3:uart_stream_byte=8'h12;
                        4:uart_stream_byte=8'h00;5:uart_stream_byte=8'h80;
                        6:uart_stream_byte=8'hfe;7:uart_stream_byte=8'hff;
                        8:uart_stream_byte=8'h78;9:uart_stream_byte=8'h56;
                        default:uart_stream_byte=8'h39;
                    endcase
                end
                3'd3: begin
                    case (index)
                        0:uart_stream_byte="R";1:uart_stream_byte="1";
                        2:uart_stream_byte="2";3:uart_stream_byte="3";
                        4:uart_stream_byte="4";default:uart_stream_byte=8'h0d;
                    endcase
                end
                default: begin
                    case (index)
                        0:uart_stream_byte=8'h59;1:uart_stream_byte=8'h59;
                        2:uart_stream_byte=8'hd2;3:uart_stream_byte=8'h04;
                        4:uart_stream_byte=8'h37;5:uart_stream_byte=8'h02;
                        6:uart_stream_byte=8'h38;7:uart_stream_byte=8'h08;
                        default:uart_stream_byte=8'h01;
                    endcase
                end
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_active_q <= 1'b0;
            uart_all_done_q <= 1'b0;
            uart_serial_bit_q <= 1'b1;
            uart_channel_q <= 3'd0;
            uart_byte_index_q <= 6'd0;
            uart_bit_index_q <= 4'd0;
            uart_clock_count_q <= 32'd0;
            uart_gap_count_q <= 32'd0;
            uart_byte_q <= 8'd0;
            uart_channel_done_o <= 5'd0;
        end else if (!uart_all_done_q) begin
            if (!uart_active_q) begin
                uart_serial_bit_q <= 1'b1;
                uart_clock_count_q <= 32'd0;
                if (uart_gap_count_q < SENSOR_CPB * 3) begin
                    uart_gap_count_q <= uart_gap_count_q + 1'b1;
                end else begin
                    uart_gap_count_q <= 32'd0;
                    uart_byte_index_q <= 6'd0;
                    uart_bit_index_q <= 4'd0;
                    uart_byte_q <= uart_stream_byte(uart_channel_q, 0);
                    uart_serial_bit_q <= 1'b0;
                    uart_active_q <= 1'b1;
                end
            end else if (uart_clock_count_q >= SENSOR_CPB - 1) begin
                uart_clock_count_q <= 32'd0;
                if (uart_bit_index_q == 4'd0) begin
                    uart_bit_index_q <= 4'd1;
                    uart_serial_bit_q <= uart_byte_q[0];
                end else if (uart_bit_index_q < 4'd8) begin
                    uart_serial_bit_q <= uart_byte_q[uart_bit_index_q];
                    uart_bit_index_q <= uart_bit_index_q + 1'b1;
                end else if (uart_bit_index_q == 4'd8) begin
                    uart_serial_bit_q <= 1'b1;
                    uart_bit_index_q <= 4'd9;
                end else if (uart_byte_index_q + 1'b1 <
                             uart_stream_length(uart_channel_q)) begin
                    uart_byte_index_q <= uart_byte_index_q + 1'b1;
                    uart_byte_q <= uart_stream_byte(
                        uart_channel_q, uart_byte_index_q + 1'b1);
                    uart_bit_index_q <= 4'd0;
                    uart_serial_bit_q <= 1'b0;
                end else begin
                    uart_channel_done_o[uart_channel_q] <= 1'b1;
                    uart_active_q <= 1'b0;
                    uart_serial_bit_q <= 1'b1;
                    if (uart_channel_q == 3'd4)
                        uart_all_done_q <= 1'b1;
                    else
                        uart_channel_q <= uart_channel_q + 1'b1;
                end
            end else begin
                uart_clock_count_q <= uart_clock_count_q + 1'b1;
            end
        end
    end

    // 数字输入、DVP 与正交编码器激励时间表。包含短毛刺、有效状态和
    // 编码器 00->11 非法跳变，便于检查滤波与错误边界。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeline_q <= 16'd0;
            digital_ir_n_o <= 1'b1;
            itr_pulse_o <= 1'b0;
            bumper_left_n_o <= 1'b1;
            bumper_right_n_o <= 1'b1;
            bumper_clear_o <= 1'b0;
            camera_vsync_o <= 1'b0;
            camera_href_o <= 1'b0;
            mt9v03x_data_o <= 8'd0;
            ov5640_data_o <= 8'd0;
            ov7725_data_o <= 8'd0;
            scc8660_data_o <= 8'd0;
            multi_ir_n_o <= 8'hff;
            encoder_a_o <= 1'b0;
            encoder_b_o <= 1'b0;
            tcrt5000_n_o <= 1'b1;
            timeline_done_o <= 1'b0;
        end else begin
            if (timeline_q != 16'hffff)
                timeline_q <= timeline_q + 1'b1;
            case (timeline_q)
                16'd8:  digital_ir_n_o <= 1'b0;
                16'd9:  digital_ir_n_o <= 1'b1;
                16'd20: digital_ir_n_o <= 1'b0;

                16'd11: multi_ir_n_o[5] <= 1'b0;
                16'd12: multi_ir_n_o[5] <= 1'b1;
                16'd30: multi_ir_n_o[1] <= 1'b0;

                16'd14: bumper_left_n_o <= 1'b0;
                16'd15: bumper_left_n_o <= 1'b1;
                16'd40: bumper_left_n_o <= 1'b0;

                16'd50: tcrt5000_n_o <= 1'b0;

                16'd60: itr_pulse_o <= 1'b1;
                16'd62: itr_pulse_o <= 1'b0;
                16'd75: itr_pulse_o <= 1'b1;
                16'd77: itr_pulse_o <= 1'b0;

                16'd100: camera_vsync_o <= 1'b1;
                16'd104: camera_vsync_o <= 1'b0;
                16'd108: begin
                    camera_href_o <= 1'b1;
                    mt9v03x_data_o <= 8'h3c;
                    ov5640_data_o <= 8'h00;
                    ov7725_data_o <= 8'hf8;
                    scc8660_data_o <= 8'h07;
                end
                16'd109: begin
                    mt9v03x_data_o <= 8'ha5;
                    ov5640_data_o <= 8'hf8;
                    ov7725_data_o <= 8'h00;
                    scc8660_data_o <= 8'he0;
                end
                16'd110: camera_href_o <= 1'b0;

                16'd200: begin encoder_a_o <= 1'b0; encoder_b_o <= 1'b1; end
                16'd206: begin encoder_a_o <= 1'b1; encoder_b_o <= 1'b1; end
                16'd212: begin encoder_a_o <= 1'b1; encoder_b_o <= 1'b0; end
                16'd218: begin encoder_a_o <= 1'b0; encoder_b_o <= 1'b0; end
                16'd230: begin encoder_a_o <= 1'b1; encoder_b_o <= 1'b1; end
                16'd236: begin encoder_a_o <= 1'b0; encoder_b_o <= 1'b0; end
                16'd300: timeline_done_o <= 1'b1;
                default: begin end
            endcase
        end
    end

    // HC-SR04：在每个片上 trigger 后延迟 4 拍，再给出 58 拍回波。
    reg hc_trigger_d_q;
    reg [1:0] hc_state_q;
    reg [7:0] hc_count_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hc_trigger_d_q <= 1'b0;
            hc_state_q <= 2'd0;
            hc_count_q <= 8'd0;
            hc_echo_o <= 1'b0;
        end else begin
            hc_trigger_d_q <= hc_trigger_i;
            case (hc_state_q)
                2'd0: begin
                    hc_echo_o <= 1'b0;
                    if (hc_trigger_d_q && !hc_trigger_i) begin
                        hc_count_q <= 8'd0;
                        hc_state_q <= 2'd1;
                    end
                end
                2'd1: begin
                    if (hc_count_q >= 8'd3) begin
                        hc_echo_o <= 1'b1;
                        hc_count_q <= 8'd0;
                        hc_state_q <= 2'd2;
                    end else begin
                        hc_count_q <= hc_count_q + 1'b1;
                    end
                end
                default: begin
                    if (hc_count_q >= 8'd57) begin
                        hc_echo_o <= 1'b0;
                        hc_count_q <= 8'd0;
                        hc_state_q <= 2'd0;
                    end else begin
                        hc_count_q <= hc_count_q + 1'b1;
                    end
                end
            endcase
        end
    end

    // TSL1401 外部 ADC 模型：每个 adc_start 延迟一拍返回 A00h+像素号。
    reg tsl_adc_pending_q;
    reg [11:0] tsl_adc_pending_data_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tsl_adc_pending_q <= 1'b0;
            tsl_adc_pending_data_q <= 12'd0;
            tsl_adc_data_o <= 12'd0;
            tsl_adc_valid_o <= 1'b0;
        end else begin
            tsl_adc_valid_o <= 1'b0;
            if (tsl_adc_pending_q) begin
                tsl_adc_data_o <= tsl_adc_pending_data_q;
                tsl_adc_valid_o <= 1'b1;
                tsl_adc_pending_q <= 1'b0;
            end
            if (tsl_adc_start_i) begin
                tsl_adc_pending_data_q <= {5'b10100, tsl_pixel_index_i};
                tsl_adc_pending_q <= 1'b1;
            end
        end
    end
endmodule

`timescale 1ns/1ps

// VL53L1X I2C 例程：软复位、装载 ST Ultra Lite Driver 默认配置、
// 启动连续测距并轮询毫米距离。默认配置覆盖 0x002D..0x0087。
module vl53l1x_i2c #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer RESET_DELAY_CYCLES = 27000,
    parameter integer BOOT_RETRY_LIMIT = 100,
    parameter integer CALIBRATION_READY_RETRY_LIMIT = 255,
    // 0：I2C 约 1.8 V；1：I2C 使用外部 2.8 V I/O 供电。必须与板上上拉电压一致。
    parameter IOVDD_2V8 = 1'b0,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire        clk,
    input  wire        reset_n,
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,
    output reg         initialized,
    output reg         bus_error,
    output reg [4:0]   range_status,
    output reg [15:0]  distance_mm,
    output reg         distance_valid
);

    localparam [6:0] I2C_ADDR = 7'h29;
    localparam [15:0] REG_SOFT_RESET       = 16'h0000;
    localparam [15:0] REG_VHV_TIMEOUT      = 16'h0008;
    localparam [15:0] REG_VHV_INIT         = 16'h000B;
    localparam [15:0] REG_GPIO_STATUS      = 16'h0031;
    localparam [15:0] REG_INTERRUPT_CLEAR  = 16'h0086;
    localparam [15:0] REG_MODE_START       = 16'h0087;
    localparam [15:0] REG_RANGE_STATUS     = 16'h0089;
    localparam [15:0] REG_DISTANCE_MM      = 16'h0096;
    localparam [15:0] REG_BOOT_STATE       = 16'h00E5;

    localparam [4:0] ST_STARTUP_DELAY = 5'd0;
    localparam [4:0] ST_INIT_START    = 5'd1;
    localparam [4:0] ST_INIT_WAIT     = 5'd2;
    localparam [4:0] ST_RESET_DELAY   = 5'd3;
    localparam [4:0] ST_BOOT_START    = 5'd4;
    localparam [4:0] ST_BOOT_WAIT     = 5'd5;
    localparam [4:0] ST_CAL_READY_START = 5'd6;
    localparam [4:0] ST_CAL_READY_WAIT  = 5'd7;
    localparam [4:0] ST_CAL_WRITE_START = 5'd8;
    localparam [4:0] ST_CAL_WRITE_WAIT  = 5'd9;
    localparam [4:0] ST_SAMPLE_DELAY  = 5'd10;
    localparam [4:0] ST_STATUS_START  = 5'd11;
    localparam [4:0] ST_STATUS_WAIT   = 5'd12;
    localparam [4:0] ST_RANGE_STATUS_START = 5'd13;
    localparam [4:0] ST_RANGE_STATUS_WAIT  = 5'd14;
    localparam [4:0] ST_RANGE_START   = 5'd15;
    localparam [4:0] ST_RANGE_WAIT    = 5'd16;
    localparam [4:0] ST_CLEAR_START   = 5'd17;
    localparam [4:0] ST_CLEAR_WAIT    = 5'd18;
    localparam [4:0] ST_PUBLISH       = 5'd19;

    localparam integer STARTUP_DELAY =
        (STARTUP_DELAY_CYCLES < 1) ? 1 : STARTUP_DELAY_CYCLES;
    localparam integer RESET_DELAY =
        (RESET_DELAY_CYCLES < 1) ? 1 : RESET_DELAY_CYCLES;
    localparam integer BOOT_RETRIES =
        (BOOT_RETRY_LIMIT < 1) ? 1 : BOOT_RETRY_LIMIT;
    localparam integer CALIBRATION_READY_RETRIES =
        (CALIBRATION_READY_RETRY_LIMIT < 1) ? 1 : CALIBRATION_READY_RETRY_LIMIT;
    localparam integer SAMPLE_INTERVAL =
        (SAMPLE_INTERVAL_CYCLES < 1) ? 1 : SAMPLE_INTERVAL_CYCLES;

    reg [4:0] state;
    reg [6:0] init_index;
    reg [15:0] boot_retry_count;
    reg [15:0] calibration_retry_count;
    reg [2:0] calibration_step;
    reg publish_pending;
    reg [31:0] delay_count;
    reg txn_start, txn_read;
    reg [15:0] txn_reg_addr, txn_write_data;
    reg [1:0] txn_write_length, txn_read_length;
    wire txn_busy, txn_done, txn_error, txn_rx_valid;
    wire [1:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes [0:1];
    integer i;

    // 该表逐字节对应 ST VL53L1X Ultra Lite Driver 的 0x002D..0x0087
    // 默认配置。0x0087 在表中保持 0，装载完成后再单独写 0x40 启动。
    function [7:0] default_config_byte;
        input [6:0] index;
        begin
            case (index)
                7'd0: default_config_byte=8'h00;
                7'd1: default_config_byte=IOVDD_2V8 ? 8'h01 : 8'h00;
                7'd2: default_config_byte=8'h00;  7'd3: default_config_byte=8'h01;
                7'd4: default_config_byte=8'h02;  7'd5: default_config_byte=8'h00;
                7'd6: default_config_byte=8'h02;  7'd7: default_config_byte=8'h08;
                7'd8: default_config_byte=8'h00;  7'd9: default_config_byte=8'h08;
                7'd10: default_config_byte=8'h10; 7'd11: default_config_byte=8'h01;
                7'd12: default_config_byte=8'h01; 7'd13: default_config_byte=8'h00;
                7'd14: default_config_byte=8'h00; 7'd15: default_config_byte=8'h00;
                7'd16: default_config_byte=8'h00; 7'd17: default_config_byte=8'hFF;
                7'd18: default_config_byte=8'h00; 7'd19: default_config_byte=8'h0F;
                7'd20: default_config_byte=8'h00; 7'd21: default_config_byte=8'h00;
                7'd22: default_config_byte=8'h00; 7'd23: default_config_byte=8'h00;
                7'd24: default_config_byte=8'h00; 7'd25: default_config_byte=8'h20;
                7'd26: default_config_byte=8'h0B; 7'd27: default_config_byte=8'h00;
                7'd28: default_config_byte=8'h00; 7'd29: default_config_byte=8'h02;
                7'd30: default_config_byte=8'h0A; 7'd31: default_config_byte=8'h21;
                7'd32: default_config_byte=8'h00; 7'd33: default_config_byte=8'h00;
                7'd34: default_config_byte=8'h05; 7'd35: default_config_byte=8'h00;
                7'd36: default_config_byte=8'h00; 7'd37: default_config_byte=8'h00;
                7'd38: default_config_byte=8'h00; 7'd39: default_config_byte=8'hC8;
                7'd40: default_config_byte=8'h00; 7'd41: default_config_byte=8'h00;
                7'd42: default_config_byte=8'h38; 7'd43: default_config_byte=8'hFF;
                7'd44: default_config_byte=8'h01; 7'd45: default_config_byte=8'h00;
                7'd46: default_config_byte=8'h08; 7'd47: default_config_byte=8'h00;
                7'd48: default_config_byte=8'h00; 7'd49: default_config_byte=8'h01;
                7'd50: default_config_byte=8'hCC; 7'd51: default_config_byte=8'h0F;
                7'd52: default_config_byte=8'h01; 7'd53: default_config_byte=8'hF1;
                7'd54: default_config_byte=8'h0D; 7'd55: default_config_byte=8'h01;
                7'd56: default_config_byte=8'h68; 7'd57: default_config_byte=8'h00;
                7'd58: default_config_byte=8'h80; 7'd59: default_config_byte=8'h08;
                7'd60: default_config_byte=8'hB8; 7'd61: default_config_byte=8'h00;
                7'd62: default_config_byte=8'h00; 7'd63: default_config_byte=8'h00;
                7'd64: default_config_byte=8'h00; 7'd65: default_config_byte=8'h0F;
                7'd66: default_config_byte=8'h89; 7'd67: default_config_byte=8'h00;
                7'd68: default_config_byte=8'h00; 7'd69: default_config_byte=8'h00;
                7'd70: default_config_byte=8'h00; 7'd71: default_config_byte=8'h00;
                7'd72: default_config_byte=8'h00; 7'd73: default_config_byte=8'h00;
                7'd74: default_config_byte=8'h00; 7'd75: default_config_byte=8'h01;
                7'd76: default_config_byte=8'h0F; 7'd77: default_config_byte=8'h0D;
                7'd78: default_config_byte=8'h0E; 7'd79: default_config_byte=8'h0E;
                7'd80: default_config_byte=8'h00; 7'd81: default_config_byte=8'h00;
                7'd82: default_config_byte=8'h02; 7'd83: default_config_byte=8'hC7;
                7'd84: default_config_byte=8'hFF; 7'd85: default_config_byte=8'h9B;
                7'd86: default_config_byte=8'h00; 7'd87: default_config_byte=8'h00;
                7'd88: default_config_byte=8'h00; 7'd89: default_config_byte=8'h01;
                7'd90: default_config_byte=8'h00;
                default: default_config_byte=8'h00;
            endcase
        end
    endfunction

    i2c_register_transaction16_vl53 #(
        .CLK_DIV(I2C_CLK_DIV)
    ) u_i2c_transaction (
        .clk(clk), .reset_n(reset_n), .start(txn_start),
        .device_addr(I2C_ADDR), .reg_addr(txn_reg_addr),
        .is_read(txn_read), .write_data(txn_write_data),
        .write_length(txn_write_length), .read_length(txn_read_length),
        .busy(txn_busy), .done(txn_done), .error(txn_error),
        .rx_valid(txn_rx_valid), .rx_index(txn_rx_index), .rx_data(txn_rx_data),
        .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_STARTUP_DELAY;
            init_index <= 7'd0;
            boot_retry_count <= 16'd0;
            calibration_retry_count <= 16'd0;
            calibration_step <= 3'd0;
            publish_pending <= 1'b0;
            delay_count <= 32'd0;
            txn_start <= 1'b0;
            txn_read <= 1'b0;
            txn_reg_addr <= 16'd0;
            txn_write_data <= 16'd0;
            txn_write_length <= 2'd0;
            txn_read_length <= 2'd0;
            initialized <= 1'b0;
            bus_error <= 1'b0;
            range_status <= 5'h1F;
            distance_mm <= 16'd0;
            distance_valid <= 1'b0;
            for (i=0; i<2; i=i+1) rx_bytes[i] <= 8'd0;
        end else begin
            txn_start <= 1'b0;
            distance_valid <= 1'b0;
            if (txn_rx_valid && txn_rx_index < 2)
                rx_bytes[txn_rx_index] <= txn_rx_data;

            case (state)
                ST_STARTUP_DELAY: begin
                    if (delay_count >= STARTUP_DELAY-1) begin
                        delay_count <= 0;
                        init_index <= 0;
                        boot_retry_count <= 0;
                        publish_pending <= 0;
                        state <= ST_INIT_START;
                    end else delay_count <= delay_count + 1'b1;
                end

                ST_INIT_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b0;
                        txn_write_length <= 2'd1;
                        txn_read_length <= 2'd0;
                        if (init_index == 7'd0) begin
                            txn_reg_addr <= REG_SOFT_RESET;
                            txn_write_data <= 16'h0000;
                        end else if (init_index == 7'd1) begin
                            txn_reg_addr <= REG_SOFT_RESET;
                            txn_write_data <= 16'h0001;
                        end else if (init_index == 7'd93) begin
                            txn_reg_addr <= REG_MODE_START;
                            txn_write_data <= 16'h0040;
                        end else begin
                            txn_reg_addr <= 16'h002D + (init_index - 7'd2);
                            txn_write_data <= {8'h00, default_config_byte(init_index - 7'd2)};
                        end
                        txn_start <= 1'b1;
                        state <= ST_INIT_WAIT;
                    end
                end

                ST_INIT_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin
                            bus_error <= 1'b1;
                            initialized <= 1'b0;
                            init_index <= 7'd0;
                            delay_count <= 0;
                            state <= ST_STARTUP_DELAY;
                        end else if (init_index == 7'd0 || init_index == 7'd1) begin
                            init_index <= init_index + 1'b1;
                            delay_count <= 0;
                            state <= ST_RESET_DELAY;
                        end else if (init_index == 7'd93) begin
                            calibration_step <= 0;
                            calibration_retry_count <= 0;
                            state <= ST_CAL_READY_START;
                        end else begin
                            init_index <= init_index + 1'b1;
                            state <= ST_INIT_START;
                        end
                    end
                end

                ST_RESET_DELAY: begin
                    if (delay_count >= RESET_DELAY-1) begin
                        delay_count <= 0;
                        if (init_index == 7'd2)
                            state <= ST_BOOT_START;
                        else
                            state <= ST_INIT_START;
                    end else delay_count <= delay_count + 1'b1;
                end

                // ST ULD 在软复位释放后轮询 FIRMWARE__SYSTEM_STATUS(0x00E5)，
                // 避免仅靠固定 1 ms 延时在慢角器件上提前访问配置寄存器。
                ST_BOOT_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg_addr <= REG_BOOT_STATE;
                        txn_write_length <= 0;
                        txn_read_length <= 1;
                        txn_start <= 1'b1;
                        state <= ST_BOOT_WAIT;
                    end
                end

                ST_BOOT_WAIT: begin
                    if (txn_done) begin
                        if (!txn_error && rx_bytes[0][0]) begin
                            boot_retry_count <= 0;
                            state <= ST_INIT_START;
                        end else if (boot_retry_count >= BOOT_RETRIES-1) begin
                            bus_error <= 1'b1;
                            initialized <= 1'b0;
                            init_index <= 0;
                            delay_count <= 0;
                            state <= ST_STARTUP_DELAY;
                        end else begin
                            boot_retry_count <= boot_retry_count + 1'b1;
                            delay_count <= 0;
                            state <= ST_RESET_DELAY;
                        end
                    end
                end

                // ULD SensorInit 的首次测量收尾：等待 ready、清中断、停止，
                // 写 0x0008=0x09/0x000B=0，随后重新启动连续测距。
                ST_CAL_READY_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg_addr <= REG_GPIO_STATUS;
                        txn_write_length <= 0;
                        txn_read_length <= 1;
                        txn_start <= 1'b1;
                        state <= ST_CAL_READY_WAIT;
                    end
                end

                ST_CAL_READY_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin
                            bus_error <= 1'b1;
                            initialized <= 1'b0;
                            init_index <= 0;
                            delay_count <= 0;
                            state <= ST_STARTUP_DELAY;
                        end else if (rx_bytes[0][0]) begin
                            calibration_retry_count <= 0;
                            calibration_step <= 0;
                            state <= ST_CAL_WRITE_START;
                        end else if (calibration_retry_count >= CALIBRATION_READY_RETRIES-1) begin
                            bus_error <= 1'b1;
                            initialized <= 1'b0;
                            init_index <= 0;
                            delay_count <= 0;
                            state <= ST_STARTUP_DELAY;
                        end else begin
                            calibration_retry_count <= calibration_retry_count + 1'b1;
                            state <= ST_CAL_READY_START;
                        end
                    end
                end

                ST_CAL_WRITE_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b0;
                        txn_write_length <= 1;
                        txn_read_length <= 0;
                        case (calibration_step)
                            3'd0: begin txn_reg_addr<=REG_INTERRUPT_CLEAR; txn_write_data<=16'h0001; end
                            3'd1: begin txn_reg_addr<=REG_MODE_START;      txn_write_data<=16'h0000; end
                            3'd2: begin txn_reg_addr<=REG_VHV_TIMEOUT;     txn_write_data<=16'h0009; end
                            3'd3: begin txn_reg_addr<=REG_VHV_INIT;        txn_write_data<=16'h0000; end
                            default: begin txn_reg_addr<=REG_MODE_START;   txn_write_data<=16'h0040; end
                        endcase
                        txn_start <= 1'b1;
                        state <= ST_CAL_WRITE_WAIT;
                    end
                end

                ST_CAL_WRITE_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin
                            bus_error <= 1'b1;
                            initialized <= 1'b0;
                            init_index <= 0;
                            delay_count <= 0;
                            state <= ST_STARTUP_DELAY;
                        end else if (calibration_step == 3'd4) begin
                            initialized <= 1'b1;
                            delay_count <= 0;
                            state <= ST_SAMPLE_DELAY;
                        end else begin
                            calibration_step <= calibration_step + 1'b1;
                            state <= ST_CAL_WRITE_START;
                        end
                    end
                end

                ST_SAMPLE_DELAY: begin
                    if (delay_count >= SAMPLE_INTERVAL-1) begin
                        delay_count <= 0;
                        state <= ST_STATUS_START;
                    end else delay_count <= delay_count + 1'b1;
                end

                ST_STATUS_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg_addr <= REG_GPIO_STATUS;
                        txn_write_length <= 0;
                        txn_read_length <= 1;
                        txn_start <= 1'b1;
                        state <= ST_STATUS_WAIT;
                    end
                end

                ST_STATUS_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin bus_error<=1'b1; state<=ST_SAMPLE_DELAY; end
                        else if (rx_bytes[0][0]) state <= ST_RANGE_STATUS_START;
                        else state <= ST_SAMPLE_DELAY;
                    end
                end

                ST_RANGE_STATUS_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg_addr <= REG_RANGE_STATUS;
                        txn_write_length <= 0;
                        txn_read_length <= 1;
                        txn_start <= 1'b1;
                        state <= ST_RANGE_STATUS_WAIT;
                    end
                end

                ST_RANGE_STATUS_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin
                            bus_error <= 1'b1;
                            state <= ST_SAMPLE_DELAY;
                        end else begin
                            range_status <= rx_bytes[0][4:0];
                            publish_pending <= 1'b0;
                            // ST ULD 映射表中 raw range status 9 对应有效测距。
                            if (rx_bytes[0][4:0] == 5'd9)
                                state <= ST_RANGE_START;
                            else
                                state <= ST_CLEAR_START;
                        end
                    end
                end

                ST_RANGE_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg_addr <= REG_DISTANCE_MM;
                        txn_read_length <= 2;
                        txn_start <= 1'b1;
                        state <= ST_RANGE_WAIT;
                    end
                end

                ST_RANGE_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin bus_error<=1'b1; state<=ST_SAMPLE_DELAY; end
                        else begin publish_pending<=1'b1; state<=ST_CLEAR_START; end
                    end
                end

                ST_CLEAR_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b0;
                        txn_reg_addr <= REG_INTERRUPT_CLEAR;
                        txn_write_data <= 16'h0001;
                        txn_write_length <= 1;
                        txn_read_length <= 0;
                        txn_start <= 1'b1;
                        state <= ST_CLEAR_WAIT;
                    end
                end

                ST_CLEAR_WAIT: begin
                    if (txn_done) begin
                        if (txn_error) begin
                            bus_error <= 1'b1;
                            publish_pending <= 1'b0;
                            delay_count <= 0;
                            state <= ST_SAMPLE_DELAY;
                        end else if (publish_pending) begin
                            state <= ST_PUBLISH;
                        end else begin
                            delay_count <= 0;
                            state <= ST_SAMPLE_DELAY;
                        end
                    end
                end

                ST_PUBLISH: begin
                    distance_mm <= {rx_bytes[0],rx_bytes[1]};
                    distance_valid <= 1'b1;
                    publish_pending <= 1'b0;
                    delay_count <= 0;
                    state <= ST_SAMPLE_DELAY;
                end

                default: state <= ST_STARTUP_DELAY;
            endcase
        end
    end
endmodule

// 16 位寄存器地址 I2C 事务层，支持 1/2 字节写和带重复起始的 1/2 字节读。
module i2c_register_transaction16_vl53 #(
    parameter integer CLK_DIV = 32
) (
    input wire clk, input wire reset_n, input wire start,
    input wire [6:0] device_addr, input wire [15:0] reg_addr,
    input wire is_read, input wire [15:0] write_data,
    input wire [1:0] write_length, input wire [1:0] read_length,
    output reg busy, output reg done, output reg error,
    output reg rx_valid, output reg [1:0] rx_index, output reg [7:0] rx_data,
    inout wire i2c_scl, inout wire i2c_sda
);
    localparam [4:0] S_IDLE=0,S_START=1,S_START_W=2,S_ADDRW=3,S_ADDRW_W=4,
        S_REGH=5,S_REGH_W=6,S_REGL=7,S_REGL_W=8,S_WDATA=9,S_WDATA_W=10,
        S_RSTART=11,S_RSTART_W=12,S_ADDRR=13,S_ADDRR_W=14,S_RDATA=15,
        S_RDATA_W=16,S_STOP=17,S_STOP_W=18,S_FINISH=19;
    localparam [2:0] CMD_START=0,CMD_STOP=1,CMD_WRITE=2,CMD_READ_ACK=3,CMD_READ_NACK=4;
    reg [4:0] state;
    reg phy_start;
    reg [2:0] phy_cmd;
    reg [7:0] phy_tx;
    wire phy_busy, phy_done, phy_ack_error;
    wire [7:0] phy_rx;
    reg [6:0] job_addr;
    reg [15:0] job_reg, job_wdata;
    reg job_read;
    reg [1:0] job_wlen, job_rlen, byte_index;

    i2c_byte_engine_vl53 #(.CLK_DIV(CLK_DIV)) u_phy (
        .clk(clk),.reset_n(reset_n),.cmd_start(phy_start),.cmd(phy_cmd),.tx_data(phy_tx),
        .busy(phy_busy),.done(phy_done),.ack_error(phy_ack_error),.rx_data(phy_rx),
        .i2c_scl(i2c_scl),.i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=S_IDLE; phy_start<=0; phy_cmd<=0; phy_tx<=0; busy<=0; done<=0;
            error<=0; rx_valid<=0; rx_index<=0; rx_data<=0; job_addr<=0; job_reg<=0;
            job_wdata<=0; job_read<=0; job_wlen<=0; job_rlen<=0; byte_index<=0;
        end else begin
            phy_start<=0; done<=0; rx_valid<=0;
            case(state)
                S_IDLE: begin
                    busy<=0;
                    if(start) begin
                        busy<=1; error<=0; job_addr<=device_addr; job_reg<=reg_addr;
                        job_wdata<=write_data; job_read<=is_read; job_wlen<=write_length;
                        job_rlen<=read_length; byte_index<=0; state<=S_START;
                    end
                end
                S_START: if(!phy_busy) begin phy_cmd<=CMD_START; phy_start<=1; state<=S_START_W; end
                S_START_W: if(phy_done) begin if(phy_ack_error) begin error<=1; state<=S_FINISH; end else state<=S_ADDRW; end
                S_ADDRW: if(!phy_busy) begin phy_cmd<=CMD_WRITE; phy_tx<={job_addr,1'b0}; phy_start<=1; state<=S_ADDRW_W; end
                S_ADDRW_W: if(phy_done) begin if(phy_ack_error) error<=1; state<=S_REGH; end
                S_REGH: if(!phy_busy) begin phy_cmd<=CMD_WRITE; phy_tx<=job_reg[15:8]; phy_start<=1; state<=S_REGH_W; end
                S_REGH_W: if(phy_done) begin if(phy_ack_error) error<=1; state<=S_REGL; end
                S_REGL: if(!phy_busy) begin phy_cmd<=CMD_WRITE; phy_tx<=job_reg[7:0]; phy_start<=1; state<=S_REGL_W; end
                S_REGL_W: if(phy_done) begin
                    if(phy_ack_error) error<=1;
                    byte_index<=0;
                    state<=job_read?S_RSTART:S_WDATA;
                end
                S_WDATA: if(!phy_busy) begin
                    phy_cmd<=CMD_WRITE;
                    phy_tx<=(job_wlen==2 && byte_index==0)?job_wdata[15:8]:job_wdata[7:0];
                    phy_start<=1; state<=S_WDATA_W;
                end
                S_WDATA_W: if(phy_done) begin
                    if(phy_ack_error) error<=1;
                    if(byte_index+1'b1>=job_wlen) state<=S_STOP;
                    else begin byte_index<=byte_index+1'b1; state<=S_WDATA; end
                end
                S_RSTART: if(!phy_busy) begin phy_cmd<=CMD_START; phy_start<=1; state<=S_RSTART_W; end
                S_RSTART_W: if(phy_done) begin if(phy_ack_error) begin error<=1; state<=S_FINISH; end else state<=S_ADDRR; end
                S_ADDRR: if(!phy_busy) begin phy_cmd<=CMD_WRITE; phy_tx<={job_addr,1'b1}; phy_start<=1; state<=S_ADDRR_W; end
                S_ADDRR_W: if(phy_done) begin if(phy_ack_error) error<=1; byte_index<=0; state<=S_RDATA; end
                S_RDATA: if(!phy_busy) begin
                    phy_cmd<=(byte_index+1'b1>=job_rlen)?CMD_READ_NACK:CMD_READ_ACK;
                    phy_start<=1; state<=S_RDATA_W;
                end
                S_RDATA_W: if(phy_done) begin
                    if(phy_ack_error) begin
                        error<=1;
                        state<=S_STOP;
                    end else begin
                        rx_valid<=1; rx_index<=byte_index; rx_data<=phy_rx;
                        if(byte_index+1'b1>=job_rlen) state<=S_STOP;
                        else begin byte_index<=byte_index+1'b1; state<=S_RDATA; end
                    end
                end
                S_STOP: if(!phy_busy) begin phy_cmd<=CMD_STOP; phy_start<=1; state<=S_STOP_W; end
                S_STOP_W: if(phy_done) begin if(phy_ack_error) error<=1; state<=S_FINISH; end
                S_FINISH: begin busy<=0; done<=1; state<=S_IDLE; end
                default: state<=S_IDLE;
            endcase
        end
    end
endmodule

// 开漏 I2C 物理层。释放 SCL 后会等待线上电平变高，因而支持时钟拉伸。
module i2c_byte_engine_vl53 #(
    parameter integer CLK_DIV = 32,
    parameter integer STRETCH_TIMEOUT_TICKS = 8192
) (
    input wire clk,input wire reset_n,input wire cmd_start,input wire [2:0] cmd,
    input wire [7:0] tx_data,output reg busy,output reg done,output reg ack_error,
    output reg [7:0] rx_data,inout wire i2c_scl,inout wire i2c_sda
);
    localparam [2:0] C_START=0,C_STOP=1,C_WRITE=2,C_READ_ACK=3,C_READ_NACK=4;
    localparam [4:0] P_IDLE=0,P_START_SETUP=1,P_START_RAISE=2,P_START_FALL=3,
      P_STOP_SETUP=4,P_STOP_RAISE=5,P_STOP_RELEASE=6,P_W_SETUP=7,P_W_RAISE=8,
      P_W_HIGH=9,P_W_FALL=10,P_ACK_SETUP=11,P_ACK_RAISE=12,P_ACK_HIGH=13,
      P_ACK_FALL=14,P_R_SETUP=15,P_R_RAISE=16,P_R_HIGH=17,P_R_FALL=18,
      P_MACK_SETUP=19,P_MACK_RAISE=20,P_MACK_HIGH=21,P_MACK_FALL=22,P_DONE=23;
    localparam integer SAFE_CLK_DIV=(CLK_DIV<1)?1:CLK_DIV;
    localparam integer STRETCH_LIMIT=(STRETCH_TIMEOUT_TICKS<1)?1:STRETCH_TIMEOUT_TICKS;
    reg [4:0] state;
    reg [31:0] div_count;
    reg [31:0] stretch_count;
    reg scl_low,sda_low;
    reg [2:0] bit_index;
    reg [7:0] tx_latched,rx_shift;
    reg [2:0] job_cmd;
    wire scl_in=i2c_scl;
    wire sda_in=i2c_sda;
    assign i2c_scl=scl_low?1'b0:1'bz;
    assign i2c_sda=sda_low?1'b0:1'bz;

    always @(posedge clk or negedge reset_n) begin
      if(!reset_n) begin
        state<=P_IDLE;div_count<=0;stretch_count<=0;scl_low<=0;sda_low<=0;bit_index<=0;
        tx_latched<=0;rx_shift<=0;job_cmd<=0;busy<=0;done<=0;ack_error<=0;rx_data<=0;
      end else begin
        done<=0;
        if(state==P_IDLE) begin
          div_count<=0;stretch_count<=0;busy<=0;
          if(cmd_start) begin
            busy<=1;ack_error<=0;job_cmd<=cmd;tx_latched<=tx_data;bit_index<=7;rx_shift<=0;
            case(cmd)
              C_START: state<=P_START_SETUP;
              C_STOP: state<=P_STOP_SETUP;
              C_WRITE: state<=P_W_SETUP;
              default: state<=P_R_SETUP;
            endcase
          end
        end else if(div_count>=SAFE_CLK_DIV-1) begin
          div_count<=0;
          case(state)
            P_START_SETUP: begin scl_low<=1;sda_low<=0;state<=P_START_RAISE;end
            P_START_RAISE: begin
              scl_low<=0;
              if(scl_in===1'b1 && sda_in===1'b1) begin stretch_count<=0;state<=P_START_FALL;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;
            end
            P_START_FALL: begin sda_low<=1;scl_low<=0;state<=P_W_FALL;end
            P_STOP_SETUP: begin scl_low<=1;sda_low<=1;state<=P_STOP_RAISE;end
            P_STOP_RAISE: begin scl_low<=0;if(scl_in===1'b1) begin stretch_count<=0;state<=P_STOP_RELEASE;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;end
            P_STOP_RELEASE: begin sda_low<=0;state<=P_DONE;end
            P_W_SETUP: begin scl_low<=1;sda_low<=~tx_latched[bit_index];state<=P_W_RAISE;end
            P_W_RAISE: begin scl_low<=0;if(scl_in===1'b1) begin stretch_count<=0;state<=P_W_HIGH;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;end
            P_W_HIGH: state<=P_W_FALL;
            P_W_FALL: begin
              scl_low<=1;
              if(job_cmd==C_START) begin job_cmd<=C_WRITE;bit_index<=7;state<=P_DONE;end
              else if(bit_index==0) state<=P_ACK_SETUP;
              else begin bit_index<=bit_index-1'b1;state<=P_W_SETUP;end
            end
            P_ACK_SETUP: begin scl_low<=1;sda_low<=0;state<=P_ACK_RAISE;end
            P_ACK_RAISE: begin scl_low<=0;if(scl_in===1'b1) begin stretch_count<=0;state<=P_ACK_HIGH;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;end
            P_ACK_HIGH: begin ack_error<=(sda_in!==1'b0);state<=P_ACK_FALL;end
            P_ACK_FALL: begin scl_low<=1;state<=P_DONE;end
            P_R_SETUP: begin scl_low<=1;sda_low<=0;state<=P_R_RAISE;end
            P_R_RAISE: begin scl_low<=0;if(scl_in===1'b1) begin stretch_count<=0;state<=P_R_HIGH;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;end
            P_R_HIGH: begin rx_shift[bit_index]<=sda_in;state<=P_R_FALL;end
            P_R_FALL: begin
              scl_low<=1;
              if(bit_index==0) state<=P_MACK_SETUP;
              else begin bit_index<=bit_index-1'b1;state<=P_R_SETUP;end
            end
            P_MACK_SETUP: begin scl_low<=1;sda_low<=(job_cmd==C_READ_ACK);state<=P_MACK_RAISE;end
            P_MACK_RAISE: begin scl_low<=0;if(scl_in===1'b1) begin stretch_count<=0;state<=P_MACK_HIGH;end
              else if(stretch_count>=STRETCH_LIMIT-1) begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
              else stretch_count<=stretch_count+1'b1;end
            P_MACK_HIGH: state<=P_MACK_FALL;
            P_MACK_FALL: begin scl_low<=1;sda_low<=0;rx_data<=rx_shift;state<=P_DONE;end
            P_DONE: begin busy<=0;done<=1;state<=P_IDLE;end
            default: state<=P_IDLE;
          endcase
        end else div_count<=div_count+1'b1;
      end
    end
endmodule

`timescale 1ns/1ps

// 通用 7 位地址、8 位寄存器地址 I2C 主机。
// - 开漏 SCL/SDA，不在 FPGA 内部伪造上拉；板上必须有与 I/O 电压匹配的上拉电阻。
// - 支持时钟拉伸与总线超时。
// - 写 1..4 字节、读 1..31 字节，均按寄存器自增顺序传输。
// - 一次事务只在 done 脉冲到来时结束；error 表示 NACK 或 SCL 拉伸超时。
module i2c_register8_master #(
    parameter integer CLK_DIV = 32,
    parameter integer STRETCH_TIMEOUT_TICKS = 8192
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        start,
    input  wire [6:0]  device_addr,
    input  wire [7:0]  reg_addr,
    input  wire        is_read,
    input  wire [31:0] write_data,
    input  wire [2:0]  write_length,
    input  wire [4:0]  read_length,
    output reg         busy,
    output reg         done,
    output reg         error,
    output reg         rx_valid,
    output reg  [4:0]  rx_index,
    output reg  [7:0]  rx_data,
    inout  wire        i2c_scl,
    inout  wire        i2c_sda
);
    localparam [4:0] S_IDLE       = 5'd0;
    localparam [4:0] S_START      = 5'd1;
    localparam [4:0] S_START_W    = 5'd2;
    localparam [4:0] S_ADDR_W     = 5'd3;
    localparam [4:0] S_ADDR_W_W   = 5'd4;
    localparam [4:0] S_REG        = 5'd5;
    localparam [4:0] S_REG_W      = 5'd6;
    localparam [4:0] S_WRITE      = 5'd7;
    localparam [4:0] S_WRITE_W    = 5'd8;
    localparam [4:0] S_RESTART    = 5'd9;
    localparam [4:0] S_RESTART_W  = 5'd10;
    localparam [4:0] S_ADDR_R     = 5'd11;
    localparam [4:0] S_ADDR_R_W   = 5'd12;
    localparam [4:0] S_READ       = 5'd13;
    localparam [4:0] S_READ_W     = 5'd14;
    localparam [4:0] S_STOP       = 5'd15;
    localparam [4:0] S_STOP_W     = 5'd16;
    localparam [4:0] S_FINISH     = 5'd17;

    localparam [2:0] CMD_START     = 3'd0;
    localparam [2:0] CMD_STOP      = 3'd1;
    localparam [2:0] CMD_WRITE     = 3'd2;
    localparam [2:0] CMD_READ_ACK  = 3'd3;
    localparam [2:0] CMD_READ_NACK = 3'd4;

    reg [4:0] state;
    reg       phy_start;
    reg [2:0] phy_cmd;
    reg [7:0] phy_tx;
    wire      phy_busy;
    wire      phy_done;
    wire      phy_error;
    wire      phy_nack_unused;
    wire [7:0] phy_rx;

    reg [6:0]  job_addr;
    reg [7:0]  job_reg;
    reg        job_read;
    reg [31:0] job_write_data;
    reg [2:0]  job_write_length;
    reg [4:0]  job_read_length;
    reg [4:0]  byte_index;

    function [7:0] select_write_byte;
        input [31:0] value;
        input [2:0]  length;
        input [4:0]  index;
        begin
            case (length)
                3'd1: select_write_byte = value[7:0];
                3'd2: select_write_byte = index[0] ? value[7:0] : value[15:8];
                3'd3: begin
                    case (index)
                        5'd0: select_write_byte = value[23:16];
                        5'd1: select_write_byte = value[15:8];
                        default: select_write_byte = value[7:0];
                    endcase
                end
                default: begin
                    case (index)
                        5'd0: select_write_byte = value[31:24];
                        5'd1: select_write_byte = value[23:16];
                        5'd2: select_write_byte = value[15:8];
                        default: select_write_byte = value[7:0];
                    endcase
                end
            endcase
        end
    endfunction

    i2c_open_drain_byte_engine #(
        .CLK_DIV(CLK_DIV),
        .STRETCH_TIMEOUT_TICKS(STRETCH_TIMEOUT_TICKS)
    ) u_phy (
        .clk(clk), .reset_n(reset_n),
        .cmd_start(phy_start), .cmd(phy_cmd), .tx_data(phy_tx),
        .busy(phy_busy), .done(phy_done), .error(phy_error),
        .nack(phy_nack_unused), .rx_data(phy_rx),
        .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE;
            phy_start <= 1'b0;
            phy_cmd <= CMD_START;
            phy_tx <= 8'd0;
            busy <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
            rx_valid <= 1'b0;
            rx_index <= 5'd0;
            rx_data <= 8'd0;
            job_addr <= 7'd0;
            job_reg <= 8'd0;
            job_read <= 1'b0;
            job_write_data <= 32'd0;
            job_write_length <= 3'd0;
            job_read_length <= 5'd0;
            byte_index <= 5'd0;
        end else begin
            phy_start <= 1'b0;
            done <= 1'b0;
            rx_valid <= 1'b0;
            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        error <= 1'b0;
                        job_addr <= device_addr;
                        job_reg <= reg_addr;
                        job_read <= is_read;
                        job_write_data <= write_data;
                        job_write_length <= write_length;
                        job_read_length <= read_length;
                        byte_index <= 5'd0;
                        state <= S_START;
                    end
                end
                S_START: if (!phy_busy) begin
                    phy_cmd <= CMD_START; phy_start <= 1'b1; state <= S_START_W;
                end
                S_START_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else state <= S_ADDR_W;
                end
                S_ADDR_W: if (!phy_busy) begin
                    phy_cmd <= CMD_WRITE; phy_tx <= {job_addr, 1'b0};
                    phy_start <= 1'b1; state <= S_ADDR_W_W;
                end
                S_ADDR_W_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else state <= S_REG;
                end
                S_REG: if (!phy_busy) begin
                    phy_cmd <= CMD_WRITE; phy_tx <= job_reg;
                    phy_start <= 1'b1; state <= S_REG_W;
                end
                S_REG_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else begin
                        byte_index <= 5'd0;
                        state <= job_read ? S_RESTART : S_WRITE;
                    end
                end
                S_WRITE: if (!phy_busy) begin
                    if (job_write_length == 0) state <= S_STOP;
                    else begin
                        phy_cmd <= CMD_WRITE;
                        phy_tx <= select_write_byte(job_write_data, job_write_length, byte_index);
                        phy_start <= 1'b1;
                        state <= S_WRITE_W;
                    end
                end
                S_WRITE_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else if (byte_index + 1'b1 >= job_write_length) state <= S_STOP;
                    else begin byte_index <= byte_index + 1'b1; state <= S_WRITE; end
                end
                S_RESTART: if (!phy_busy) begin
                    phy_cmd <= CMD_START; phy_start <= 1'b1; state <= S_RESTART_W;
                end
                S_RESTART_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else state <= S_ADDR_R;
                end
                S_ADDR_R: if (!phy_busy) begin
                    phy_cmd <= CMD_WRITE; phy_tx <= {job_addr, 1'b1};
                    phy_start <= 1'b1; state <= S_ADDR_R_W;
                end
                S_ADDR_R_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else begin byte_index <= 5'd0; state <= S_READ; end
                end
                S_READ: if (!phy_busy) begin
                    if (job_read_length == 0) state <= S_STOP;
                    else begin
                        phy_cmd <= (byte_index + 1'b1 >= job_read_length) ?
                                   CMD_READ_NACK : CMD_READ_ACK;
                        phy_start <= 1'b1;
                        state <= S_READ_W;
                    end
                end
                S_READ_W: if (phy_done) begin
                    if (phy_error) begin error <= 1'b1; state <= S_STOP; end
                    else begin
                        rx_valid <= 1'b1;
                        rx_index <= byte_index;
                        rx_data <= phy_rx;
                        if (byte_index + 1'b1 >= job_read_length) state <= S_STOP;
                        else begin byte_index <= byte_index + 1'b1; state <= S_READ; end
                    end
                end
                S_STOP: if (!phy_busy) begin
                    phy_cmd <= CMD_STOP; phy_start <= 1'b1; state <= S_STOP_W;
                end
                S_STOP_W: if (phy_done) begin
                    if (phy_error) error <= 1'b1;
                    state <= S_FINISH;
                end
                S_FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

// 单字节开漏物理层。命令完成时 done 脉冲一个 clk 周期。
module i2c_open_drain_byte_engine #(
    parameter integer CLK_DIV = 32,
    parameter integer STRETCH_TIMEOUT_TICKS = 8192
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       cmd_start,
    input  wire [2:0] cmd,
    input  wire [7:0] tx_data,
    output reg        busy,
    output reg        done,
    output reg        error,
    // 仅表示写字节第九位未被拉低；error 还可能表示 SCL 超时。
    // 普通 I2C 上层应继续把 nack 当错误，SCCB 上层可按规范忽略该位。
    output reg        nack,
    output reg [7:0]  rx_data,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);
    localparam [2:0] C_START=3'd0, C_STOP=3'd1, C_WRITE=3'd2,
                     C_READ_ACK=3'd3, C_READ_NACK=3'd4;
    localparam [4:0] P_IDLE=5'd0, P_START_SETUP=5'd1, P_START_RAISE=5'd2,
      P_START_FALL=5'd3, P_STOP_SETUP=5'd4, P_STOP_RAISE=5'd5,
      P_STOP_RELEASE=5'd6, P_W_SETUP=5'd7, P_W_RAISE=5'd8,
      P_W_HIGH=5'd9, P_W_FALL=5'd10, P_ACK_SETUP=5'd11,
      P_ACK_RAISE=5'd12, P_ACK_HIGH=5'd13, P_ACK_FALL=5'd14,
      P_R_SETUP=5'd15, P_R_RAISE=5'd16, P_R_HIGH=5'd17,
      P_R_FALL=5'd18, P_MACK_SETUP=5'd19, P_MACK_RAISE=5'd20,
      P_MACK_HIGH=5'd21, P_MACK_FALL=5'd22, P_DONE=5'd23;
    localparam integer SAFE_DIV = (CLK_DIV < 1) ? 1 : CLK_DIV;
    localparam integer STRETCH_LIMIT =
        (STRETCH_TIMEOUT_TICKS < 1) ? 1 : STRETCH_TIMEOUT_TICKS;

    reg [4:0] state;
    reg [31:0] div_count;
    reg [31:0] stretch_count;
    reg scl_low;
    reg sda_low;
    reg [2:0] bit_index;
    reg [7:0] tx_latched;
    reg [7:0] rx_shift;
    reg [2:0] job_cmd;
    wire scl_in = i2c_scl;
    wire sda_in = i2c_sda;

    assign i2c_scl = scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_low ? 1'b0 : 1'bz;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= P_IDLE; div_count <= 0; stretch_count <= 0;
            scl_low <= 0; sda_low <= 0; bit_index <= 0;
            tx_latched <= 0; rx_shift <= 0; job_cmd <= 0;
            busy <= 0; done <= 0; error <= 0; nack <= 0; rx_data <= 0;
        end else begin
            done <= 1'b0;
            if (state == P_IDLE) begin
                div_count <= 0;
                stretch_count <= 0;
                busy <= 0;
                if (cmd_start) begin
                    busy <= 1;
                    error <= 0;
                    nack <= 0;
                    job_cmd <= cmd;
                    tx_latched <= tx_data;
                    bit_index <= 3'd7;
                    rx_shift <= 0;
                    case (cmd)
                        C_START: state <= P_START_SETUP;
                        C_STOP: state <= P_STOP_SETUP;
                        C_WRITE: state <= P_W_SETUP;
                        default: state <= P_R_SETUP;
                    endcase
                end
            end else if (div_count >= SAFE_DIV-1) begin
                div_count <= 0;
                case (state)
                    P_START_SETUP: begin scl_low<=1; sda_low<=0; state<=P_START_RAISE; end
                    P_START_RAISE: begin
                        scl_low <= 0;
                        if (scl_in === 1'b1 && sda_in === 1'b1) begin
                            stretch_count <= 0; state <= P_START_FALL;
                        end else if (stretch_count >= STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count <= stretch_count + 1'b1;
                    end
                    P_START_FALL: begin sda_low<=1; scl_low<=0; state<=P_W_FALL; end
                    P_STOP_SETUP: begin scl_low<=1; sda_low<=1; state<=P_STOP_RAISE; end
                    P_STOP_RAISE: begin
                        scl_low <= 0;
                        if (scl_in === 1'b1) begin stretch_count<=0; state<=P_STOP_RELEASE; end
                        else if (stretch_count >= STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count<=stretch_count+1'b1;
                    end
                    P_STOP_RELEASE: begin sda_low<=0; state<=P_DONE; end
                    P_W_SETUP: begin scl_low<=1; sda_low<=~tx_latched[bit_index]; state<=P_W_RAISE; end
                    P_W_RAISE: begin
                        scl_low <= 0;
                        if (scl_in === 1'b1) begin stretch_count<=0; state<=P_W_HIGH; end
                        else if (stretch_count >= STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count<=stretch_count+1'b1;
                    end
                    P_W_HIGH: state<=P_W_FALL;
                    P_W_FALL: begin
                        scl_low<=1;
                        if (job_cmd==C_START) begin job_cmd<=C_WRITE; bit_index<=7; state<=P_DONE; end
                        else if (bit_index==0) state<=P_ACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=P_W_SETUP; end
                    end
                    P_ACK_SETUP: begin scl_low<=1; sda_low<=0; state<=P_ACK_RAISE; end
                    P_ACK_RAISE: begin
                        scl_low<=0;
                        if (scl_in===1'b1) begin stretch_count<=0; state<=P_ACK_HIGH; end
                        else if (stretch_count>=STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count<=stretch_count+1'b1;
                    end
                    P_ACK_HIGH: begin
                        nack <= (sda_in !== 1'b0);
                        error <= (sda_in !== 1'b0);
                        state<=P_ACK_FALL;
                    end
                    P_ACK_FALL: begin scl_low<=1; state<=P_DONE; end
                    P_R_SETUP: begin scl_low<=1; sda_low<=0; state<=P_R_RAISE; end
                    P_R_RAISE: begin
                        scl_low<=0;
                        if (scl_in===1'b1) begin stretch_count<=0; state<=P_R_HIGH; end
                        else if (stretch_count>=STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count<=stretch_count+1'b1;
                    end
                    P_R_HIGH: begin rx_shift[bit_index]<=sda_in; state<=P_R_FALL; end
                    P_R_FALL: begin
                        scl_low<=1;
                        if (bit_index==0) state<=P_MACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=P_R_SETUP; end
                    end
                    P_MACK_SETUP: begin
                        scl_low<=1; sda_low<=(job_cmd==C_READ_ACK); state<=P_MACK_RAISE;
                    end
                    P_MACK_RAISE: begin
                        scl_low<=0;
                        if (scl_in===1'b1) begin stretch_count<=0; state<=P_MACK_HIGH; end
                        else if (stretch_count>=STRETCH_LIMIT-1) begin
                            error<=1; scl_low<=0; sda_low<=0; stretch_count<=0; state<=P_DONE;
                        end else stretch_count<=stretch_count+1'b1;
                    end
                    P_MACK_HIGH: state<=P_MACK_FALL;
                    P_MACK_FALL: begin scl_low<=1; sda_low<=0; rx_data<=rx_shift; state<=P_DONE; end
                    P_DONE: begin busy<=0; done<=1; state<=P_IDLE; end
                    default: state<=P_IDLE;
                endcase
            end else div_count <= div_count + 1'b1;
        end
    end
endmodule

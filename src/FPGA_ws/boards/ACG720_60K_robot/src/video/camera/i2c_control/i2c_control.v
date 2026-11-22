/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_control
// Description   : I2C控制器顶层文件
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module i2c_control #(
    parameter SYS_CLOCK = 50_000_000,
    parameter SCL_CLOCK = 400_000
) (
    input wire Clk, Rst_n, wrreg_req, rdreg_req,
    input wire [15:0] addr,
    input wire addr_mode,
    input wire [7:0] wrdata,
    output reg [7:0] rddata,
    input wire [7:0] device_id,
    output reg RW_Done, ack,
    input wire [31:0] dly_cnt_max,
    output wire i2c_sclk,
    inout wire i2c_sdat
);
    localparam [2:0] IDLE = 0, ISSUE = 1, WAIT_BYTE = 2, FINISH = 3, DELAY = 4;
    localparam [5:0] WR = 1, STA = 2, RD = 4, STO = 8, NACK = 32;
    reg [2:0] state, byte_index;
    reg reading, go;
    reg [5:0] command;
    reg [7:0] tx_byte;
    wire [7:0] rx_byte;
    wire byte_done, byte_nack;
    reg [31:0] delay_count;
    wire [15:0] reg_addr = addr_mode ? addr : {addr[7:0], addr[15:8]};

    i2c_bit_shift #(.SYS_CLOCK(SYS_CLOCK), .SCL_CLOCK(SCL_CLOCK)) byte_engine (
        .Clk(Clk), .Rst_n(Rst_n), .Cmd(command), .Go(go),
        .Rx_DATA(rx_byte), .Tx_DATA(tx_byte), .Trans_Done(byte_done),
        .ack_o(byte_nack), .i2c_sclk(i2c_sclk), .i2c_sdat(i2c_sdat)
    );

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            state <= IDLE;
            byte_index <= 0;
            reading <= 0;
            command <= 0;
            tx_byte <= 0;
            go <= 0;
            rddata <= 0;
            RW_Done <= 0;
            ack <= 0;
            delay_count <= 0;
        end else begin
            go <= 0;
            case (state)
                IDLE: begin
                    byte_index <= 0;
                    delay_count <= 0;
                    ack <= 0;
                    RW_Done <= 0;
                    if (wrreg_req || rdreg_req) begin
                        reading <= !wrreg_req;
                        state <= ISSUE;
                    end
                end
                ISSUE: begin
                    go <= 1;
                    state <= WAIT_BYTE;
                    case (byte_index)
                        0: begin command <= WR | STA; tx_byte <= device_id; end
                        1: begin
                            command <= reading && !addr_mode ? WR | STO : WR;
                            tx_byte <= reg_addr[15:8];
                        end
                        2: begin command <= reading ? WR | STO : WR; tx_byte <= reg_addr[7:0]; end
                        3: begin
                            command <= reading ? WR | STA : WR | STO;
                            tx_byte <= reading ? device_id | 8'd1 : wrdata;
                        end
                        4: command <= RD | NACK | STO;
                        default: ;
                    endcase
                end
                WAIT_BYTE: if (byte_done) begin
                    if (!reading || byte_index < 4)
                        ack <= ack | byte_nack;
                    if (byte_index == (reading ? 4 : 3))
                        state <= FINISH;
                    else begin
                        byte_index <= byte_index == 1 && !addr_mode ? 3'd3 : byte_index + 1'b1;
                        state <= ISSUE;
                    end
                end
                FINISH: begin
                    if (reading) rddata <= rx_byte;
                    state <= DELAY;
                end
                DELAY: begin
                    if (delay_count <= dly_cnt_max)
                        delay_count <= delay_count + 1'b1;
                    else begin
                        delay_count <= 0;
                        RW_Done <= 1;
                        state <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
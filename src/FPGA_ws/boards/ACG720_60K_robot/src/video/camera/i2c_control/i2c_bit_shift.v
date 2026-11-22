/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_bit_shift
// Description   : I2C协议基本数据数据字节传输模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module i2c_bit_shift #(
    parameter SYS_CLOCK = 50_000_000,
    parameter SCL_CLOCK = 400_000
) (
    input wire Clk, Rst_n,
    input wire [5:0] Cmd,
    input wire Go,
    output reg [7:0] Rx_DATA,
    input wire [7:0] Tx_DATA,
    output reg Trans_Done, ack_o, i2c_sclk,
    inout wire i2c_sdat
);
    localparam SCL_CNT_M = SYS_CLOCK / SCL_CLOCK / 4 - 1;
    localparam DIV_BITS = SCL_CNT_M < 1 ? 1 : $clog2(SCL_CNT_M + 1);
    localparam WR = 6'b000001, STA = 6'b000010, RD = 6'b000100,
               STO = 6'b001000, ACK = 6'b010000, NACK = 6'b100000;
    localparam [2:0] IDLE = 0, GEN_STA = 1, WR_DATA = 2, RD_DATA = 3,
                     CHECK_ACK = 4, GEN_ACK = 5, GEN_STO = 6;
    reg [2:0] state;
    reg [4:0] cnt;
    reg [DIV_BITS-1:0] div_cnt;
    reg en_div_cnt, sda_value, sda_enable;
    wire tick = div_cnt == SCL_CNT_M;
    wire [1:0] phase = cnt[1:0];
    wire last_tick = cnt == ((state == WR_DATA || state == RD_DATA) ? 5'd31 : 5'd3);

    assign i2c_sdat = sda_enable && !sda_value ? 1'b0 : 1'bz;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)
            div_cnt <= 0;
        else if (en_div_cnt && div_cnt < SCL_CNT_M)
            div_cnt <= div_cnt + 1'b1;
        else
            div_cnt <= 0;
    end

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            state <= IDLE;
            cnt <= 0;
            en_div_cnt <= 0;
            sda_value <= 1;
            sda_enable <= 0;
            i2c_sclk <= 1;
            Trans_Done <= 0;
            ack_o <= 0;
            Rx_DATA <= 0;
        end else if (state == IDLE) begin
            Trans_Done <= 0;
            sda_enable <= 1;
            en_div_cnt <= Go;
            if (Go) begin
                if (Cmd & STA) state <= GEN_STA;
                else if (Cmd & WR) state <= WR_DATA;
                else if (Cmd & RD) state <= RD_DATA;
            end
        end else if (tick) begin
            cnt <= last_tick ? 5'd0 : cnt + 1'b1;
            case (state)
                GEN_STA: begin
                    case (phase)
                        0: begin sda_value <= 1; sda_enable <= 1; end
                        1: i2c_sclk <= 1;
                        2: begin sda_value <= 0; i2c_sclk <= 1; end
                        3: i2c_sclk <= 0;
                    endcase
                    if (last_tick) begin
                        if (Cmd & WR) state <= WR_DATA;
                        else if (Cmd & RD) state <= RD_DATA;
                    end
                end
                WR_DATA, RD_DATA: begin
                    if (phase == 0) begin
                        sda_enable <= state == WR_DATA;
                        if (state == WR_DATA)
                            sda_value <= Tx_DATA[7 - cnt[4:2]];
                        else
                            i2c_sclk <= 0;
                    end else
                        i2c_sclk <= phase != 3;
                    if (state == RD_DATA && phase == 2)
                        Rx_DATA <= {Rx_DATA[6:0], i2c_sdat};
                    if (last_tick)
                        state <= state == WR_DATA ? CHECK_ACK : GEN_ACK;
                end
                CHECK_ACK, GEN_ACK: begin
                    if (phase == 0) begin
                        i2c_sclk <= 0;
                        sda_enable <= state == GEN_ACK;
                        if (state == GEN_ACK) begin
                            if (Cmd & ACK) sda_value <= 0;
                            else if (Cmd & NACK) sda_value <= 1;
                        end
                    end else
                        i2c_sclk <= phase != 3;
                    if (state == CHECK_ACK && phase == 2)
                        ack_o <= i2c_sdat;
                    if (last_tick) begin
                        if (Cmd & STO)
                            state <= GEN_STO;
                        else begin
                            state <= IDLE;
                            Trans_Done <= 1;
                        end
                    end
                end
                GEN_STO: begin
                    if (phase == 0) begin
                        sda_value <= 0;
                        sda_enable <= 1;
                    end else begin
                        i2c_sclk <= 1;
                        if (phase == 2) sda_value <= 1;
                    end
                    if (last_tick) begin
                        state <= IDLE;
                        Trans_Done <= 1;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

module mdio_bit_shift #(
    parameter SYS_CLOCK = 50_000_000,
    parameter MDC_CLOCK = 2_000
) (
    input wire Clk, Rst_n,
    input wire Mode, Go,
    input wire [4:0] Phy_addr, Reg_addr,
    input wire [15:0] Tx_Data,
    output reg [15:0] Rx_Data,
    output reg Trans_Done, mdc,
    inout wire mdio
);
    localparam MDC_CNT_M = SYS_CLOCK / MDC_CLOCK / 4 - 1;
    localparam DIV_BITS = MDC_CNT_M < 1 ? 1 : $clog2(MDC_CNT_M + 1);
    localparam IDLE = 8'h01, PRE = 8'h02, ST = 8'h04, OP = 8'h08,
               PHYAD = 8'h10, REGAD = 8'h20, TA = 8'h40, DATA = 8'h80;

    reg [DIV_BITS-1:0] div_cnt;
    reg en_div_cnt;
    reg [7:0] state;
    reg [6:0] cnt;
    reg mdio_o, mdio_oe;
    wire mdc_plus = div_cnt == MDC_CNT_M;
    wire [6:0] last_count = state == PRE ? 7'd127 :
                            (state == PHYAD || state == REGAD) ? 7'd19 :
                            state == DATA ? 7'd63 : 7'd7;
    wire last_tick = cnt == last_count;

    assign mdio = mdio_oe ? mdio_o : 1'bz;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)
            div_cnt <= 0;
        else if (en_div_cnt && div_cnt < MDC_CNT_M)
            div_cnt <= div_cnt + 1'b1;
        else
            div_cnt <= 0;
    end

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            Rx_Data <= 0;
            mdio_oe <= 0;
            mdio_o <= 1;
            mdc <= 0;
            cnt <= 0;
            en_div_cnt <= 0;
            Trans_Done <= 0;
            state <= IDLE;
        end else if (state == IDLE) begin
            Trans_Done <= 0;
            mdio_oe <= 0;
            en_div_cnt <= Go;
            if (Go)
                state <= PRE;
        end else if (mdc_plus) begin
            cnt <= last_tick ? 7'd0 : cnt + 1'b1;
            // 每位四拍：第 0 拍置数据，第 1 拍采样，第 2 拍保持，第 3 拍拉低。
            mdc <= cnt[1:0] == 2'd1 || cnt[1:0] == 2'd2;
            if (cnt[1:0] == 0) begin
                mdio_oe <= (state != TA && state != DATA) || !Mode;
                case (state)
                    PRE:   mdio_o <= 1'b1;
                    ST:    mdio_o <= cnt[2];
                    OP:    mdio_o <= cnt[2] ? !Mode : Mode;
                    PHYAD: mdio_o <= Phy_addr[4 - cnt[4:2]];
                    REGAD: mdio_o <= Reg_addr[4 - cnt[4:2]];
                    TA:    mdio_o <= !cnt[2];
                    DATA:  if (!Mode) mdio_o <= Tx_Data[15 - cnt[5:2]];
                    default: ;
                endcase
            end
            if (state == DATA && Mode && cnt[1:0] == 1)
                Rx_Data <= {Rx_Data[14:0], mdio};
            if (last_tick) begin
                case (state)
                    PRE:   state <= ST;
                    ST:    state <= OP;
                    OP:    state <= PHYAD;
                    PHYAD: state <= REGAD;
                    REGAD: state <= TA;
                    TA:    state <= DATA;
                    DATA: begin
                        state <= IDLE;
                        Trans_Done <= 1;
                    end
                    default: state <= IDLE;
                endcase
            end
        end
    end
endmodule

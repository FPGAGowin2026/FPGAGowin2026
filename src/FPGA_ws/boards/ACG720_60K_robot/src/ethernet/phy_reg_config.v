module phy_reg_config (
    input wire Clk, Rst_n,
    output wire Phy_rst_n,
    output wire [15:0] Rd_Data,
    output wire Phy_init_done,
    output reg Phy_link_up,
    output wire mdc,
    inout wire mdio
);
    localparam [3:0] PROBE = 0, SELECT_DELAY_PAGE = 1, READ_DELAY = 2,
        SET_DELAY = 3, READ_RX_DELAY = 4, SET_RX_DELAY = 5,
        SELECT_STD_PAGE = 6, ADV_1000 = 7, ADV_BASE = 8,
        RESTART_AUTONEG = 9, SELECT_STATUS_PAGE = 10, POLL_LINK = 11;
    localparam [21:0] LINK_POLL_CYCLES = 22'd2500000;

    reg [21:0] reset_count;
    reg [21:0] poll_count;
    reg [3:0] step;
    reg [4:0] phy_address;
    reg [15:0] delay_config;
    reg [21:0] command;
    reg go, busy;
    wire done;

    assign Phy_rst_n = reset_count >= 22'd600000;
    assign Phy_init_done = step == POLL_LINK;

    mdio_bit_shift #(.MDC_CLOCK(1_000_000)) mdio_access (
        .Clk(Clk), .Rst_n(Phy_rst_n), .Mode(command[21]), .Go(go),
        .Phy_addr(phy_address), .Reg_addr(command[20:16]), .Tx_Data(command[15:0]),
        .Rx_Data(Rd_Data), .Trans_Done(done), .mdc(mdc), .mdio(mdio)
    );

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            reset_count <= 0;
            poll_count <= 0;
            step <= PROBE;
            phy_address <= 0;
            delay_config <= 0;
            command <= 0;
            go <= 0;
            busy <= 0;
            Phy_link_up <= 0;
        end else begin
            go <= 0;
            if (reset_count < 22'd3300000) begin
                reset_count <= reset_count + 1'b1;
            end else if (busy) begin
                if (done) begin
                    busy <= 0;
                    case (step)
                        PROBE: begin
                            // 从地址 0 开始读取 PHY ID，兼容不同的板级地址绑带。
                            if (Rd_Data == 16'h001c)
                                step <= SELECT_DELAY_PAGE;
                            else
                                phy_address <= phy_address + 1'b1;
                        end
                        READ_DELAY, READ_RX_DELAY: begin
                            delay_config <= Rd_Data;
                            step <= step + 1'b1;
                        end
                        POLL_LINK: begin
                            Phy_link_up <= Rd_Data != 16'hffff && Rd_Data[5:4] == 2'b10 &&
                                           Rd_Data[3] && Rd_Data[2];
                            poll_count <= 0;
                        end
                        default: step <= step + 1'b1;
                    endcase
                end
            end else if (step == POLL_LINK && poll_count < LINK_POLL_CYCLES - 1'b1) begin
                poll_count <= poll_count + 1'b1;
            end else begin
                case (step)
                    PROBE:             command <= {1'b1, 5'd2,  16'd0};
                    SELECT_DELAY_PAGE: command <= {1'b0, 5'd31, 16'h0d08};
                    READ_DELAY:        command <= {1'b1, 5'd17, 16'd0};
                    // RGMII-ID: data and forwarded clock share the FPGA clock.
                    SET_DELAY:         command <= {1'b0, 5'd17, delay_config | 16'h0100};
                    READ_RX_DELAY:     command <= {1'b1, 5'd21, 16'd0};
                    // RXC 由 PHY 延后约 2 ns，让输入采样避开 RXD 跳变。
                    SET_RX_DELAY:      command <= {1'b0, 5'd21, delay_config | 16'h0008};
                    SELECT_STD_PAGE:   command <= {1'b0, 5'd31, 16'd0};
                    ADV_1000:          command <= {1'b0, 5'd9,  16'h0200};
                    ADV_BASE:          command <= {1'b0, 5'd4,  16'h0001};
                    RESTART_AUTONEG:   command <= {1'b0, 5'd0,  16'h1200};
                    SELECT_STATUS_PAGE:command <= {1'b0, 5'd31, 16'h0a43};
                    default:           command <= {1'b1, 5'd26, 16'd0};
                endcase
                go <= 1;
                busy <= 1;
            end
        end
    end
endmodule

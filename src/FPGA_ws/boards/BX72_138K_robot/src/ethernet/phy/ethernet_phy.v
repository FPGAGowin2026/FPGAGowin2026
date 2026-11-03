// ==================== rgmii_io.v ====================
//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
//
// Create Date: 2021/09/20 00:00:00
// Design Name:
// Module Name:
// Project Name:
// Target Devices: xc7z020clg400-2
// Tool Versions: Vivado 2018.3
// Description: gmii转rgmii
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module gmii_to_rgmii(
  reset_n,

  gmii_tx_clk,
  gmii_txd,
  gmii_txen,
  gmii_txer,

  rgmii_tx_clk,
  rgmii_txd,
  rgmii_txen
);



  input        reset_n;

  input        gmii_tx_clk;
  input  [7:0] gmii_txd;
  input        gmii_txen;
  input        gmii_txer;

  output       rgmii_tx_clk;
  output [3:0] rgmii_txd;
  output       rgmii_txen;

  // ODDR 在同一上升沿采样 D0/D1；先将完整 GMII 字/控制寄存，
  // 两相都从同一快照取值，保证字节与 TX_CTL 配对且固定延迟一拍。
  // 误码位与使能一并寄存，维持 TX_CTL=EN / EN^ER 的配对关系。
  (* syn_preserve=1 *) reg [9:0] gmii_tx_stage_q;
  always @(posedge gmii_tx_clk or negedge reset_n) begin
    if (!reset_n) begin
      gmii_tx_stage_q <= 10'd0;
    end else begin
      gmii_tx_stage_q <= {gmii_txer,gmii_txen,gmii_txd};
    end
  end

  generate
    genvar i;
        for(i=0;i<4;i=i+1)
        begin : rgmii_data_lane
        ODDR    U_ODDR_dq1
                (.Q0(rgmii_txd[i]),
                 .Q1(),
                 .D0(gmii_tx_stage_q[i]),
                 .D1(gmii_tx_stage_q[i+4]),
                 .TX(1'b1),
                 .CLK(gmii_tx_clk)
                );
        end
  endgenerate

	ODDR    U_ODDR_en1
			(.Q0(rgmii_txen),
             .Q1(),

             .D0(gmii_tx_stage_q[8]),
             .D1(gmii_tx_stage_q[8]^gmii_tx_stage_q[9]),
             .TX(1'b1),

             .CLK(gmii_tx_clk)
			);

	ODDR    U_ODDR_clk1
			(.Q0(rgmii_tx_clk),
             .Q1(),

             .D0(1'b1),
             .D1(1'b0),
             .TX(1'b1),

             .CLK(gmii_tx_clk)
			);

endmodule

// RGMII：上升沿低四位/RX_DV，下降沿高四位/RX_DV xor RX_ER。
// 输入为已移相的 RX 采样时钟；板测仍须确认 PHY 延迟与外部输入时序。
module bx72_rgmii_rx (
    input wire rgmii_rx_clk_i,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rxdv,
    output wire [7:0] gmii_rxd,
    output wire gmii_rxdv,
    output wire gmii_rxer
);
    wire ctl_rise, ctl_fall;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin: lanes
        IDDR capture(.Q0(gmii_rxd[lane]),.Q1(gmii_rxd[lane+4]),
            .D(rgmii_rxd[lane]),.CLK(rgmii_rx_clk_i));
    end endgenerate
    IDDR control_capture(.Q0(ctl_rise),.Q1(ctl_fall),.D(rgmii_rxdv),.CLK(rgmii_rx_clk_i));
    assign gmii_rxdv=ctl_rise;
    assign gmii_rxer=ctl_rise^ctl_fall;
endmodule

// ==================== phy_reg_config.v ====================
module phy_reg_config(
    input  wire        Clk,
    input  wire        Rst_n,
    output wire        Phy_rst_n,
    output wire [15:0] Rd_Data,
    output wire        Phy_init_done,
    output wire        Phy_link_up,
    output wire        mdc,
    inout  wire        mdio
);
    localparam [2:0] MAX_INIT_WRITES = 3'd3;
    localparam [22:0] LINK_POLL_INTERVAL = 23'd2500000;

    reg [24:0] reset_count;
    reg [1:0] state;
    reg [2:0] reg_count;
    reg [21:0] mdio_data;
    reg go;
    reg [22:0] link_poll_count;
    reg link_up;
    wire trans_done;

    wire start = (reset_count > 25'd1499999);
    assign Phy_rst_n = (reset_count > 25'd500000);
    assign Phy_init_done = (reg_count == MAX_INIT_WRITES);
    assign Phy_link_up = link_up;

    mdio_bit_shift mdio_bit_shift (
        .Clk(Clk),
        .Rst_n(Phy_rst_n),
        .Mode(mdio_data[21]),
        .Go(go),
        .Phy_addr(5'b00001),
        .Reg_addr(mdio_data[20:16]),
        .Tx_Data(mdio_data[15:0]),
        .Rx_Data(Rd_Data),
        .Trans_Done(trans_done),
        .mdc(mdc),
        .mdio(mdio)
    );

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            reset_count <= 25'd0;
            state <= 2'd0;
            reg_count <= 3'd0;
            mdio_data <= 22'd0;
            go <= 1'b0;
            link_poll_count <= 23'd0;
            link_up <= 1'b0;
        end else begin
            if (reset_count < 25'd1500000)
                reset_count <= reset_count + 1'b1;

            if (!start) begin
                state <= 2'd0;
                reg_count <= 3'd0;
                go <= 1'b0;
                link_poll_count <= 23'd0;
                link_up <= 1'b0;
            end else if (reg_count < MAX_INIT_WRITES) begin
                link_poll_count <= 23'd0;
                case (state)
                    2'd0: begin
                        case (reg_count)
                            3'd0: mdio_data <= {1'b0, 5'd9, 16'h0200}; // advertise 1000BASE-T full duplex only
                            3'd1: mdio_data <= {1'b0, 5'd4, 16'h0001}; // IEEE 802.3 selector; no 10/100 modes
                            3'd2: mdio_data <= {1'b0, 5'd0, 16'h1200}; // restart auto-negotiation
                            default: mdio_data <= {1'b0, 5'd0, 16'h1200};
                        endcase
                        go <= 1'b1;
                        state <= 2'd1;
                    end
                    2'd1: begin
                        if (trans_done) begin
                            go <= 1'b0;
                            state <= 2'd0;
                            reg_count <= reg_count + 1'b1;
                        end
                    end
                    default: state <= 2'd0;
                endcase
            end else if (state == 2'd0) begin
                go <= 1'b0;
                if (link_poll_count >= LINK_POLL_INTERVAL - 1'b1) begin
                    link_poll_count <= 23'd0;
                    mdio_data <= {1'b1, 5'd1, 16'd0}; // read BMSR
                    go <= 1'b1;
                    state <= 2'd1;
                end else begin
                    link_poll_count <= link_poll_count + 1'b1;
                end
            end else if (state == 2'd1 && trans_done) begin
                go <= 1'b0;
                state <= 2'd0;
                link_up <= Rd_Data[2];
                link_poll_count <= 23'd0;
            end
        end
    end
endmodule

module mdio_bit_shift(
	Clk,
	Rst_n,

	Mode,
	Go,
	Phy_addr,
	Reg_addr,
	Tx_Data,
	Rx_Data,
	Trans_Done,

	mdc,
	mdio
);
	input Clk;
	input Rst_n;

	input Mode; //1:read,0:write
	input Go;
	input [4:0] Phy_addr;//phy地址
	input [4:0] Reg_addr;//reg地址
	input [15:0] Tx_Data;
	output reg [15:0] Rx_Data;
	output reg Trans_Done;
	output reg mdc;
	inout mdio;

	reg mdio_o;
	reg mdio_oe;

	//系统时钟采用50MHz
	parameter SYS_CLOCK = 50_000_000;
	//MDC总线时钟采用2kHz
	parameter MDC_CLOCK = 2_000;
	//产生时钟mdc计数器最大值
	localparam MDC_CNT_M = SYS_CLOCK/MDC_CLOCK/4 - 1;

	reg [19:0] div_cnt;
	reg en_div_cnt;
	always @ (posedge Clk or negedge Rst_n)
	if (!Rst_n)
		div_cnt <= 'd0;
	else if (en_div_cnt) begin
		if(div_cnt < MDC_CNT_M)
			div_cnt <= div_cnt + 1'b1;
		else
			div_cnt <= 'd0;
	end else
		div_cnt <= 'd0;

	wire mdc_plus = div_cnt == MDC_CNT_M;

	assign mdio = mdio_oe ? mdio_o : 1'bz;

	localparam
		IDLE  = 8'h01,
		PRE   = 8'h02,
		ST    = 8'h04,
		OP    = 8'h08,
		PHYAD = 8'h10,
		REGAD = 8'h20,
		TA    = 8'h40,
		DATA  = 8'h80;

	//--------------------
	reg [7:0] state;
	reg [7:0] cnt;
	always @ (posedge Clk or negedge Rst_n)
	if (!Rst_n) begin
		Rx_Data <= 'd0;
		mdio_oe <= 1'b0;
		mdio_o <= 1'b1;
		cnt <= 'd0;
		en_div_cnt <= 1'b0;
		Trans_Done <= 1'b0;
		state <= IDLE;
	end else begin
		case (state)
			IDLE:
				begin
					Trans_Done <= 1'b0;
					mdio_oe <= 1'b0;
					if (Go) begin
						en_div_cnt <= 1'b1;
						state <= PRE;
					end else begin
						en_div_cnt <= 1'b0;
						state <= IDLE;
					end
				end
			PRE: //PRE 32'hffff_ffff 32bits
				begin
					if (mdc_plus) begin
						if (cnt == 'd127)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd127)
							state <= ST;
						case (cnt)
							0,4, 8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,
							68,72,76,80,84,88,92,96,100,104,108,112,116,120,124 :
								begin mdio_o <= 1'b1; mdio_oe <= 1'b1; end //set data

							1,5, 9,13,17,21,25,29,33,37,41,45,49,53,57,61,65,
							69,73,77,81,85,89,93,97,101,105,109,113,117,121,125 :
								begin mdc <= 1'b1; end //mdc posedge

							2,6,10,14,18,22,26,30,34,38,42,46,50,54,58,62,66,
							70,74,78,82,86,90,94,98,102,106,110,114,118,122,126 :
								begin mdc <= 1'b1; end //mdc keep high

							3,7,11,15,19,23,27,31,35,39,43,47,51,55,59,63,67,
							71,75,79,83,87,91,95,99,103,107,111,115,119,123,127 :
								begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			ST: //ST 2'b01
				begin
					if (mdc_plus) begin
						if (cnt == 'd7)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd7)
							state <= OP;
						case (cnt)
							0 : begin mdio_o <= 1'b0; mdio_oe <= 1'b1; end //set data
							4 : begin mdio_o <= 1'b1; mdio_oe <= 1'b1; end //set data
							1, 5 : begin mdc <= 1'b1; end //mdc posedge
							2, 6 : begin mdc <= 1'b1; end //mdc keep high
							3, 7 : begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			OP: //OP 01:write, 10:read 2bits
				begin
					if (mdc_plus) begin
						if (cnt == 'd7)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd7)
							state <= PHYAD;
						case (cnt)
							0: begin mdio_o <= Mode; mdio_oe <= 1'b1; end //set data
							4: begin mdio_o <= !Mode; mdio_oe <= 1'b1; end //set data
							1,5: begin mdc <= 1'b1; end //mdc posedge
							2,6: begin mdc <= 1'b1; end //mdc keep high
							3,7: begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			PHYAD: //PHYAD 5bits
				begin
					if (mdc_plus) begin
						if (cnt == 'd19)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd19)
							state <= REGAD;
						case (cnt)
							0,4,8 ,12,16: begin mdio_o <= Phy_addr[4 - cnt[4:2]]; mdio_oe <= 1'b1; end //set data
							1,5,9 ,13,17: begin mdc <= 1'b1; end //mdc posedge
							2,6,10,14,18: begin mdc <= 1'b1; end //mdc keep high
							3,7,11,15,19: begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			REGAD: //REGAD 5bits
				begin
					if (mdc_plus) begin
						if (cnt == 'd19)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd19)
							state <= TA;
						case (cnt)
							0,4,8 ,12,16: begin mdio_o <= Reg_addr[4 - cnt[4:2]]; mdio_oe <= 1'b1; end //set data
							1,5,9 ,13,17: begin mdc <= 1'b1; end //mdc posedge
							2,6,10,14,18: begin mdc <= 1'b1; end //mdc keep high
							3,7,11,15,19: begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			TA: //TA Z0:read, 10:write 2bits
				begin
					if (mdc_plus) begin
						if (cnt == 'd7)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;
						if (cnt == 'd7)
							state <= DATA;
						case (cnt)
							0: begin mdio_o <= 1'b1; mdio_oe <= (Mode == 1'b0); end //set data
							//4: begin mdio_o <= 1'b0; mdio_oe <= 1'b1; end //set data
							4: begin mdio_o <= 1'b0; mdio_oe <= (Mode == 1'b0); end //set data
							1,5: begin mdc <= 1'b1; end //mdc posedge
							2,6: begin mdc <= 1'b1; end //mdc keep high
							3,7: begin mdc <= 1'b0; end //mdc negedge
							default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
						endcase
					end
				end
			DATA:
				begin
					if (mdc_plus) begin
						if (cnt == 'd63)
							cnt <= 'd0;
						else
							cnt <= cnt + 1'b1;

						if (cnt == 'd63) begin
							state <= IDLE;
							Trans_Done <= 1'b1;
						end

						if (Mode) //read data
							case (cnt)
								0,4,8 ,12,16,20,24,28,32,36,40,44,48,52,56,60: begin mdio_oe <= 1'b0; mdc <= 1'b0; end
								1,5,9 ,13,17,21,25,29,33,37,41,45,49,53,57,61: begin mdc <= 1'b1; end //mdc posedge
								2,6,10,14,18,22,26,30,34,38,42,46,50,54,58,62:
									begin
										mdc <= 1'b1;
										Rx_Data <= {Rx_Data[14:0], mdio};
									end //mdc keep high
								3,7,11,15,19,23,27,31,35,39,43,47,51,55,59,63: begin mdc <= 1'b0; end //mdc negedge
								default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
							endcase
						else //write data
							case (cnt)
								0,4,8 ,12,16,20,24,28,32,36,40,44,48,52,56,60:
									begin
										mdio_oe <= 1'b1;
										mdio_o <= Tx_Data[15 - cnt[5:2]];
									end //set data
								1,5,9 ,13,17,21,25,29,33,37,41,45,49,53,57,61: begin mdc <= 1'b1; end //mdc posedge
								2,6,10,14,18,22,26,30,34,38,42,46,50,54,58,62: begin mdc <= 1'b1; end //mdc keep high
								3,7,11,15,19,23,27,31,35,39,43,47,51,55,59,63: begin mdc <= 1'b0; end //mdc negedge
								default : begin mdio_o <= 1'b1; mdc <= 1'b1; end
							endcase
					end
				end
			default : state <= IDLE;
		endcase
	end

endmodule

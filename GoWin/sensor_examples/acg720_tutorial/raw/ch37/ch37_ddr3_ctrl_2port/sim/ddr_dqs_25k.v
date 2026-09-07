
// DQS
module DQS(DQSR90, DQSW0, DQSW270, RPOINT, WPOINT, RVALID, RBURST, RFLAG, WFLAG, DQSIN, DLLSTEP, WSTEP, READ, RLOADN, RMOVE, RDIR, WLOADN, WMOVE, WDIR, HOLD, RCLKSEL, PCLK, FCLK, RESET);
input DQSIN,PCLK,FCLK,RESET;
input [3:0] READ;
input [2:0] RCLKSEL;
input [7:0] DLLSTEP;
input [7:0] WSTEP;
input RLOADN, RMOVE, RDIR, WLOADN, WMOVE, WDIR, HOLD;
output DQSR90, DQSW0, DQSW270; 
output [2:0] RPOINT, WPOINT;
output RVALID,RBURST, RFLAG, WFLAG;

parameter FIFO_MODE_SEL = 1'b0; // FIFO mode select,1'b0: DDR memory mode;1'b1: GDDR mode
parameter RD_PNTR = 3'b000; // FIFO read pointer setting
parameter DQS_MODE = "X1"; // "X1", "X2_DDR2", "X2_DDR3","X4","X2_DDR3_EXT"
parameter HWL = "false";   //"true"; "false"


reg UPDATE=0;
reg [2:0] MODE;
initial begin
  if (HWL=="false")
    UPDATE=0;
  else
    UPDATE=1;
end
initial begin
  if (DQS_MODE=="X1")
    MODE=3'b000;
  else if (DQS_MODE=="X2_DDR2")
    MODE=3'b001;
  else if (DQS_MODE=="X2_DDR3")
    MODE=3'b010;
  else if (DQS_MODE=="X4")
    MODE=3'b011;
  else if (DQS_MODE=="X2_DDR3_EXT")
    MODE=3'b110;
end
 //gw5a25_JXLeOdqufK12r_iVCKY_Mg core( 
 ddr_dqs_top core(
    cdrclk, clkrclk1, DQSR90, DQSW0, DQSW270, nextclk, RBURST, RVALID, RFLAG, RPOINT,
    WFLAG, WPOINT, 1'b0, 1'b0, DLLSTEP, 8'b0, DQSIN, 1'b1, 1'b1, {3'b0, FCLK},
    HOLD, 4'b0, 1'b0, 1'b0, FIFO_MODE_SEL, 1'b1, 1'b0, 1'b0, MODE, 1'b1, 1'b1,
    1'b1, 2'b0, 2'b0, 1'b1, 1'b1, 1'b1, RD_PNTR, 2'b0, 2'b11, 1'b0, 8'b0, 8'b0,
    1'b0, 1'b0, UPDATE, 2'b11, 2'b11, RDIR, RLOADN, RMOVE, READ, RCLKSEL, RESET,
    3'b0, 11'b0, PCLK, 1'b1, 1'b0, WDIR, WLOADN, WMOVE, WSTEP

);
/*
gw5a25_JXLeOdqufK12r_iVCKY_Mg core( .cdr_divclk_cib_pclk(cdr_divclk_cib_pclk), .cdrclk(cdrclk), .dqsr90(DQSR90), .dqsw(DQSW0),
     .dqsw270(DQSW270), .nextclk(nextclk), .r_burst_det_cib(RBURST), .r_datavaild_cib(RVALID),
     .r_mrgt_flag_cib(RFLAG), .rd_pntr(RPOINT), .w_mrgt_flag_cib(WFLAG), .wr_pntr(WPOINT), .cdr_slip(1'b0),
     .cdr_switch(1'b0), .dll_code0(DLLSTEP), .dll_code1(8'b0), .dqsi(DQSIN), .forcearchn(1'b1), .gsrn(1'b1), .hclk({3'b0, FCLK}),
     .hold(HOLD), .mc1_cdr_divmod(4'b0), .mc1_cdr_en(1'b0), .mc1_centered_r(1'b0), .mc1_cou_en(FIFO_MODE_SEL),
     .mc1_delay_en_r(1'b1), .mc1_dis_gsr(1'b0), .mc1_dllcodesel(1'b0), .mc1_dqs_mode(MODE),
     .mc1_dqs_rst_pol(1'b1), .mc1_en_dqsfifo(1'b1), .mc1_en_w(1'b1), .mc1_hclk90_sel(2'b0),
     .mc1_hclk_sel(2'b0), .mc1_hold_en(1'b1), .mc1_mrg_en_r(1'b1), .mc1_mrg_en_w(1'b1),
     .mc1_pntr_fifo(3'b000), .mc1_r_delay_mode(2'b0), .mc1_r_iodly_pwron(2'b11), .mc1_read_dis(1'b0),
     .mc1_s_r(8'b0), .mc1_setting_w(8'b0), .mc1_sign_r(1'b0), .mc1_sign_w(1'b0), .mc1_updatequ(UPDATE),
     .mc1_w_delay_mode(2'b11), .mc1_w_iodly_pwron(2'b11), .r_direction(RDIR), .r_loadn(RLOADN), .r_move(RMOVE),
     .read(READ), .read_rclk_sel(RCLKSEL), .rst(RESET), .sela(3'b0), .selb(11'b0), .sysclk(PCLK), .vccc(1'b1), .vss(1'b0),
     .w_direction(WDIR), .w_loadn(WLOADN), .w_move(WMOVE), .wl_sela(WSTEP) );

*/

endmodule


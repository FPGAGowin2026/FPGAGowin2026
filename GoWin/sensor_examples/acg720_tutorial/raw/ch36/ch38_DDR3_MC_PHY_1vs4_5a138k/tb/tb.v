
`timescale 1ns /1ps
module tb();

  //`include "gwmc_param.v"
  //`include "gwmc_local_param.v"

   reg clk,rst,rst_n,display,start;
   wire                  	ddr_cs;
   wire                  	ddr_ras;
   wire                  	ddr_cas;
   wire                  	ddr_we;
   wire                  	ddr_ck;
   wire                  	ddr_ck_n;
   wire                  	ddr_cke;
   wire                  	ddr_odt;
   wire                  	ddr_reset_n;
   wire [4-1:0]   			ddr_dm;		//DM_WIDTH=2
   wire [32-1:0]   			ddr_dq;		//DQ_WIDTH=16
   wire [4-1:0]  			ddr_dqs;	//DQS_WIDTH=2
   wire [4-1:0]  			ddr_dqs_n;
   wire [14-1:0]  			ddr_addr;		//ROW_WIDTH=14
   wire [3-1:0] 			ddr_bank;		//BANK_WIDTH=3


   wire  				    error;
   wire  				    error1;



GSR GSR(.GSRI(1'b1));


top u_top(
    .clk(clk),  
    .error(error),
    .error1(error1),
	.led(led),
	.init_calib_complete(init_calib_complete),
    .ddr_addr(ddr_addr),
	.ddr_bank(ddr_bank),
	.ddr_cs(ddr_cs),
	.ddr_ras(ddr_ras),
	.ddr_cas(ddr_cas),
	.ddr_we(ddr_we),
	.ddr_ck(ddr_ck),
	.ddr_ck_n(ddr_ck_n),
	.ddr_cke(ddr_cke),
	.ddr_odt(ddr_odt),
	.ddr_reset_n(ddr_reset_n),
	.ddr_dm(ddr_dm),
	.ddr_dq(ddr_dq),
	.ddr_dqs(ddr_dqs),
	.ddr_dqs_n(ddr_dqs_n)
);



// ddr3_model ddr3_model(
          // .rst_n            (ddr_reset_n),
          // .ck               (ddr_ck),
          // .ck_n             (ddr_ck_n),
          // .cke              (ddr_cke),
          // .cs_n             (ddr_cs),
          // .ras_n            (ddr_ras),
          // .cas_n            (ddr_cas),
          // .we_n             (ddr_we),
          // .dm_tdqs          (ddr_dm),
          // .ba               (ddr_bank),
          // .addr             (ddr_addr),
          // .dq               (ddr_dq),
          // .dqs              (ddr_dqs),
          // .dqs_n            (ddr_dqs_n),
          // .tdqs_n           (),
          // .odt              (ddr_odt)
        // );


generate 
  genvar o;
    for(o = 0; o < 2; o = o + 1'b1) begin:ddr3_model
        ddr3_model ddr3_model(
          .rst_n            (ddr_reset_n),
          .ck               (ddr_ck),
          .ck_n             (ddr_ck_n),
          .cke              (ddr_cke),
          .cs_n             (ddr_cs),
          .ras_n            (ddr_ras),
          .cas_n            (ddr_cas),
          .we_n             (ddr_we),
          .dm_tdqs          (ddr_dm[o*2+:2]),
          .ba               (ddr_bank),
          .addr             (ddr_addr),
          .dq               (ddr_dq[o*16+:16]),
          .dqs              (ddr_dqs[o*2+:2]),
          .dqs_n            (ddr_dqs_n[o*2+:2]),
          .tdqs_n           (),
          .odt              (ddr_odt)
        );
      end
endgenerate


always #10 clk = ~clk;

initial begin
  $fsdbDumpfile("ddr3.fsdb");
  $fsdbDumpvars;
end

initial
begin
    clk = 0;
	#100;
   #4000000000;
   #4000000000;
   $finish;
end

endmodule



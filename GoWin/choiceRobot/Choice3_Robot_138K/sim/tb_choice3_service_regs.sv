`timescale 1ns/1ps
module tb_choice3_service_regs;
  reg clk=0,rst=1,cyc=0,stb=0,we=0;reg [31:0] adr=0,dat_i=0;
  reg [3:0] sel=4'hf;wire [31:0] dat_o;wire ack;
  reg [7:0] health=8'ha5,fault=8'h12;reg [3:0] nav=4'h7;
  reg route_done=1,drive=1,armed=1,cpu_fault=0;
  reg [15:0] range_mm=16'd345;reg range_valid=1,blocked=0;
  reg signed [15:0] roll=-16'sd12,pitch=16'sd34,yaw=16'sd178;
  reg ext_lease=1,ext_seen=1,ext_accept=1,ext_reject=0,ext_timeout=1,ext_conflict=1;
  reg [2:0] ext_reason=3'd3;reg [31:0] ext_seq=32'h1234_5678;
  reg [31:0] ext_accepts=7,ext_rejects=2,ext_timeouts=1;
  wire [31:0] heartbeat,scratch,transactions,errors_count;
  wire heartbeat_write,arm_pulse,disarm_pulse,route_pulse,clear_pulse,manual_valid;
  wire [2:0] manual_motion;
  integer errors=0;integer ack_count;
  always #5 clk=~clk;

  choice3_robot_service_regs u_dut(
    .wb_clk_i(clk),.wb_rst_i(rst),.wb_cyc_i(cyc),.wb_stb_i(stb),.wb_we_i(we),
    .wb_adr_i(adr),.wb_dat_i(dat_i),.wb_sel_i(sel),.wb_dat_o(dat_o),.wb_ack_o(ack),
    .health_i(health),.fault_i(fault),.nav_state_i(nav),.route_done_i(route_done),
    .drive_enabled_i(drive),.armed_i(armed),.cpu_watchdog_fault_i(cpu_fault),
    .fused_range_mm_i(range_mm),.fused_range_valid_i(range_valid),
    .front_blocked_i(blocked),.roll_deg_i(roll),.pitch_deg_i(pitch),.yaw_deg_i(yaw),
    .ext_lease_active_i(ext_lease),.ext_sequence_seen_i(ext_seen),
    .ext_accepted_pulse_i(ext_accept),.ext_rejected_pulse_i(ext_reject),
    .ext_timeout_pulse_i(ext_timeout),.ext_conflict_pulse_i(ext_conflict),
    .ext_reject_reason_i(ext_reason),.ext_last_sequence_i(ext_seq),
    .ext_accepted_count_i(ext_accepts),.ext_rejected_count_i(ext_rejects),
    .ext_timeout_count_i(ext_timeouts),.heartbeat_o(heartbeat),
    .heartbeat_write_o(heartbeat_write),.arm_pulse_o(arm_pulse),
    .disarm_pulse_o(disarm_pulse),.route_start_pulse_o(route_pulse),
    .clear_fault_pulse_o(clear_pulse),.manual_command_valid_o(manual_valid),
    .manual_motion_o(manual_motion),.scratch_o(scratch),
    .transaction_count_o(transactions),.error_count_o(errors_count));

  task check(input bit cond,input string msg);
    if(!cond)begin $display("FAIL: %s",msg);errors=errors+1;end
  endtask
  task wb_read(input [31:0] address,output [31:0] value);
    begin
      @(negedge clk);adr=address;we=0;cyc=1;stb=1;
      @(posedge clk);#1;check(ack,"read did not ACK");value=dat_o;
      @(negedge clk);cyc=0;stb=0;
      @(posedge clk);#1;check(!ack,"ACK did not clear");
    end
  endtask
  task wb_write(input [31:0] address,input [31:0] value,input [3:0] bytes);
    begin
      @(negedge clk);adr=address;dat_i=value;sel=bytes;we=1;cyc=1;stb=1;
      @(posedge clk);#1;check(ack,"write did not ACK");
      @(negedge clk);cyc=0;stb=0;we=0;sel=4'hf;
    end
  endtask
  reg [31:0] value;
  initial begin
    repeat(3)@(posedge clk);@(negedge clk);rst=0;
    wb_read(32'h2000_0000,value);check(value==32'h5242_4f54,"SYS_ID mismatch");
    wb_read(32'h2000_0004,value);check(value==32'h0003_0000,"ABI version mismatch");
    wb_read(32'h2000_0008,value);check(value==32'h0000_007f,"capabilities mismatch");
    wb_read(32'h2000_1014,value);check(value==32'h0000_01af,"extension status packing mismatch");
    wb_read(32'h2000_1018,value);check(value==ext_seq,"last sequence mismatch");
    wb_read(32'h2000_101c,value);check(value==7,"accept count mismatch");
    wb_read(32'h2000_1020,value);check(value==2,"reject count mismatch");
    wb_read(32'h2000_1024,value);check(value==1,"timeout count mismatch");

    // A held Wishbone request is accepted once, never once per cycle.
    @(negedge clk);adr=32'h2000_0000;we=0;cyc=1;stb=1;ack_count=0;
    repeat(4)begin @(posedge clk);#1;if(ack)ack_count=ack_count+1;end
    check(ack_count==1,"held request generated multiple ACKs");
    @(negedge clk);cyc=0;stb=0;@(posedge clk);

    wb_write(32'h2000_000c,32'ha55a_5aa5,4'hf);
    check(heartbeat==32'ha55a_5aa5,"heartbeat write failed");
    wb_write(32'h2000_2000,32'h0000_0b0d,4'hf);
    // Command fields: arm+route+clear, manual valid, motion=5.
    check(arm_pulse&&route_pulse&&clear_pulse&&manual_valid&&manual_motion==5,
      "high-level command decode failed");
    check(!disarm_pulse,"unexpected disarm bit");
    wb_write(32'h2000_f000,32'hcafe_babe,4'b0011);
    check(scratch==32'h0000_babe,"byte-select scratch write failed");
    wb_read(32'h2000_0002,value);check(value==32'hdead_beef,"misaligned access data mismatch");
    check(errors_count==1,"invalid access error count mismatch");

    if(errors==0)$display("PASS: tb_choice3_service_regs");
    else begin $display("FAIL: tb_choice3_service_regs errors=%0d",errors);$fatal(1);end
    $finish;
  end
endmodule

`timescale 1ns/1ps
module tb_choice3_external_command_guard;
    reg clk=0,rst_n=0,safety_block=0;
    reg valid=0;reg [31:0] cmd_sequence=0;reg [3:0] opcode=0;reg [2:0] motion=0;
    wire ready,arm,disarm,clear_fault,route,manual_valid;
    wire [2:0] manual_motion,reject_reason;
    wire accepted,rejected,lease_active,lease_timeout,sequence_seen;
    wire [31:0] last_sequence,accepted_count,rejected_count,timeout_count;

    reg cpu_arm=0,cpu_disarm=0,cpu_clear=0,cpu_route=0,cpu_manual=0;
    reg [2:0] cpu_motion=0;
    reg ext_arm=0,ext_disarm=0,ext_clear=0,ext_route=0,ext_manual=0;
    reg [2:0] ext_motion=0;
    wire arb_arm,arb_disarm,arb_clear,arb_route,arb_manual,arb_conflict;
    wire [2:0] arb_motion;
    integer errors=0;reg [31:0] timeout_before;

    always #5 clk=~clk;

    choice3_external_command_guard #(.LEASE_TIMEOUT_CYCLES(5)) u_guard(
      .clk_i(clk),.rst_n_i(rst_n),.safety_block_i(safety_block),
      .command_valid_i(valid),.command_ready_o(ready),
      .command_sequence_i(cmd_sequence),.command_opcode_i(opcode),
      .command_motion_i(motion),.arm_pulse_o(arm),.disarm_pulse_o(disarm),
      .clear_fault_pulse_o(clear_fault),.route_start_pulse_o(route),
      .manual_command_valid_o(manual_valid),.manual_motion_o(manual_motion),
      .accepted_pulse_o(accepted),.rejected_pulse_o(rejected),
      .reject_reason_o(reject_reason),.lease_active_o(lease_active),
      .lease_timeout_pulse_o(lease_timeout),.sequence_seen_o(sequence_seen),
      .last_sequence_o(last_sequence),.accepted_count_o(accepted_count),
      .rejected_count_o(rejected_count),.timeout_count_o(timeout_count));

    choice3_command_arbiter u_arbiter(
      .clk_i(clk),.rst_n_i(rst_n),.cpu_arm_i(cpu_arm),
      .cpu_disarm_i(cpu_disarm),.cpu_clear_i(cpu_clear),.cpu_route_i(cpu_route),
      .cpu_manual_valid_i(cpu_manual),.cpu_manual_motion_i(cpu_motion),
      .ext_arm_i(ext_arm),.ext_disarm_i(ext_disarm),.ext_clear_i(ext_clear),
      .ext_route_i(ext_route),.ext_manual_valid_i(ext_manual),
      .ext_manual_motion_i(ext_motion),.arm_o(arb_arm),.disarm_o(arb_disarm),
      .clear_o(arb_clear),.route_o(arb_route),.manual_valid_o(arb_manual),
      .manual_motion_o(arb_motion),.conflict_pulse_o(arb_conflict));

    task check(input bit cond,input string msg);
      if(!cond)begin $display("FAIL: %s",msg);errors=errors+1;end
    endtask

    task send_command(input [31:0] seq,input [3:0] op,input [2:0] mot);
      begin
        @(negedge clk);cmd_sequence=seq;opcode=op;motion=mot;valid=1;
        @(posedge clk);#1;
        @(negedge clk);valid=0;opcode=0;motion=0;
      end
    endtask

    task reset_dut;
      begin
        @(negedge clk);rst_n=0;valid=0;safety_block=0;
        cpu_arm=0;cpu_disarm=0;cpu_clear=0;cpu_route=0;cpu_manual=0;
        ext_arm=0;ext_disarm=0;ext_clear=0;ext_route=0;ext_manual=0;
        repeat(2)@(posedge clk);
        @(negedge clk);rst_n=1;
        @(posedge clk);#1;check(ready,"ready must assert after reset");
      end
    endtask

    initial begin
      reset_dut();

      send_command(32'd1,4'h0,3'd0);
      check(rejected&&reject_reason==3'd5,"keepalive without lease must reject");
      check(!sequence_seen&&last_sequence==0,"rejected command consumed sequence");

      send_command(32'd100,4'h5,3'd2);
      check(accepted&&manual_valid&&manual_motion==2,"first manual command not accepted");
      check(lease_active&&last_sequence==100,"manual did not establish lease/sequence");

      send_command(32'd100,4'h5,3'd2);
      check(rejected&&reject_reason==3'd1,"duplicate sequence not rejected");
      send_command(32'd99,4'h5,3'd2);
      check(rejected&&reject_reason==3'd1,"stale sequence not rejected");

      send_command(32'd101,4'hf,3'd0);
      check(rejected&&reject_reason==3'd2,"unknown opcode not rejected");
      check(last_sequence==100,"unknown opcode consumed sequence");
      send_command(32'd101,4'h1,3'd1);
      check(rejected&&reject_reason==3'd3,"reserved payload bits not rejected");
      check(last_sequence==100,"bad payload consumed sequence");

      send_command(32'd101,4'h1,3'd0);
      check(accepted&&arm&&lease_active&&last_sequence==101,"valid retry not accepted");
      send_command(32'd102,4'h0,3'd0);
      check(accepted&&lease_active&&last_sequence==102,"keepalive not accepted");
      timeout_before=timeout_count;
      repeat(4)begin @(posedge clk);#1;check(!lease_timeout,"lease expired early");end
      @(posedge clk);#1;
      check(lease_timeout&&disarm&&!lease_active,"lease timeout did not fail safe");
      check(timeout_count==timeout_before+1,"timeout counter incorrect");

      // First packet may establish an arbitrary sequence.  Thereafter modulo
      // arithmetic must accept the normal ffffffff -> 00000000 wrap.
      reset_dut();
      send_command(32'hffff_fffe,4'h1,3'd0);
      check(accepted&&arm,"wrap setup arm failed");
      send_command(32'hffff_ffff,4'h0,3'd0);
      check(accepted,"pre-wrap sequence rejected");
      send_command(32'h0000_0000,4'h0,3'd0);
      check(accepted&&last_sequence==0,"sequence wrap rejected");

      @(negedge clk);safety_block=1;
      @(posedge clk);#1;
      check(disarm&&!lease_active,"safety block did not revoke active lease");
      send_command(32'd1,4'h5,3'd3);
      check(rejected&&reject_reason==3'd4,"safety-blocked command not rejected");
      check(!lease_active,"safety-blocked command restored lease");
      check(last_sequence==0,"safety rejection consumed sequence");
      @(negedge clk);safety_block=0;
      send_command(32'd1,4'h5,3'd3);
      check(accepted&&manual_valid&&manual_motion==3,"safe retry not accepted");
      check(accepted_count==4&&rejected_count==1,"wrap-phase counters incorrect");

      // Arbiter conflict must never leak either motion request to the core.
      @(negedge clk);cpu_arm=1;ext_route=1;
      @(posedge clk);#1;
      check(arb_conflict&&arb_disarm,"CPU/external conflict did not disarm");
      check(!arb_arm&&!arb_route&&!arb_manual,"conflicting action leaked through");
      @(negedge clk);cpu_arm=0;ext_route=0;
      @(posedge clk);#1;
      @(negedge clk);ext_manual=1;ext_motion=3'd4;
      @(posedge clk);#1;
      check(arb_manual&&arb_motion==4&&!arb_disarm,"external manual arbitration failed");
      @(negedge clk);ext_manual=0;

      if(errors==0)$display("PASS: tb_choice3_external_command_guard");
      else begin
        $display("FAIL: tb_choice3_external_command_guard errors=%0d",errors);
        $fatal(1);
      end
      $finish;
    end
endmodule

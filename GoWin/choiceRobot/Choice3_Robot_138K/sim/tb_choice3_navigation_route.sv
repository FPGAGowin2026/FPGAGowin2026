`timescale 1ns/1ps

module tb_choice3_navigation_route;
  reg clk=0,rst_n=0,sample_tick=0,enable=1,start_pulse=0,stop_pulse=0;
  reg front_blocked=0,front_slow=0,left_blocked=0,right_blocked=0;
  reg signed [31:0] wheel0_ticks=0,wheel1_ticks=0,wheel2_ticks=0,wheel3_ticks=0;
  wire signed [31:0] target0,target1,target2,target3;
  wire [3:0] state;
  wire route_done;
  integer errors=0;

  always #5 clk=~clk;

  choice3_navigation_fsm #(
    .LEG1_TICKS(4),.TURN_TICKS(3),.LEG2_TICKS(5),
    .AVOID_BACK_SAMPLES(2),.AVOID_TURN_SAMPLES(2)
  ) u_dut (
    .clk_i(clk),.rst_n_i(rst_n),.sample_tick_i(sample_tick),.enable_i(enable),
    .start_pulse_i(start_pulse),.stop_pulse_i(stop_pulse),
    .front_blocked_i(front_blocked),.front_slow_i(front_slow),
    .left_blocked_i(left_blocked),.right_blocked_i(right_blocked),
    .wheel0_ticks_i(wheel0_ticks),.wheel1_ticks_i(wheel1_ticks),
    .wheel2_ticks_i(wheel2_ticks),.wheel3_ticks_i(wheel3_ticks),
    .target0_mm_s_o(target0),.target1_mm_s_o(target1),
    .target2_mm_s_o(target2),.target3_mm_s_o(target3),
    .state_o(state),.route_done_o(route_done)
  );

  task check(input bit condition,input string message);
    if(!condition)begin
      $display("FAIL: %s",message);
      errors=errors+1;
    end
  endtask

  task pulse_start;
    begin
      @(negedge clk);start_pulse=1;
      @(posedge clk);#1;
      @(negedge clk);start_pulse=0;
    end
  endtask

  task pulse_stop;
    begin
      @(negedge clk);stop_pulse=1;
      @(posedge clk);#1;
      @(negedge clk);stop_pulse=0;
    end
  endtask

  task publish_ticks(
    input signed [31:0] tick0,input signed [31:0] tick1,
    input signed [31:0] tick2,input signed [31:0] tick3
  );
    begin
      @(negedge clk);
      wheel0_ticks=tick0;wheel1_ticks=tick1;
      wheel2_ticks=tick2;wheel3_ticks=tick3;
      sample_tick=1;
      @(posedge clk);#1;
      @(negedge clk);sample_tick=0;
    end
  endtask

  initial begin
    repeat(3)@(posedge clk);
    @(negedge clk);rst_n=1;
    @(posedge clk);#1;
    check(state==4'd0 && !route_done,"reset did not leave navigation idle");
    check(target0==0 && target1==0 && target2==0 && target3==0,
          "idle navigation commanded motion");

    pulse_start();#1;
    check(state==4'd1 && !route_done,"start did not enter first leg");
    check(target0==32'sd260 && target1==32'sd260 &&
          target2==32'sd260 && target3==32'sd260,
          "first leg cruise target mismatch");

    front_slow=1;#1;
    check(target0==32'sd120 && target1==32'sd120 &&
          target2==32'sd120 && target3==32'sd120,
          "front_slow did not select reduced route speed");
    front_slow=0;

    publish_ticks(32'sd4,32'sd4,32'sd4,32'sd4);#1;
    check(state==4'd2,"first leg distance did not enter turn");
    check(target0==-32'sd180 && target1==-32'sd180 &&
          target2==32'sd180 && target3==32'sd180,
          "turn target signs mismatch");

    publish_ticks(32'sd1,32'sd1,32'sd7,32'sd7);#1;
    check(state==4'd3,"turn distance did not enter second leg");
    check(target0==32'sd260 && target1==32'sd260 &&
          target2==32'sd260 && target3==32'sd260,
          "second leg cruise target mismatch");

    publish_ticks(32'sd6,32'sd6,32'sd12,32'sd12);#1;
    check(state==4'd6 && route_done,"second leg did not complete the route");
    check(target0==0 && target1==0 && target2==0 && target3==0,
          "completed route did not command zero speed");

    publish_ticks(32'sd20,32'sd20,32'sd20,32'sd20);#1;
    check(state==4'd6 && route_done,"route_done was not sticky in done state");

    pulse_stop();#1;
    check(state==4'd0 && !route_done,"stop did not clear completed route");

    if(errors==0)$display("PASS: tb_choice3_navigation_route");
    else begin
      $display("FAIL: tb_choice3_navigation_route errors=%0d",errors);
      $fatal(1);
    end
    $finish;
  end

  initial begin #20_000;$display("FAIL: timeout");$fatal(1);end
endmodule

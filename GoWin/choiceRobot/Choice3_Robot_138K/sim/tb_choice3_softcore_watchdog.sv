`timescale 1ns/1ps

module tb_choice3_softcore_watchdog;
  reg clk=0,safety_reset_n=0,heartbeat_write=0;
  wire alive,fault,allow_commit,allow_start,path_abort;
  integer errors=0;

  always #5 clk=~clk;

  softcore_service_watchdog #(.TIMEOUT_CYCLES(4)) u_dut (
    .clk_i(clk),.safety_reset_n_i(safety_reset_n),
    .heartbeat_write_i(heartbeat_write),.alive_o(alive),.fault_o(fault),
    .allow_commit_o(allow_commit),.allow_start_o(allow_start),
    .path_abort_o(path_abort)
  );

  task check(input bit condition,input string message);
    if(!condition)begin
      $display("FAIL: %s",message);
      errors=errors+1;
    end
  endtask

  task pulse_heartbeat;
    begin
      @(negedge clk);heartbeat_write=1;
      @(posedge clk);#1;
      @(negedge clk);heartbeat_write=0;
    end
  endtask

  initial begin
    repeat(3)@(posedge clk);
    check(!alive && !fault && !allow_commit && !allow_start && !path_abort,
          "safety reset did not clear watchdog outputs");

    @(negedge clk);safety_reset_n=1;
    repeat(6)begin
      @(posedge clk);#1;
      check(!alive && !fault && !allow_commit && !allow_start && !path_abort,
            "watchdog started before its first heartbeat");
    end

    pulse_heartbeat();#1;
    check(alive && !fault && allow_commit && allow_start && !path_abort,
          "first heartbeat did not authorize service actions");

    repeat(3)begin
      @(posedge clk);#1;
      check(alive && !fault && allow_commit && allow_start && !path_abort,
            "watchdog expired before TIMEOUT_CYCLES");
    end
    @(posedge clk);#1;
    check(!alive && fault && !allow_commit && !allow_start && path_abort,
          "timeout did not revoke authorization and pulse path_abort");

    @(posedge clk);#1;
    check(!alive && fault && !allow_commit && !allow_start && !path_abort,
          "fault was not latched or path_abort was not one cycle");

    pulse_heartbeat();#1;
    check(!alive && fault && !allow_commit && !allow_start && !path_abort,
          "heartbeat improperly cleared a latched watchdog fault");

    @(negedge clk);#2;safety_reset_n=0;#1;
    check(!alive && !fault && !allow_commit && !allow_start && !path_abort,
          "asynchronous safety reset did not clear latched fault");
    @(negedge clk);safety_reset_n=1;
    pulse_heartbeat();#1;
    check(alive && !fault && allow_commit && allow_start && !path_abort,
          "watchdog did not recover after safety reset and fresh heartbeat");

    if(errors==0)$display("PASS: tb_choice3_softcore_watchdog");
    else begin
      $display("FAIL: tb_choice3_softcore_watchdog errors=%0d",errors);
      $fatal(1);
    end
    $finish;
  end

  initial begin #20_000;$display("FAIL: timeout");$fatal(1);end
endmodule

`timescale 1ns/1ps
module tb_vl53l0x;
  reg clk=0,reset_n=0;tri1 scl,sda;reg slave_sda_low=0;
  wire sensor_ok,initialized,identity_error,bus_error,timeout_error,distance_valid;
  wire[7:0]model_id,range_status_raw;wire[2:0]interrupt_status;wire[15:0]distance_mm;
  integer errors=0,samples=0,n;integer saw_identity_error=0;reg[7:0]received;
  // 与 DUT 完全分离的黄金调参表；TB 不调用 DUT 内部函数，也不读取其 op_index。
  reg[15:0]golden_tune[0:79];
  assign sda=slave_sda_low?1'b0:1'bz;always #5 clk=~clk;
  always @(posedge distance_valid)samples=samples+1;
  always @(posedge identity_error)saw_identity_error=1;
  sensor_example_top #(.I2C_CLK_DIV(2),.I2C_STRETCH_TIMEOUT_TICKS(16),
    .STARTUP_DELAY_CYCLES(4),.SAMPLE_INTERVAL_CYCLES(8),.POLL_RETRY_LIMIT(8))dut(
    .clk(clk),.reset_n(reset_n),.tof_scl(scl),.tof_sda(sda),.sensor_ok(sensor_ok),
    .initialized(initialized),.identity_error(identity_error),.bus_error(bus_error),
    .timeout_error(timeout_error),.model_id(model_id),.interrupt_status(interrupt_status),
    .range_status_raw(range_status_raw),
    .distance_mm(distance_mm),.distance_valid(distance_valid));
  task wait_start;begin @(negedge sda);if(scl!==1)begin $display("ERROR: START");errors=errors+1;end end endtask
  task recv_byte(output reg[7:0]v);integer b;begin v=0;for(b=7;b>=0;b=b-1)begin @(posedge scl);v[b]=sda;end end endtask
  task ack;begin @(negedge scl);slave_sda_low=1;@(posedge scl);@(negedge scl);slave_sda_low=0;end endtask
  task expect_byte(input[7:0]v);begin recv_byte(received);if(received!==v)begin
    $display("ERROR: expected %02h got %02h",v,received);errors=errors+1;end ack;end endtask
  task stop;begin @(posedge scl);if(sda!==0)begin $display("ERROR: STOP");errors=errors+1;end @(posedge sda);end endtask
  task send(input[7:0]v,input exp_ack);integer b;reg a;begin
    for(b=7;b>=0;b=b-1)begin slave_sda_low=~v[b];@(posedge scl);@(negedge scl);end
    slave_sda_low=0;@(posedge scl);a=(sda===0);@(negedge scl);
    if(a!==exp_ack)begin $display("ERROR: master ACK");errors=errors+1;end end endtask
  task write1(input[7:0]r,input[7:0]v);begin wait_start;expect_byte(8'h52);expect_byte(r);expect_byte(v);stop;end endtask
  task write2(input[7:0]r,input[15:0]v);begin wait_start;expect_byte(8'h52);expect_byte(r);expect_byte(v[15:8]);expect_byte(v[7:0]);stop;end endtask
  task read1(input[7:0]r,input[7:0]v);begin wait_start;expect_byte(8'h52);expect_byte(r);wait_start;expect_byte(8'h53);send(v,0);stop;end endtask
  task read2(input[7:0]r,input[15:0]v);begin wait_start;expect_byte(8'h52);expect_byte(r);wait_start;expect_byte(8'h53);send(v[15:8],1);send(v[7:0],0);stop;end endtask
  task read6(input[7:0]r,input[47:0]v);integer j;begin wait_start;expect_byte(8'h52);expect_byte(r);wait_start;expect_byte(8'h53);
    for(j=5;j>=0;j=j-1)send(v[j*8+:8],j!=0);stop;end endtask
  task nack_address;begin wait_start;recv_byte(received);if(received!==8'h52)errors=errors+1;
    @(posedge scl);@(negedge scl);stop;end endtask
  task set_golden_tune(input integer index,input[7:0]reg_addr,input[7:0]value);begin
    golden_tune[index]={reg_addr,value};
  end endtask

  initial begin
    set_golden_tune(0,8'hFF,8'h01);set_golden_tune(1,8'h00,8'h00);
    set_golden_tune(2,8'hFF,8'h00);set_golden_tune(3,8'h09,8'h00);
    set_golden_tune(4,8'h10,8'h00);set_golden_tune(5,8'h11,8'h00);
    set_golden_tune(6,8'h24,8'h01);set_golden_tune(7,8'h25,8'hFF);
    set_golden_tune(8,8'h75,8'h00);set_golden_tune(9,8'hFF,8'h01);
    set_golden_tune(10,8'h4E,8'h2C);set_golden_tune(11,8'h48,8'h00);
    set_golden_tune(12,8'h30,8'h20);set_golden_tune(13,8'hFF,8'h00);
    set_golden_tune(14,8'h30,8'h09);set_golden_tune(15,8'h54,8'h00);
    set_golden_tune(16,8'h31,8'h04);set_golden_tune(17,8'h32,8'h03);
    set_golden_tune(18,8'h40,8'h83);set_golden_tune(19,8'h46,8'h25);
    set_golden_tune(20,8'h60,8'h00);set_golden_tune(21,8'h27,8'h00);
    set_golden_tune(22,8'h50,8'h06);set_golden_tune(23,8'h51,8'h00);
    set_golden_tune(24,8'h52,8'h96);set_golden_tune(25,8'h56,8'h08);
    set_golden_tune(26,8'h57,8'h30);set_golden_tune(27,8'h61,8'h00);
    set_golden_tune(28,8'h62,8'h00);set_golden_tune(29,8'h64,8'h00);
    set_golden_tune(30,8'h65,8'h00);set_golden_tune(31,8'h66,8'hA0);
    set_golden_tune(32,8'hFF,8'h01);set_golden_tune(33,8'h22,8'h32);
    set_golden_tune(34,8'h47,8'h14);set_golden_tune(35,8'h49,8'hFF);
    set_golden_tune(36,8'h4A,8'h00);set_golden_tune(37,8'hFF,8'h00);
    set_golden_tune(38,8'h7A,8'h0A);set_golden_tune(39,8'h7B,8'h00);
    set_golden_tune(40,8'h78,8'h21);set_golden_tune(41,8'hFF,8'h01);
    set_golden_tune(42,8'h23,8'h34);set_golden_tune(43,8'h42,8'h00);
    set_golden_tune(44,8'h44,8'hFF);set_golden_tune(45,8'h45,8'h26);
    set_golden_tune(46,8'h46,8'h05);set_golden_tune(47,8'h40,8'h40);
    set_golden_tune(48,8'h0E,8'h06);set_golden_tune(49,8'h20,8'h1A);
    set_golden_tune(50,8'h43,8'h40);set_golden_tune(51,8'hFF,8'h00);
    set_golden_tune(52,8'h34,8'h03);set_golden_tune(53,8'h35,8'h44);
    set_golden_tune(54,8'hFF,8'h01);set_golden_tune(55,8'h31,8'h04);
    set_golden_tune(56,8'h4B,8'h09);set_golden_tune(57,8'h4C,8'h05);
    set_golden_tune(58,8'h4D,8'h04);set_golden_tune(59,8'hFF,8'h00);
    set_golden_tune(60,8'h44,8'h00);set_golden_tune(61,8'h45,8'h20);
    set_golden_tune(62,8'h47,8'h08);set_golden_tune(63,8'h48,8'h28);
    set_golden_tune(64,8'h67,8'h00);set_golden_tune(65,8'h70,8'h04);
    set_golden_tune(66,8'h71,8'h01);set_golden_tune(67,8'h72,8'hFE);
    set_golden_tune(68,8'h76,8'h00);set_golden_tune(69,8'h77,8'h00);
    set_golden_tune(70,8'hFF,8'h01);set_golden_tune(71,8'h0D,8'h01);
    set_golden_tune(72,8'hFF,8'h00);set_golden_tune(73,8'h80,8'h01);
    set_golden_tune(74,8'h01,8'hF8);set_golden_tune(75,8'hFF,8'h01);
    set_golden_tune(76,8'h8E,8'h01);set_golden_tune(77,8'h00,8'h01);
    set_golden_tune(78,8'hFF,8'h00);set_golden_tune(79,8'h80,8'h00);
    wait(reset_n);
    read1(8'hC0,8'h00);read1(8'hC0,8'hEE);
    read1(8'h89,8'h00);write1(8'h89,8'h01);write1(8'h88,8'h00);
    write1(8'h80,8'h01);write1(8'hFF,8'h01);write1(8'h00,8'h00);
    read1(8'h91,8'hAB);write1(8'h00,8'h01);write1(8'hFF,8'h00);
    write1(8'h80,8'h00);read1(8'h60,8'h00);write1(8'h60,8'h12);
    write2(8'h44,16'h0020);write1(8'h01,8'hFF);
    write1(8'h80,1);write1(8'hFF,1);write1(8'h00,0);write1(8'hFF,6);
    read1(8'h83,0);write1(8'h83,4);write1(8'hFF,7);write1(8'h81,1);
    write1(8'h80,1);write1(8'h94,8'h6B);write1(8'h83,0);
    read1(8'h83,0);read1(8'h83,1);write1(8'h83,1);read1(8'h92,8'h8C);
    write1(8'h81,0);write1(8'hFF,6);read1(8'h83,4);write1(8'h83,0);
    write1(8'hFF,1);write1(8'h00,1);write1(8'hFF,0);write1(8'h80,0);
    read6(8'hB0,48'hFFFFFFFFFFFF);
    write1(8'hFF,1);write1(8'h4F,0);write1(8'h4E,8'h2C);write1(8'hFF,0);write1(8'hB6,8'hB4);
    write1(8'hB0,8'h00);write1(8'hB1,8'hF0);write1(8'hB2,8'hFF);
    write1(8'hB3,8'h00);write1(8'hB4,8'h00);write1(8'hB5,8'h00);
    for(n=0;n<80;n=n+1)write1(golden_tune[n][15:8],golden_tune[n][7:0]);
    write1(8'h0A,4);read1(8'h84,8'hFF);write1(8'h84,8'hEF);write1(8'h0B,1);write1(8'h01,8'hE8);
    write1(8'h01,1);write1(8'h00,8'h41);read1(8'h13,0);read1(8'h13,7);
    write1(8'h0B,1);write1(8'h00,0);
    write1(8'h01,2);write1(8'h00,1);read1(8'h13,0);read1(8'h13,7);
    write1(8'h0B,1);write1(8'h00,0);write1(8'h01,8'hE8);
    write1(8'h80,1);write1(8'hFF,1);write1(8'h00,0);write1(8'h91,8'hAB);
    write1(8'h00,1);write1(8'hFF,0);write1(8'h80,0);write1(8'h00,2);
    read1(8'h13,0);read1(8'h13,3);read1(8'h14,8'h58);
    read2(8'h1E,16'd500);write1(8'h0B,1);
    nack_address;
  end
  initial begin
    #30 reset_n=1;@(posedge distance_valid);#1;
    if(!sensor_ok||!initialized||identity_error||timeout_error)begin $display("ERROR: flags");errors=errors+1;end
    if(model_id!==8'hEE||interrupt_status!==3||range_status_raw!==8'h58||distance_mm!==500)begin $display("ERROR: result");errors=errors+1;end
    if(saw_identity_error!=1)begin $display("ERROR: ID retry not exercised");errors=errors+1;end
    wait(bus_error);#1;if(samples!=1)begin $display("ERROR: error transaction published");errors=errors+1;end
    if(errors==0)$display("PASS: VL53L0X independent golden tuning, raw range status, continuous 500-mm read, and NACK handling");
    else $fatal(1,"FAIL: %0d errors",errors);#20 $finish;
  end
  initial #8000000 $fatal(1,"FAIL: timeout");
endmodule

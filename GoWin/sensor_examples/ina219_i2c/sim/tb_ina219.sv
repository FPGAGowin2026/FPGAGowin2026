`timescale 1ns/1ps

module tb_ina219;
    reg clk=0,reset_n=0;
    tri1 scl;tri1 sda;
    reg slave_sda_low=0;
    reg slave_scl_low=0;
    assign sda=slave_sda_low?1'b0:1'bz;
    assign scl=slave_scl_low?1'b0:1'bz;
    wire sensor_ok,initialized,bus_error,sample_valid;
    wire signed [15:0] shunt_voltage_raw;
    wire [15:0] bus_voltage_raw,bus_voltage_mv;
    integer errors=0;
    integer valid_samples=0;
    reg [7:0] received;
    always #5 clk=~clk;
    always @(posedge sample_valid) valid_samples=valid_samples+1;

    sensor_example_top #(.I2C_CLK_DIV(2),.STARTUP_DELAY_CYCLES(4),.SAMPLE_INTERVAL_CYCLES(8)) dut(
      .clk(clk),.reset_n(reset_n),.ina_scl(scl),.ina_sda(sda),.sensor_ok(sensor_ok),
      .initialized(initialized),.bus_error(bus_error),.shunt_voltage_raw(shunt_voltage_raw),
      .bus_voltage_raw(bus_voltage_raw),.bus_voltage_mv(bus_voltage_mv),.sample_valid(sample_valid));

    task wait_start;begin @(negedge sda);if(scl!==1'b1)begin $display("ERROR: START");errors=errors+1;end end endtask
    task recv_byte(output reg[7:0]data);integer b;begin data=0;for(b=7;b>=0;b=b-1)begin @(posedge scl);data[b]=sda;end end endtask
    task slave_ack;begin @(negedge scl);slave_sda_low=1;@(posedge scl);@(negedge scl);slave_sda_low=0;end endtask
    task expect_byte(input[7:0]expected);begin recv_byte(received);if(received!==expected)begin
      $display("ERROR: expected %02h got %02h",expected,received);errors=errors+1;end slave_ack;end endtask
    task wait_stop;begin @(posedge scl);if(sda!==0)begin $display("ERROR: STOP setup");errors=errors+1;end @(posedge sda);end endtask
    task send_byte(input[7:0]data,input expected_ack);integer b;reg ack;begin
      for(b=7;b>=0;b=b-1)begin slave_sda_low=~data[b];@(posedge scl);@(negedge scl);end
      slave_sda_low=0;@(posedge scl);ack=(sda===0);@(negedge scl);
      if(ack!==expected_ack)begin $display("ERROR: ACK expected %0d got %0d",expected_ack,ack);errors=errors+1;end
    end endtask
    task serve_write2(input[7:0]reg_addr,input[15:0]data);begin
      wait_start;expect_byte(8'h80);expect_byte(reg_addr);expect_byte(data[15:8]);expect_byte(data[7:0]);wait_stop;
    end endtask
    task serve_read2(input[7:0]reg_addr,input[15:0]data);begin
      wait_start;expect_byte(8'h80);expect_byte(reg_addr);wait_start;expect_byte(8'h81);
      send_byte(data[15:8],1);send_byte(data[7:0],0);wait_stop;
    end endtask
    task serve_read2_mid_read_timeout(input[7:0]reg_addr);begin
      wait_start;expect_byte(8'h80);expect_byte(reg_addr);wait_start;expect_byte(8'h81);
      // 先完成一个数据位，再把 SCL 持续拉低，命中 P_R_RAISE/P_MACK 读阶段超时。
      slave_sda_low=0;@(posedge scl);@(negedge scl);slave_scl_low=1;
      wait(bus_error);slave_scl_low=0;slave_sda_low=0;
    end endtask

    initial begin
      wait(reset_n);
      serve_write2(8'h00,16'h399F);
      serve_read2(8'h00,16'h399F);
      serve_read2(8'h01,16'hFF9C);
      serve_read2(8'h02,16'h5DC2);
      serve_read2_mid_read_timeout(8'h01);
    end
    initial begin
      #30 reset_n=1;@(posedge sample_valid);#1;
      if(!sensor_ok||!initialized)begin $display("ERROR: initialization");errors=errors+1;end
      if(bus_error)begin $display("ERROR: bus_error");errors=errors+1;end
      if(shunt_voltage_raw!==16'shFF9C)begin $display("ERROR: shunt=%h",shunt_voltage_raw);errors=errors+1;end
      if(bus_voltage_raw!==16'h5DC2)begin $display("ERROR: bus raw=%h",bus_voltage_raw);errors=errors+1;end
      if(bus_voltage_mv!==16'd12000)begin $display("ERROR: bus mV=%0d",bus_voltage_mv);errors=errors+1;end
      wait(bus_error);#1;
      if(!bus_error)begin $display("ERROR: missing stuck-SCL timeout");errors=errors+1;end
      if(valid_samples!==1)begin $display("ERROR: partial read published as a sample");errors=errors+1;end
      if(errors==0)$display("PASS: INA219 open-drain I2C configuration, voltage reads, and bus timeout");
      else $fatal(1,"FAIL: %0d errors",errors);#20 $finish;
    end
    initial #1000000 $fatal(1,"FAIL: timeout");
endmodule

`timescale 1ns/1ps

module tb_vl53l1x;
    reg clk=0, reset_n=0;
    tri1 scl;
    tri1 sda;
    reg slave_sda_low=0;
    assign sda=slave_sda_low?1'b0:1'bz;
    wire initialized,bus_error,distance_valid;
    wire [15:0] distance_mm;
    wire [4:0] range_status;
    integer errors=0;
    integer cfg_index;
    reg [7:0] received;
    localparam [727:0] DEFAULT_CONFIG = 728'h0000000102000208000810010100000000FF000F0000000000200B0000020A2100000500000000C8000038FF010008000001CC0F01F10D0168008008B8000000000F890000000000000000010F0D0E0E000002C7FF9B0000000100;

    function [7:0] expected_config_byte;
        input integer index;
        begin
            expected_config_byte = DEFAULT_CONFIG[(90-index)*8 +: 8];
        end
    endfunction

    always #5 clk=~clk;
    sensor_example_top #(.I2C_CLK_DIV(2),.STARTUP_DELAY_CYCLES(4),.RESET_DELAY_CYCLES(4),
          .SAMPLE_INTERVAL_CYCLES(8)) dut (
        .clk(clk),.reset_n(reset_n),.tof_scl(scl),.tof_sda(sda),
        .initialized(initialized),.bus_error(bus_error),
        .range_status(range_status),
        .distance_mm(distance_mm),.distance_valid(distance_valid)
    );

    task wait_start;
        begin
            @(negedge sda);
            if(scl!==1'b1) begin $display("ERROR: START while SCL low"); errors=errors+1; end
        end
    endtask
    task recv_byte(output reg [7:0] data);
        integer bit_no;
        begin
            data=0;
            for(bit_no=7;bit_no>=0;bit_no=bit_no-1) begin
                @(posedge scl); data[bit_no]=sda;
            end
        end
    endtask
    task slave_ack;
        begin
            @(negedge scl); slave_sda_low=1;
            @(posedge scl); @(negedge scl); slave_sda_low=0;
        end
    endtask
    task wait_stop;
        begin
            @(posedge scl);
            if(sda!==1'b0) begin $display("ERROR: STOP setup missing"); errors=errors+1; end
            @(posedge sda);
        end
    endtask
    task expect_byte(input [7:0] expected);
        begin
            recv_byte(received);
            if(received!==expected) begin
                $display("ERROR: expected byte %02h, got %02h",expected,received);
                errors=errors+1;
            end
            slave_ack;
        end
    endtask
    task send_byte(input [7:0] data,input expected_master_ack);
        integer bit_no;
        reg observed_ack;
        begin
            for(bit_no=7;bit_no>=0;bit_no=bit_no-1) begin
                slave_sda_low=~data[bit_no];
                @(posedge scl); @(negedge scl);
            end
            slave_sda_low=0;
            @(posedge scl); observed_ack=(sda===1'b0); @(negedge scl);
            if(observed_ack!==expected_master_ack) begin
                $display("ERROR: master ACK expected %0d got %0d",expected_master_ack,observed_ack);
                errors=errors+1;
            end
        end
    endtask
    task serve_write(input [15:0] reg_addr,input [7:0] data);
        begin
            wait_start; expect_byte(8'h52); expect_byte(reg_addr[15:8]); expect_byte(reg_addr[7:0]);
            expect_byte(data); wait_stop;
        end
    endtask
    task serve_read1(input [15:0] reg_addr,input [7:0] data0);
        begin
            wait_start; expect_byte(8'h52); expect_byte(reg_addr[15:8]); expect_byte(reg_addr[7:0]);
            wait_start; expect_byte(8'h53); send_byte(data0,1'b0); wait_stop;
        end
    endtask
    task serve_read2(input [15:0] reg_addr,input [7:0] data0,input [7:0] data1);
        begin
            wait_start; expect_byte(8'h52); expect_byte(reg_addr[15:8]); expect_byte(reg_addr[7:0]);
            wait_start; expect_byte(8'h53); send_byte(data0,1'b1); send_byte(data1,1'b0); wait_stop;
        end
    endtask

    initial begin
        wait(reset_n);
        serve_write(16'h0000,8'h00);
        serve_write(16'h0000,8'h01);
        // 第一次 boot-state 轮询未就绪，第二次就绪，验证不是固定延时盲写。
        serve_read1(16'h00E5,8'h00);
        serve_read1(16'h00E5,8'h01);
        for(cfg_index=0;cfg_index<91;cfg_index=cfg_index+1)
            serve_write(16'h002D+cfg_index,expected_config_byte(cfg_index));
        // ULD SensorInit 首次测量与 VHV 收尾。
        serve_write(16'h0087,8'h40);
        serve_read1(16'h0031,8'h00);
        serve_read1(16'h0031,8'h01);
        serve_write(16'h0086,8'h01);
        serve_write(16'h0087,8'h00);
        serve_write(16'h0008,8'h09);
        serve_write(16'h000B,8'h00);
        serve_write(16'h0087,8'h40);
        // 第一帧 range status 无效：必须清中断但不能发布 distance_valid。
        serve_read1(16'h0031,8'h01);
        serve_read1(16'h0089,8'h04);
        serve_write(16'h0086,8'h01);
        // 第二帧 raw status=9 才允许读取并发布 500 mm。
        serve_read1(16'h0031,8'h01);
        serve_read1(16'h0089,8'h09);
        serve_read2(16'h0096,8'h01,8'hF4);
        serve_write(16'h0086,8'h01);
    end

    initial begin
        #30 reset_n=1;
        @(posedge distance_valid); #1;
        if(!initialized) begin $display("ERROR: not initialized"); errors=errors+1; end
        if(bus_error) begin $display("ERROR: I2C bus_error asserted"); errors=errors+1; end
        if(range_status!==5'd9) begin $display("ERROR: range_status=%0d",range_status); errors=errors+1; end
        if(distance_mm!==16'd500) begin $display("ERROR: distance=%0d",distance_mm); errors=errors+1; end
        if(errors==0) $display("PASS: VL53L1X open-drain I2C initialization and distance read");
        else $fatal(1,"FAIL: %0d errors",errors);
        #20 $finish;
    end
    initial #1000000 $fatal(1,"FAIL: timeout");
endmodule

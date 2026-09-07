`timescale 1ns/1ps

module tb_mpu6050;
    localparam integer WAKE_WAIT_CYCLES = 20;
    reg clk=0, reset_n=0;
    tri1 scl, sda;
    reg slave_sda_low=0;
    wire sensor_ok, initialized, identity_error, bus_error, sample_valid;
    reg [2:0] sample_select=0;
    wire [15:0] sample_word;
    integer errors=0;
    integer samples=0;
    integer clk_cycles=0;
    integer config_writes=0;
    integer status_reads=0;
    integer burst_reads=0;
    integer pwr2_stop_cycle=-1;
    integer first_post_wake_start_cycle=-1;
    integer saw_identity_error=0;
    reg [7:0] received;
    integer k;
    reg [7:0] burst [0:13];
    assign sda = slave_sda_low ? 1'b0 : 1'bz;
    always #5 clk=~clk;
    always @(posedge clk) clk_cycles=clk_cycles+1;
    always @(posedge sample_valid) begin
        samples=samples+1;
        if (!initialized) begin
            $display("ERROR: sample_valid before initialized"); errors=errors+1;
        end
    end
    always @(posedge identity_error) saw_identity_error=1;

    sensor_example_top #(
        .I2C_CLK_DIV(2), .I2C_STRETCH_TIMEOUT_TICKS(16),
        .STARTUP_DELAY_CYCLES(4), .WAKE_STABILIZE_CYCLES(WAKE_WAIT_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(8)
    ) dut (
        .clk(clk),.reset_n(reset_n),.imu_scl(scl),.imu_sda(sda),
        .sensor_ok(sensor_ok),.initialized(initialized),
        .identity_error(identity_error),.bus_error(bus_error),
        .sample_select(sample_select),.sample_word(sample_word),.sample_valid(sample_valid)
    );

    task wait_start; begin
        @(negedge sda);
        if (scl!==1'b1) begin $display("ERROR: START while SCL low"); errors=errors+1; end
    end endtask
    task recv_byte(output reg [7:0] data); integer b; begin
        data=0; for(b=7;b>=0;b=b-1) begin @(posedge scl); data[b]=sda; end
    end endtask
    task slave_ack; begin
        @(negedge scl); slave_sda_low=1; @(posedge scl); @(negedge scl); slave_sda_low=0;
    end endtask
    task expect_byte(input [7:0] expected); begin
        recv_byte(received);
        if(received!==expected) begin
            $display("ERROR: expected %02h got %02h",expected,received); errors=errors+1;
        end
        slave_ack;
    end endtask
    task wait_stop; begin
        @(posedge scl); if(sda!==1'b0) begin $display("ERROR: STOP setup"); errors=errors+1; end
        @(posedge sda);
    end endtask
    task send_byte(input [7:0] data,input expected_ack); integer b; reg ack; begin
        for(b=7;b>=0;b=b-1) begin slave_sda_low=~data[b]; @(posedge scl); @(negedge scl); end
        slave_sda_low=0; @(posedge scl); ack=(sda===1'b0); @(negedge scl);
        if(ack!==expected_ack) begin
            $display("ERROR: ACK expected %0d got %0d",expected_ack,ack); errors=errors+1;
        end
    end endtask
    task serve_write1(input [7:0] reg_addr,input [7:0] value); begin
        wait_start;
        if(reg_addr==8'h19 && pwr2_stop_cycle>=0) begin
            first_post_wake_start_cycle=clk_cycles;
            if(first_post_wake_start_cycle-pwr2_stop_cycle < WAKE_WAIT_CYCLES) begin
                $display("ERROR: wake stabilization only %0d cycles, expected >=%0d",
                    first_post_wake_start_cycle-pwr2_stop_cycle,WAKE_WAIT_CYCLES);
                errors=errors+1;
            end
        end
        expect_byte(8'hD0); expect_byte(reg_addr); expect_byte(value); wait_stop;
        config_writes=config_writes+1;
        if(reg_addr==8'h6C) pwr2_stop_cycle=clk_cycles;
    end endtask
    task serve_read1(input [7:0] reg_addr,input [7:0] value); begin
        wait_start; expect_byte(8'hD0); expect_byte(reg_addr);
        wait_start; expect_byte(8'hD1); send_byte(value,0); wait_stop;
        if(reg_addr==8'h3A) status_reads=status_reads+1;
    end endtask
    task serve_read14(input [7:0] reg_addr); begin
        wait_start; expect_byte(8'hD0); expect_byte(reg_addr);
        wait_start; expect_byte(8'hD1);
        for(k=0;k<14;k=k+1) send_byte(burst[k], k<13);
        wait_stop;
        burst_reads=burst_reads+1;
    end endtask
    task nack_next_address; begin
        wait_start; recv_byte(received);
        if(received!==8'hD0) begin $display("ERROR: expected final address"); errors=errors+1; end
        // SDA 保持释放，地址 ACK 位为 NACK。
        @(posedge scl); @(negedge scl); wait_stop;
    end endtask

    initial begin
        burst[0]=8'h01; burst[1]=8'h02;
        burst[2]=8'hFF; burst[3]=8'hFE;
        burst[4]=8'h7F; burst[5]=8'hFF;
        burst[6]=8'h00; burst[7]=8'h00;
        burst[8]=8'h12; burst[9]=8'h34;
        burst[10]=8'hAB; burst[11]=8'hCD;
        burst[12]=8'h80; burst[13]=8'h00;
        wait(reset_n);
        // 身份不匹配不得开始配置；下一轮身份正确后继续。
        serve_read1(8'h75,8'h00);
        serve_read1(8'h75,8'h68);
        serve_write1(8'h6B,8'h01);
        serve_write1(8'h6C,8'h00);
        serve_write1(8'h19,8'h04);
        serve_write1(8'h1A,8'h03);
        serve_write1(8'h1B,8'h00);
        serve_write1(8'h1C,8'h00);
        serve_write1(8'h38,8'h01);
        // 数据未就绪时不允许发布旧样本。
        serve_read1(8'h3A,8'h00);
        serve_read1(8'h3A,8'h01);
        serve_read14(8'h3B);
        // 下一次轮询用 NACK 验证错误路径。
        nack_next_address;
    end

    initial begin
        #30 reset_n=1;
        @(posedge sample_valid); #1;
        if(!sensor_ok||!initialized||identity_error) begin
            $display("ERROR: initialization flags"); errors=errors+1;
        end
        if(saw_identity_error!=1) begin $display("ERROR: identity retry not exercised");errors=errors+1;end
        if(config_writes!=7) begin $display("ERROR: expected 7 config writes, got %0d",config_writes);errors=errors+1;end
        if(pwr2_stop_cycle<0||first_post_wake_start_cycle<0) begin $display("ERROR: wake wait not observed");errors=errors+1;end
        if(status_reads!=2||burst_reads!=1) begin
            $display("ERROR: expected ready polling 2 times and one atomic burst, got %0d/%0d",status_reads,burst_reads);
            errors=errors+1;
        end
        sample_select=0;#1;if(sample_word!==16'h0102)begin $display("ERROR: accel_x");errors=errors+1;end
        sample_select=1;#1;if(sample_word!==16'hFFFE)begin $display("ERROR: accel_y");errors=errors+1;end
        sample_select=2;#1;if(sample_word!==16'h7FFF)begin $display("ERROR: accel_z");errors=errors+1;end
        sample_select=3;#1;if(sample_word!==16'h0000)begin $display("ERROR: temperature");errors=errors+1;end
        sample_select=4;#1;if(sample_word!==16'h1234)begin $display("ERROR: gyro_x");errors=errors+1;end
        sample_select=5;#1;if(sample_word!==16'hABCD)begin $display("ERROR: gyro_y");errors=errors+1;end
        sample_select=6;#1;if(sample_word!==16'h8000)begin $display("ERROR: gyro_z");errors=errors+1;end
        wait(bus_error); #1;
        if(samples!==1) begin $display("ERROR: partial/error transaction published"); errors=errors+1; end
        if(errors==0) $display("PASS: MPU6050 identity, PWR_MGMT_2 + wake stabilization, atomic 14-byte sample, and NACK handling");
        else $fatal(1,"FAIL: %0d errors",errors);
        #20 $finish;
    end
    initial #1500000 $fatal(1,"FAIL: timeout");
endmodule

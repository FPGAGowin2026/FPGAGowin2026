`timescale 1ns/1ps

module tb_lis3mdl;
    reg clk=0, reset_n=0, miso=0;
    wire cs_n, sclk, mosi, sensor_ok, initialized, sample_valid;
    wire signed [15:0] mag_x, mag_y, mag_z;
    reg [7:0] regs [0:63];
    reg [7:0] cmd, value;
    reg [5:0] addr;
    reg byte_completed;
    integer i, errors=0;
    reg first_edge_seen;
    integer status_reads=0;
    reg premature_sample=1'b0;
    always #5 clk=~clk;

    // 每次 CS 拉低后，LIS3MDL 的空闲 SCLK 必须为高，首个有效边沿必须是下降沿。
    always @(negedge cs_n) begin
        first_edge_seen = 1'b0;
        if (sclk !== 1'b1) begin
            $display("ERROR: LIS3MDL SCLK is not idle-high");
            errors = errors + 1;
        end
        fork : check_first_spi_edge
            begin
                @(negedge sclk);
                if (!first_edge_seen) first_edge_seen = 1'b1;
            end
            begin
                @(posedge sclk);
                if (!first_edge_seen) begin
                    $display("ERROR: LIS3MDL first clock edge is not falling");
                    errors = errors + 1;
                    first_edge_seen = 1'b1;
                end
            end
        join_any
        disable check_first_spi_edge;
    end

    // 100 MHz TB 时钟下得到 10 MHz SCLK，符合 LIS3MDL 上限。
    sensor_example_top #(.CLK_DIV(5), .STARTUP_DELAY_CYCLES(4), .SAMPLE_INTERVAL_CYCLES(8)) dut (
        .clk(clk), .reset_n(reset_n), .mag_miso(miso),
        .mag_cs_n(cs_n), .mag_sclk(sclk), .mag_mosi(mosi),
        .sensor_ok(sensor_ok), .initialized(initialized),
        .mag_x(mag_x), .mag_y(mag_y), .mag_z(mag_z), .sample_valid(sample_valid)
    );

    task spi_recv_byte(output reg [7:0] data);
        integer bit_no;
        begin
            data=0;
            for (bit_no=7; bit_no>=0; bit_no=bit_no-1) begin
                @(posedge sclk); data[bit_no]=mosi;
            end
        end
    endtask
    task spi_send_byte(input [7:0] data);
        integer bit_no;
        begin
            for (bit_no=7; bit_no>=0; bit_no=bit_no-1) begin
                @(negedge sclk); miso=data[bit_no];
            end
        end
    endtask

    initial begin
        for(i=0;i<64;i=i+1) regs[i]=0;
        regs[6'h0F]=8'h3D;
        regs[6'h27]=8'h00;
        regs[6'h28]=8'h34; regs[6'h29]=8'h12;
        regs[6'h2A]=8'h00; regs[6'h2B]=8'h80;
        regs[6'h2C]=8'h55; regs[6'h2D]=8'hAA;
        forever begin
            @(negedge cs_n);
            spi_recv_byte(cmd);
            addr=cmd[5:0];
            if(cmd[7]) begin
                while(!cs_n) begin
                    byte_completed=1'b0;
                    fork : send_or_chip_select_end
                        begin
                            spi_send_byte(regs[addr]);
                            byte_completed=1'b1;
                        end
                        begin
                            @(posedge cs_n);
                        end
                    join_any
                    disable send_or_chip_select_end;
                    if(byte_completed && cmd[6]) addr=addr+1'b1;
                end
                if (cmd[5:0] == 6'h27) begin
                    status_reads=status_reads+1;
                    // 第一次状态轮询返回未就绪，第二次起置位 ZYXDA。
                    if(status_reads==1) regs[6'h27]=8'h08;
                end
            end else begin
                spi_recv_byte(value); regs[addr]=value; @(posedge cs_n);
            end
            miso=0;
        end
    end

    initial begin
        #30 reset_n=1;
        wait(initialized);
        if(!sensor_ok) begin $display("ERROR: WHO_AM_I"); errors=errors+1; end
        if(regs[6'h20]!==8'h70) begin $display("ERROR: CTRL1"); errors=errors+1; end
        if(regs[6'h21]!==8'h00) begin $display("ERROR: CTRL2"); errors=errors+1; end
        if(regs[6'h22]!==8'h00) begin $display("ERROR: CTRL3"); errors=errors+1; end
        if(regs[6'h23]!==8'h0C) begin $display("ERROR: CTRL4"); errors=errors+1; end
        if(regs[6'h24]!==8'h40) begin $display("ERROR: CTRL5"); errors=errors+1; end
        @(posedge sample_valid); #1;
        if(status_reads<2 || premature_sample) begin
            $display("ERROR: sample published without observing STATUS_REG.ZYXDA");
            errors=errors+1;
        end
        if(mag_x!==16'sh1234) begin $display("ERROR: mag_x=%h",mag_x); errors=errors+1; end
        if(mag_y!==16'sh8000) begin $display("ERROR: mag_y=%h",mag_y); errors=errors+1; end
        if(mag_z!==16'shAA55) begin $display("ERROR: mag_z=%h",mag_z); errors=errors+1; end
        if(errors==0) $display("PASS: LIS3MDL SPI initialization and XYZ burst read");
        else $fatal(1,"FAIL: %0d errors",errors);
        #20 $finish;
    end
    always @(posedge sample_valid) begin
        if(status_reads<2) premature_sample=1'b1;
    end
    initial #200000 $fatal(1,"FAIL: timeout");
endmodule

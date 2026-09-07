`timescale 1ns/1ps

module tb_ov5640_dvp;
    reg clk=0,rst_n=0,pclk=0,vsync=1,href=0;
    reg [7:0] data=0;
    tri1 scl,sda;
    reg slave_sda_low=0;
    reg slave_scl_low=0;
    wire camera_xclk,camera_reset_n,camera_pwdn,camera_configured;
    wire camera_identity_error,camera_bus_error;
    wire [15:0] camera_chip_id;
    wire frame_valid,line_valid,pixel_valid;
    wire [15:0] pixel;
    integer errors=0,pixels=0,init_index;
    integer saw_identity_error=0;
    integer saw_bus_error=0;
    reg [7:0] received;
    assign sda=slave_sda_low?1'b0:1'bz;
    assign scl=slave_scl_low?1'b0:1'bz;
    always #10 clk=~clk;
    always #5 pclk=~pclk;
    always @(posedge pixel_valid) pixels=pixels+1;
    always @(posedge camera_identity_error) saw_identity_error=1;
    always @(posedge camera_bus_error) saw_bus_error=1;

    // 独立于 DUT ROM 的黄金寄存器表，避免“实现函数同时充当预期值”的自证循环。
    function [15:0] expected_reg_addr(input [5:0] index);begin
        case(index)
          0:expected_reg_addr=16'h3103;  1:expected_reg_addr=16'h3008;
          2:expected_reg_addr=16'h3008;  3:expected_reg_addr=16'h3103;
          4:expected_reg_addr=16'h3017;  5:expected_reg_addr=16'h3018;
          6:expected_reg_addr=16'h3034;  7:expected_reg_addr=16'h3035;
          8:expected_reg_addr=16'h3036;  9:expected_reg_addr=16'h3037;
          10:expected_reg_addr=16'h3108; 11:expected_reg_addr=16'h3630;
          12:expected_reg_addr=16'h3631; 13:expected_reg_addr=16'h3632;
          14:expected_reg_addr=16'h3633; 15:expected_reg_addr=16'h3621;
          16:expected_reg_addr=16'h3704; 17:expected_reg_addr=16'h3703;
          18:expected_reg_addr=16'h3715; 19:expected_reg_addr=16'h3717;
          20:expected_reg_addr=16'h370B; 21:expected_reg_addr=16'h3705;
          22:expected_reg_addr=16'h3905; 23:expected_reg_addr=16'h3906;
          24:expected_reg_addr=16'h3901; 25:expected_reg_addr=16'h3731;
          26:expected_reg_addr=16'h3600; 27:expected_reg_addr=16'h3601;
          28:expected_reg_addr=16'h302D; 29:expected_reg_addr=16'h3620;
          30:expected_reg_addr=16'h371B; 31:expected_reg_addr=16'h471C;
          32:expected_reg_addr=16'h3800; 33:expected_reg_addr=16'h3801;
          34:expected_reg_addr=16'h3802; 35:expected_reg_addr=16'h3803;
          36:expected_reg_addr=16'h3804; 37:expected_reg_addr=16'h3805;
          38:expected_reg_addr=16'h3806; 39:expected_reg_addr=16'h3807;
          40:expected_reg_addr=16'h3808; 41:expected_reg_addr=16'h3809;
          42:expected_reg_addr=16'h380A; 43:expected_reg_addr=16'h380B;
          44:expected_reg_addr=16'h380C; 45:expected_reg_addr=16'h380D;
          46:expected_reg_addr=16'h380E; 47:expected_reg_addr=16'h380F;
          48:expected_reg_addr=16'h3814; 49:expected_reg_addr=16'h3815;
          50:expected_reg_addr=16'h4300; 51:expected_reg_addr=16'h501F;
          52:expected_reg_addr=16'h5000; 53:expected_reg_addr=16'h5001;
          54:expected_reg_addr=16'h300E; 55:expected_reg_addr=16'h460B;
          56:expected_reg_addr=16'h4713; 57:expected_reg_addr=16'h4407;
          58:expected_reg_addr=16'h460C; 59:expected_reg_addr=16'h3824;
          60:expected_reg_addr=16'h3008; 61:expected_reg_addr=16'h503D;
          default:expected_reg_addr=16'h4740;
        endcase
    end endfunction
    function [7:0] expected_reg_data(input [5:0] index);begin
        case(index)
          0:expected_reg_data=8'h11;  1:expected_reg_data=8'h82;
          2:expected_reg_data=8'h42;  3:expected_reg_data=8'h03;
          4:expected_reg_data=8'hFF;  5:expected_reg_data=8'hFF;
          6:expected_reg_data=8'h1A;  7:expected_reg_data=8'h11;
          8:expected_reg_data=8'h46;  9:expected_reg_data=8'h13;
          10:expected_reg_data=8'h01; 11:expected_reg_data=8'h36;
          12:expected_reg_data=8'h0E; 13:expected_reg_data=8'hE2;
          14:expected_reg_data=8'h12; 15:expected_reg_data=8'hE0;
          16:expected_reg_data=8'hA0; 17:expected_reg_data=8'h5A;
          18:expected_reg_data=8'h78; 19:expected_reg_data=8'h01;
          20:expected_reg_data=8'h60; 21:expected_reg_data=8'h1A;
          22:expected_reg_data=8'h02; 23:expected_reg_data=8'h10;
          24:expected_reg_data=8'h0A; 25:expected_reg_data=8'h12;
          26:expected_reg_data=8'h08; 27:expected_reg_data=8'h33;
          28:expected_reg_data=8'h60; 29:expected_reg_data=8'h52;
          30:expected_reg_data=8'h20; 31:expected_reg_data=8'h50;
          32:expected_reg_data=8'h00; 33:expected_reg_data=8'h00;
          34:expected_reg_data=8'h00; 35:expected_reg_data=8'h04;
          36:expected_reg_data=8'h0A; 37:expected_reg_data=8'h3F;
          38:expected_reg_data=8'h07; 39:expected_reg_data=8'h9B;
          40:expected_reg_data=8'h01; 41:expected_reg_data=8'h40;
          42:expected_reg_data=8'h00; 43:expected_reg_data=8'hF0;
          44:expected_reg_data=8'h07; 45:expected_reg_data=8'h68;
          46:expected_reg_data=8'h03; 47:expected_reg_data=8'hD8;
          48:expected_reg_data=8'h31; 49:expected_reg_data=8'h31;
          50:expected_reg_data=8'h6F; 51:expected_reg_data=8'h01;
          52:expected_reg_data=8'hA7; 53:expected_reg_data=8'hA3;
          54:expected_reg_data=8'h58; 55:expected_reg_data=8'h35;
          56:expected_reg_data=8'h03; 57:expected_reg_data=8'h04;
          58:expected_reg_data=8'h22; 59:expected_reg_data=8'h02;
          60:expected_reg_data=8'h02; 61:expected_reg_data=8'h00;
          default:expected_reg_data=8'h23;
        endcase
    end endfunction

    sensor_example_top #(
        .SCCB_CLK_DIV(2),.POWERDOWN_HOLD_CYCLES(3),.RESET_HOLD_CYCLES(3),
        .STARTUP_DELAY_CYCLES(4),.SOFT_RESET_DELAY_CYCLES(4)
    ) dut (
        .clk_i(clk),.rst_n_i(rst_n),.camera_xclk_o(camera_xclk),
        .camera_reset_n_o(camera_reset_n),.camera_pwdn_o(camera_pwdn),
        .camera_scl_io(scl),.camera_sda_io(sda),
        .camera_configured_o(camera_configured),
        .camera_identity_error_o(camera_identity_error),
        .camera_bus_error_o(camera_bus_error),.camera_chip_id_o(camera_chip_id),
        .camera_pclk_i(pclk),.camera_vsync_i(vsync),.camera_href_i(href),
        .camera_data_i(data),.frame_valid_o(frame_valid),.line_valid_o(line_valid),
        .pixel_valid_o(pixel_valid),.pixel_rgb565_o(pixel)
    );

    task check(input condition,input [511:0] message); begin
        if(!condition) begin $display("ERROR: %0s",message);errors=errors+1;end
    end endtask
    task wait_start; begin
        @(negedge sda); if(scl!==1'b1)begin $display("ERROR: SCCB START");errors=errors+1;end
    end endtask
    task recv_byte(output reg [7:0] value);integer b;begin
        value=0;for(b=7;b>=0;b=b-1)begin @(posedge scl);value[b]=sda;end
    end endtask
    task slave_dont_care;begin
        // SCCB 第九位允许为高；主机必须忽略它，而不能套用 I2C ACK 规则。
        @(negedge scl);slave_sda_low=0;@(posedge scl);@(negedge scl);slave_sda_low=0;
    end endtask
    task expect_byte(input [7:0] expected);begin
        recv_byte(received);if(received!==expected)begin
            $display("ERROR: SCCB expected %02h got %02h",expected,received);errors=errors+1;
        end slave_dont_care;
    end endtask
    task wait_stop;begin
        @(posedge scl);if(sda!==0)begin $display("ERROR: SCCB STOP setup");errors=errors+1;end
        @(posedge sda);
    end endtask
    task send_byte(input [7:0] value,input expected_ack);integer b;reg ack;begin
        for(b=7;b>=0;b=b-1)begin slave_sda_low=~value[b];@(posedge scl);@(negedge scl);end
        slave_sda_low=0;@(posedge scl);ack=(sda===0);@(negedge scl);
        if(ack!==expected_ack)begin $display("ERROR: master ACK");errors=errors+1;end
    end endtask
    task serve_read(input [15:0] reg_addr,input [7:0] value);begin
        wait_start;expect_byte(8'h78);expect_byte(reg_addr[15:8]);expect_byte(reg_addr[7:0]);
        wait_start;expect_byte(8'h79);send_byte(value,0);wait_stop;
    end endtask
    task serve_write(input [15:0] reg_addr,input [7:0] value);begin
        wait_start;expect_byte(8'h78);expect_byte(reg_addr[15:8]);expect_byte(reg_addr[7:0]);
        expect_byte(value);wait_stop;
    end endtask
    task send_pixel_byte(input [7:0] value);begin data=value;@(posedge pclk);#1;end endtask

    initial begin
        wait(rst_n);
        // 先制造一次真实 SCL 拉伸超时，随后释放并验证可恢复配置。
        slave_scl_low=1;
        wait(camera_bus_error);
        slave_scl_low=0;
        serve_read(16'h300A,8'h00);serve_read(16'h300B,8'h00);
        wait(camera_identity_error);
        href=1;vsync=0;send_pixel_byte(8'hAA);send_pixel_byte(8'h55);
        check(!pixel_valid && pixels==0,"pixels blocked before a verified configuration");
        href=0;vsync=1;
        serve_read(16'h300A,8'h56);serve_read(16'h300B,8'h40);
        for(init_index=0;init_index<=62;init_index=init_index+1)
            serve_write(expected_reg_addr(init_index[5:0]),
                        expected_reg_data(init_index[5:0]));
    end

    initial begin
        #35 rst_n=1;
        repeat(8)begin @(posedge clk);#1;check(camera_xclk===clk,"XCLK follows source clock");end
        wait(camera_configured);#1;
        check(camera_chip_id==16'h5640,"chip ID verification");
        check(saw_identity_error==1,"identity mismatch retry path");
        check(saw_bus_error==1,"SCL timeout retry path");
        check(!camera_identity_error && !camera_bus_error,"final SCCB status");
        check(camera_reset_n && !camera_pwdn,"camera power/reset state");
        check(dut.init_u.init_reg_addr(40)==16'h3808 && dut.init_u.init_reg_data(40)==8'h01,
              "QVGA width high byte");
        check(dut.init_u.init_reg_addr(43)==16'h380B && dut.init_u.init_reg_data(43)==8'hF0,
              "QVGA height low byte");
        check(dut.init_u.init_reg_addr(50)==16'h4300 && dut.init_u.init_reg_data(50)==8'h6F,
              "RGB565 format");
        check(dut.init_u.init_reg_addr(54)==16'h300E && dut.init_u.init_reg_data(54)==8'h58,
              "DVP enable");
        check(dut.init_u.init_reg_addr(62)==16'h4740 && dut.init_u.init_reg_data(62)==8'h23,
              "explicit PCLK/HREF/VSYNC polarity");
        // configured 必须先经过 PCLK 双触发同步，采集复位才会释放。
        repeat(3) @(posedge pclk);
        vsync=1;href=0;repeat(2)@(posedge pclk);
        vsync=0;href=1;
        // 0x4300=0x6F 的真实总线顺序为 RGB565 低字节在前。
        send_pixel_byte(8'h00);send_pixel_byte(8'hF8);
        check(pixel_valid && pixel==16'hF800,"first RGB565 pixel");
        send_pixel_byte(8'hE0);send_pixel_byte(8'h07);
        check(pixel_valid && pixel==16'h07E0,"second RGB565 pixel");
        href=0;@(posedge pclk);#1;check(!line_valid && !pixel_valid,"line end");
        href=1;send_pixel_byte(8'hAA);href=0;@(posedge pclk);#1;
        href=1;send_pixel_byte(8'h34);check(!pixel_valid,"discard partial pixel");
        send_pixel_byte(8'h12);check(pixel_valid && pixel==16'h1234,"clean next line pixel");
        vsync=1;href=0;@(posedge pclk);#1;
        check(!frame_valid && pixels==3,"frame close and complete-pixel count");
        if(errors==0)$display("PASS: OV5640 XCLK/reset, SCCB ID+QVGA RGB565 init, PCLK reset CDC, and little-endian DVP capture");
        else $fatal(1,"FAIL: %0d errors",errors);
        #20 $finish;
    end
    initial #4000000 $fatal(1,"FAIL: timeout");
endmodule

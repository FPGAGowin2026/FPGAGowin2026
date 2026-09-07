`timescale 1ns/1ps

module tb_tsl1401;
    reg clk=0,reset_n=0;
    wire tsl_clk,tsl_si,adc_start;
    reg [11:0] adc_data=0;
    reg adc_valid=0;
    wire [6:0] pixel_index;
    wire [11:0] pixel_data;
    wire pixel_valid,frame_start,frame_done,frame_active,adc_timeout;
    integer errors=0;
    integer pixels=0;
    integer frames=0;
    integer si_rises=0;
    integer sensor_clk_rises=0;
    integer frame_clk_start=0;
    integer adc_requests=0;
    reg respond=1;
    reg [6:0] requested_index;
    always #5 clk=~clk;

    sensor_example_top #(
        .CLK_HALF_PERIOD_CYCLES(2),.PIXEL_SETTLE_CYCLES(2),
        .ADC_TIMEOUT_CYCLES(8),.INTEGRATION_CYCLES(4),
        .CHARGE_TRANSFER_CYCLES(1)
    ) dut (
        .clk(clk),.reset_n(reset_n),.tsl_clk(tsl_clk),.tsl_si(tsl_si),
        .adc_start(adc_start),.adc_data(adc_data),.adc_valid(adc_valid),
        .pixel_index(pixel_index),.pixel_data(pixel_data),.pixel_valid(pixel_valid),
        .frame_start(frame_start),.frame_done(frame_done),.frame_active(frame_active),
        .adc_timeout(adc_timeout)
    );

    always @(posedge tsl_si) si_rises=si_rises+1;
    always @(posedge tsl_clk) sensor_clk_rises=sensor_clk_rises+1;
    always @(posedge frame_done) frames=frames+1;
    always @(posedge clk) begin
        adc_valid <= 1'b0;
        if(adc_start && respond) begin
            adc_requests=adc_requests+1;
            requested_index <= pixel_index;
            adc_data <= {5'd0,pixel_index};
            adc_valid <= 1'b1;
        end
        if(pixel_valid) begin
            if(pixel_index!==pixels[6:0] || pixel_data!=={5'd0,pixels[6:0]}) begin
                $display("ERROR: pixel %0d index=%0d data=%0d",pixels,pixel_index,pixel_data);
                errors=errors+1;
            end
            pixels=pixels+1;
        end
    end

    initial begin
        #30 reset_n=1;
        // 数据手册要求上电后先用一次 SI+129 CLK 清出不确定数据；
        // 该周期不得请求 ADC 或向用户发布帧。
        @(posedge tsl_si);
        @(negedge frame_active); #1;
        if(sensor_clk_rises!=129 || adc_requests!=0 || pixels!=0 || frames!=0) begin
            $display("ERROR: startup prime clocks=%0d adc=%0d pixels=%0d frames=%0d",
                     sensor_clk_rises,adc_requests,pixels,frames);
            errors=errors+1;
        end
        @(posedge frame_start);
        frame_clk_start=sensor_clk_rises;
        @(posedge frame_done); #1;
        if(pixels!=128 || adc_requests!=128 || frames!=1 || si_rises!=2 ||
           sensor_clk_rises-frame_clk_start!=129) begin
            $display("ERROR: complete frame pixels=%0d adc=%0d frames=%0d si=%0d clocks=%0d",
                     pixels,adc_requests,frames,si_rises,sensor_clk_rises-frame_clk_start);
            errors=errors+1;
        end
        if(frame_active || adc_timeout) begin
            $display("ERROR: flags after successful frame"); errors=errors+1;
        end
        // 第二帧不返回 ADC 数据：必须超时并丢弃，不能伪造 frame_done。
        respond=0;
        @(posedge frame_start);
        frame_clk_start=sensor_clk_rises;
        wait(adc_timeout); #1;
        wait(!frame_active); #1;
        if(frames!=1 || pixels!=128 || frame_active) begin
            $display("ERROR: timeout frame was published"); errors=errors+1;
        end
        if(sensor_clk_rises-frame_clk_start!=129) begin
            $display("ERROR: timeout flush clocks=%0d expected=129",sensor_clk_rises-frame_clk_start);
            errors=errors+1;
        end
        if(errors==0) $display("PASS: TSL1401 128 samples, mandatory 129th CLK, ADC handshake, and timeout resynchronization");
        else $fatal(1,"FAIL: %0d errors",errors);
        #20 $finish;
    end
    initial #1000000 $fatal(1,"FAIL: timeout");
endmodule

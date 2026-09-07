`timescale 1ns/1ps

module tb_choice3_sensor_guards;
    reg clk=0,rst_n=0,sample_valid=0;
    reg signed [15:0] ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    wire good_sample,rejected;
    reg event_toggle=0,source_ready=1;
    wire fresh_health,event_pulse;
    integer errors=0;

    always #5 clk=~clk;
    choice3_imu_sample_guard u_guard(.sample_valid_i(sample_valid),
      .accel_x_i(ax),.accel_y_i(ay),.accel_z_i(az),
      .gyro_x_i(gx),.gyro_y_i(gy),.gyro_z_i(gz),
      .good_sample_o(good_sample),.sentinel_rejected_o(rejected));
    choice3_event_freshness #(.TIMEOUT_CYCLES(5)) u_fresh(
      .clk_i(clk),.rst_n_i(rst_n),.event_toggle_async_i(event_toggle),
      .source_ready_i(source_ready),.health_o(fresh_health),.event_pulse_o(event_pulse));

    task automatic check(input bit condition,input string message);
        if(!condition)begin $display("FAIL: %s",message);errors=errors+1;end
    endtask
    initial begin
        repeat(3)@(posedge clk);rst_n=1;repeat(2)@(posedge clk);
        sample_valid=1;#1;
        check(!good_sample&&rejected,"六轴全 0 哨兵未拒绝");
        ax=-16'sd1;ay=-16'sd1;az=-16'sd1;gx=-16'sd1;gy=-16'sd1;gz=-16'sd1;#1;
        check(!good_sample&&rejected,"六轴全 F 哨兵未拒绝");
        ax=16'sd25;ay=-16'sd11;az=16'sd2048;gx=16'sd2;gy=16'sd0;gz=-16'sd3;#1;
        check(good_sample&&!rejected,"正常 IMU 样本被误拒绝");

        @(negedge clk);event_toggle=~event_toggle;sample_valid=0;
        repeat(4)@(posedge clk);#1;
        check(fresh_health,"有效事件未建立 freshness 健康");
        repeat(6)@(posedge clk);#1;
        check(!fresh_health,"无新事件时 freshness 未按期失效");
        source_ready=0;event_toggle=~event_toggle;repeat(4)@(posedge clk);#1;
        check(!fresh_health,"source_ready 失效时仍报告健康");

        if(errors==0)$display("PASS: tb_choice3_sensor_guards");
        else begin $display("FAIL: tb_choice3_sensor_guards errors=%0d",errors);$fatal(1);end
        $finish;
    end
    initial begin #10000;$display("FAIL: sensor guard TB timeout");$fatal(1);end
endmodule

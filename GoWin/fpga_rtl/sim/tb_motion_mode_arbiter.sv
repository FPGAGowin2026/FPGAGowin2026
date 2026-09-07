`timescale 1ns/1ps

// 单一自检覆盖仲裁器对外安全契约，不模拟尚未确定的顶层同步和电机模型。
module tb_motion_mode_arbiter;

    localparam COMMAND_WIDTH = 32;
    localparam MODE_FPGA     = 1'b0;
    localparam MODE_JETSON   = 1'b1;

    reg clk_i = 1'b0;
    reg rst_n_i = 1'b1;
    reg requested_mode_i = MODE_FPGA;
    reg motion_stopped_i = 1'b1;
    reg estop_i = 1'b0;
    reg critical_fault_i = 1'b0;
    reg safety_stop_i = 1'b0;

    reg fpga_command_valid_i = 1'b1;
    reg signed [COMMAND_WIDTH-1:0] fpga_linear_i = 32'sd120;
    reg signed [COMMAND_WIDTH-1:0] fpga_angular_i = -32'sd30;

    reg jetson_command_valid_i = 1'b1;
    reg signed [COMMAND_WIDTH-1:0] jetson_linear_i = 32'sd450;
    reg signed [COMMAND_WIDTH-1:0] jetson_angular_i = 32'sd80;

    wire active_mode_o;
    wire switch_pending_o;
    wire motion_enabled_o;
    wire signed [COMMAND_WIDTH-1:0] linear_command_o;
    wire signed [COMMAND_WIDTH-1:0] angular_command_o;

    integer check_count = 0;

    motion_mode_arbiter #(
        .COMMAND_WIDTH(COMMAND_WIDTH)
    ) dut (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .requested_mode_i(requested_mode_i),
        .motion_stopped_i(motion_stopped_i),
        .estop_i(estop_i),
        .critical_fault_i(critical_fault_i),
        .safety_stop_i(safety_stop_i),
        .fpga_command_valid_i(fpga_command_valid_i),
        .fpga_linear_i(fpga_linear_i),
        .fpga_angular_i(fpga_angular_i),
        .jetson_command_valid_i(jetson_command_valid_i),
        .jetson_linear_i(jetson_linear_i),
        .jetson_angular_i(jetson_angular_i),
        .active_mode_o(active_mode_o),
        .switch_pending_o(switch_pending_o),
        .motion_enabled_o(motion_enabled_o),
        .linear_command_o(linear_command_o),
        .angular_command_o(angular_command_o)
    );

    always #5 clk_i = ~clk_i;

    // 每个场景同时核对活动模式、切换状态、运动使能和最终运动目标。
    task check_outputs;
        input expected_mode;
        input expected_pending;
        input expected_enabled;
        input signed [COMMAND_WIDTH-1:0] expected_linear;
        input signed [COMMAND_WIDTH-1:0] expected_angular;
        begin
            #1;
            check_count = check_count + 1;
            if (active_mode_o !== expected_mode ||
                switch_pending_o !== expected_pending ||
                motion_enabled_o !== expected_enabled ||
                linear_command_o !== expected_linear ||
                angular_command_o !== expected_angular) begin
                $display(
                    "FAIL check=%0d mode=%b pending=%b enabled=%b linear=%0d angular=%0d",
                    check_count,
                    active_mode_o,
                    switch_pending_o,
                    motion_enabled_o,
                    linear_command_o,
                    angular_command_o
                );
                $fatal(1);
            end
        end
    endtask

    initial begin
        // 复位期间固定为 FPGA 独立模式且禁止输出。
        #1 rst_n_i = 1'b0;
        check_outputs(MODE_FPGA, 1'b0, 1'b0, 32'sd0, 32'sd0);

        @(negedge clk_i);
        rst_n_i = 1'b1;
        @(posedge clk_i);
        check_outputs(MODE_FPGA, 1'b0, 1'b1, 32'sd120, -32'sd30);

        // 急停、严重故障和安全停车在任意模式下都必须置零。
        @(negedge clk_i);
        estop_i = 1'b1;
        check_outputs(MODE_FPGA, 1'b0, 1'b0, 32'sd0, 32'sd0);
        estop_i = 1'b0;
        critical_fault_i = 1'b1;
        check_outputs(MODE_FPGA, 1'b0, 1'b0, 32'sd0, 32'sd0);
        critical_fault_i = 1'b0;
        safety_stop_i = 1'b1;
        check_outputs(MODE_FPGA, 1'b0, 1'b0, 32'sd0, 32'sd0);
        safety_stop_i = 1'b0;
        check_outputs(MODE_FPGA, 1'b0, 1'b1, 32'sd120, -32'sd30);

        // 运动中请求切换：立即置零，但 active_mode 不得改变。
        motion_stopped_i = 1'b0;
        requested_mode_i = MODE_JETSON;
        check_outputs(MODE_FPGA, 1'b1, 1'b0, 32'sd0, 32'sd0);
        repeat (2) @(posedge clk_i);
        check_outputs(MODE_FPGA, 1'b1, 1'b0, 32'sd0, 32'sd0);

        // 确认停止后切换，并保持一拍零输出再放行 Jetson。
        @(negedge clk_i);
        motion_stopped_i = 1'b1;
        @(posedge clk_i);
        check_outputs(MODE_JETSON, 1'b1, 1'b0, 32'sd0, 32'sd0);
        @(posedge clk_i);
        check_outputs(MODE_JETSON, 1'b0, 1'b1, 32'sd450, 32'sd80);

        // 未选中的 FPGA 命令不能影响协同模式输出。
        fpga_linear_i = -32'sd999;
        fpga_angular_i = -32'sd888;
        check_outputs(MODE_JETSON, 1'b0, 1'b1, 32'sd450, 32'sd80);

        // 当前来源无有效命令时保持停车。
        jetson_command_valid_i = 1'b0;
        check_outputs(MODE_JETSON, 1'b0, 1'b0, 32'sd0, 32'sd0);
        jetson_command_valid_i = 1'b1;

        // 停止状态下切回 FPGA，仍必须经过一拍零输出。
        requested_mode_i = MODE_FPGA;
        @(posedge clk_i);
        check_outputs(MODE_FPGA, 1'b1, 1'b0, 32'sd0, 32'sd0);
        @(posedge clk_i);
        check_outputs(MODE_FPGA, 1'b0, 1'b1, -32'sd999, -32'sd888);

        // 异步复位随时回到安全默认值。
        rst_n_i = 1'b0;
        check_outputs(MODE_FPGA, 1'b0, 1'b0, 32'sd0, 32'sd0);

        $display("PASS: %0d checks", check_count);
        $finish;
    end

endmodule

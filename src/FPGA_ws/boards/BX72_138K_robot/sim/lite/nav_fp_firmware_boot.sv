`timescale 1ns/1ps

// GoRV32 FP核执行链接ROM中的固件；外部MMIO只模拟确定的静止传感器和输出端口。
module nav_fp_firmware_boot;
    localparam integer CPU_CLOCK_HZ = 40000000;
    localparam [63:0] MAX_COMMIT_CYCLES =
        (64'd40000000 * 64'd30000) / 64'd1000000;
    reg clk = 1'b0;
    reg rst = 1'b1;
    reg gsri = 1'b0;
    always #12.5 clk = ~clk;
    GSR GSR(.GSRI(gsri));

    wire cpu_ready;
    wire cpu_fault;
    wire [7:0] fault_code;
    wire [31:0] heartbeat;
    wire io_valid;
    wire io_write;
    wire [11:0] io_addr;
    wire [31:0] io_wdata;
    wire [3:0] io_wstrb;
    reg [31:0] io_rdata;

    bx72_nav_platform dut (
        .clk(clk), .rst(rst),
        .cpu_ready_o(cpu_ready), .cpu_fault_o(cpu_fault),
        .fault_code_o(fault_code), .heartbeat_o(heartbeat),
        .io_valid_o(io_valid), .io_write_o(io_write), .io_addr_o(io_addr),
        .io_wdata_o(io_wdata), .io_wstrb_o(io_wstrb),
        .io_ready_i(1'b1), .io_rdata_i(io_rdata)
    );

    integer cycles = 0;
    integer wait_cycles;
    reg [31:0] ready_time = 0;
    integer control_commits = 0;
    integer valid_pose_commits = 0;
    integer invalid_pose_commits = 0;
    integer last_commit_cycle = 0;
    integer max_commit_cycles = 0;
    integer diag_writes = 0;
    reg [20:0] us_phase_ps = 0;
    reg [13:0] sample_us_count = 0;
    reg [31:0] now_us = 1;
    reg [31:0] snapshot_us = 1;
    reg [31:0] snapshot_status = 32'h00000017;
    reg [31:0] imu_seq = 1;
    reg [31:0] wheel_seq = 1;
    reg [31:0] imu_stamp = 1;
    reg [31:0] wheel_stamp = 1;
    reg [31:0] snapshot_imu_seq = 1;
    reg [31:0] snapshot_wheel_seq = 1;
    reg [31:0] snapshot_imu_stamp = 1;
    reg [31:0] snapshot_wheel_stamp = 1;
    reg sensor_freeze = 1'b0;
    reg signed [31:0] out_vx = 0;
    reg signed [31:0] out_vy = 0;
    reg signed [31:0] out_wz = 0;
    reg signed [31:0] pose_roll = 0;
    reg signed [31:0] pose_pitch = 0;
    reg signed [31:0] pose_yaw = 0;
    reg [31:0] pose_flags = 0;
    reg [31:0] max_gap_reported = 0;
    reg [31:0] firmware_selftest = 0;
    reg [31:0] p_diag [0:6];
    integer p_index;
    integer p_i;

    task fail;
        input [255:0] reason;
        begin
            $display("FAIL nav_fp_firmware_boot cycle=%0d now_us=%0d reason=%0s",
                     cycles, now_us, reason);
            $fatal(1, "nav_fp_firmware_boot failure at cycle=%0d", cycles);
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            cycles <= 0;
            us_phase_ps <= 0;
            sample_us_count <= 0;
            now_us <= 1;
            snapshot_us <= 1;
            snapshot_status <= 32'h00000017;
            imu_seq <= 1;
            wheel_seq <= 1;
            imu_stamp <= 1;
            wheel_stamp <= 1;
            snapshot_imu_seq <= 1;
            snapshot_wheel_seq <= 1;
            snapshot_imu_stamp <= 1;
            snapshot_wheel_stamp <= 1;
            control_commits <= 0;
            valid_pose_commits <= 0;
            invalid_pose_commits <= 0;
            last_commit_cycle <= 0;
            max_commit_cycles <= 0;
            diag_writes <= 0;
            out_vx <= 0;
            out_vy <= 0;
            out_wz <= 0;
            pose_roll <= 0;
            pose_pitch <= 0;
            pose_yaw <= 0;
            pose_flags <= 0;
            max_gap_reported <= 0;
            firmware_selftest <= 0;
            for (p_i = 0; p_i < 7; p_i = p_i + 1) p_diag[p_i] <= 0;
        end else begin
            cycles <= cycles + 1;
            if (us_phase_ps + 21'd25000 >= 21'd1000000) begin
                us_phase_ps <= us_phase_ps + 21'd25000 - 21'd1000000;
                now_us <= now_us + 1;
                if (sample_us_count == 9999) begin
                    sample_us_count <= 0;
                    if (!sensor_freeze) begin
                        imu_seq <= imu_seq + 1;
                        wheel_seq <= wheel_seq + 1;
                        imu_stamp <= now_us + 1;
                        wheel_stamp <= now_us + 1;
                    end
                end else begin
                    sample_us_count <= sample_us_count + 1'b1;
                end
            end else begin
                us_phase_ps <= us_phase_ps + 21'd25000;
            end

            if (cpu_fault) begin
                $display("FAIL nav_fp_firmware_boot cycle=%0d fault_code=%0d heartbeat=%0d",
                         cycles, fault_code, heartbeat);
                $fatal(1, "nav_fp real CPU sticky fault at cycle=%0d", cycles);
            end

            if (io_valid && io_write && io_wstrb == 4'hf) begin
                case (io_addr)
                    12'h108: begin
                        snapshot_us <= now_us;
                        snapshot_status <= 32'h00000017;
                        snapshot_imu_seq <= imu_seq;
                        snapshot_wheel_seq <= wheel_seq;
                        snapshot_imu_stamp <= imu_stamp;
                        snapshot_wheel_stamp <= wheel_stamp;
                    end
                    12'h200: out_vx <= io_wdata;
                    12'h204: out_vy <= io_wdata;
                    12'h208: out_wz <= io_wdata;
                    12'h21c: pose_roll <= io_wdata;
                    12'h220: pose_pitch <= io_wdata;
                    12'h218: pose_yaw <= io_wdata;
                    12'h224: pose_flags <= io_wdata;
                    12'h26c: max_gap_reported <= io_wdata;
                    12'h27c: firmware_selftest <= io_wdata;
                    12'h20c: if (io_wdata == 32'h42583732) begin
                        if (out_vx != 0 || out_vy != 0 || out_wz != 0)
                            fail("manual-link-down output was not zero");
                        control_commits <= control_commits + 1;
                        if (last_commit_cycle != 0 &&
                            cycles - last_commit_cycle > max_commit_cycles)
                            max_commit_cycles <= cycles - last_commit_cycle;
                        last_commit_cycle <= cycles;
                    end
                    12'h22c: if (io_wdata == 32'h42583732) begin
                        if (pose_flags[0]) valid_pose_commits <= valid_pose_commits + 1;
                        else if (sensor_freeze) invalid_pose_commits <= invalid_pose_commits + 1;
                    end
                    default: begin
                        if (io_addr >= 12'h240 && io_addr <= 12'h258 && io_addr[1:0] == 2'b00) begin
                            p_index = (io_addr - 12'h240) >> 2;
                            p_diag[p_index] <= io_wdata;
                            diag_writes <= diag_writes + 1;
                        end
                    end
                endcase
            end

            if (cycles > CPU_CLOCK_HZ * 2) fail("test exceeded two simulated seconds");
        end
    end

    always @* begin
        io_rdata = 32'd0;
        case (io_addr)
            12'h100: io_rdata = snapshot_us;
            12'h104: io_rdata = snapshot_status;
            12'h108: io_rdata = now_us;
            12'h110: io_rdata = snapshot_imu_seq;
            12'h114: io_rdata = 32'd0;
            12'h118: io_rdata = 32'd0;
            12'h11c: io_rdata = 32'h00002000; // 静止1g，±4g量程。
            12'h120: io_rdata = snapshot_wheel_seq;
            12'h124, 12'h128, 12'h12c: io_rdata = 32'd0;
            12'h130: io_rdata = snapshot_wheel_stamp;
            12'h134: io_rdata = snapshot_imu_stamp;
            12'h140, 12'h144, 12'h148, 12'h14c, 12'h150, 12'h154: io_rdata = 32'd0;
            12'h160: io_rdata = 32'd0;
            12'h190: io_rdata = 32'd1;
            12'h194, 12'h198, 12'h19c: io_rdata = 32'd0;
            12'h1a0, 12'h1a4, 12'h1a8, 12'h1ac: io_rdata = 32'd0;
            12'h280: io_rdata = 32'd0;
            default: io_rdata = 32'd0;
        endcase
    end

    initial begin
        repeat (8) @(negedge clk);
        gsri = 1'b1;
        repeat (2) @(negedge clk);
        rst = 1'b0;

        wait_cycles = 0;
        while (!cpu_ready && !cpu_fault &&
               wait_cycles < (CPU_CLOCK_HZ * 3 + 1) / 2) begin
            @(negedge clk);
            wait_cycles = wait_cycles + 1;
        end
        if (cpu_fault) fail("startup monitor reported a sticky CPU fault");
        if (!cpu_ready) fail("GoRV32 FP did not complete firmware startup");

        ready_time = now_us;
        wait (now_us >= ready_time + 300000);
        repeat (2) @(negedge clk);
        if (firmware_selftest != 32'h80000000)
            fail("firmware selftest did not report success");
        if (control_commits < 10 || valid_pose_commits == 0)
            fail("no sustained 50Hz valid pose and motor output");
        if (pose_roll > 30 || pose_roll < -30 || pose_pitch > 30 || pose_pitch < -30 ||
            pose_yaw > 30 || pose_yaw < -30)
            fail("static IMU pose was outside the zero-attitude tolerance");
        if (diag_writes < 7) fail("EKF covariance diagnostics were not published");
        for (p_i = 0; p_i < 7; p_i = p_i + 1) begin
            if (p_diag[p_i] == 0 || p_diag[p_i][31] || p_diag[p_i][30:23] == 8'hff)
                fail("EKF covariance diagonal was zero, negative, or non-finite");
        end
        if (max_gap_reported == 0 || max_gap_reported > 30000)
            fail("maximum control gap was outside the 50Hz budget");
        if (max_commit_cycles > MAX_COMMIT_CYCLES)
            fail("motor output commits exceeded the 30ms control interval");

        @(negedge clk);
        sensor_freeze = 1'b1;
        wait (now_us >= ready_time + 450000);
        repeat (2) @(negedge clk);
        if (invalid_pose_commits == 0)
            fail("stale IMU and wheel timestamps did not invalidate the pose");

        $display("PASS nav_fp_firmware_boot cycles=%0d ready=%0d heartbeat=%0d commits=%0d valid=%0d invalid=%0d max_gap_us=%0d max_commit_cycles=%0d selftest=%08x",
                 cycles, cpu_ready, heartbeat, control_commits, valid_pose_commits,
                 invalid_pose_commits, max_gap_reported, max_commit_cycles, firmware_selftest);
        $finish;
    end
endmodule

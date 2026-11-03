`timescale 1ns/1ps

// 真实核心和雷达跨域FIFO，CPU由可控MMIO事务替代。
module bx72_nav_platform #(parameter integer CLK_HZ=40000000)(
    input wire clk,rst,
    output wire cpu_ready_o,cpu_fault_o,output wire [7:0] fault_code_o,
    output wire [31:0] heartbeat_o,
    output wire io_valid_o,io_write_o,output wire [11:0] io_addr_o,
    output wire [31:0] io_wdata_o,output wire [3:0] io_wstrb_o,
    input wire io_ready_i,input wire [31:0] io_rdata_i
);
    reg valid=0,writing=0;
    reg [11:0] address=0;
    reg [31:0] data=0;
    reg [3:0] strobe=4'hf;
    assign cpu_ready_o=!rst;
    assign cpu_fault_o=0;
    assign fault_code_o=0;
    assign heartbeat_o=0;
    assign io_valid_o=valid;
    assign io_write_o=writing;
    assign io_addr_o=address;
    assign io_wdata_o=data;
    assign io_wstrb_o=strobe;
endmodule

module shared_lidar_queue_tb;
    reg clk=0,loc_clk=0,reset_n=0;
    always #12.5 clk=~clk;
    always #15 loc_clk=~loc_clk;
    reg sample=0,hit=0,packet_error=0;
    reg [15:0] angle=0,distance=0;
    reg [7:0] strength=0;
    reg [31:0] capture_stamp=0;
    reg task_valid=0;
    reg [255:0] task_data=0;
    wire lidar_valid;
    wire [1151:0] lidar_payload;
    wire [831:0] fusion_payload;
    wire [543:0] navigation_payload;
    wire [319:0] local_plan_payload;
    reg [1151:0] captured_lidar;
    integer lidar_packets=0;
    reg [72:0] expected[0:129];
    reg [31:0] read_value,old_head;
    reg [63:0] point_record;
    integer i;

    bx72_lite_robot_core #(.CONTROL_CLK_HZ(40_000_000)) dut(
        .control_clk(clk),.loc_clk33m(loc_clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'b0),.encoder_b_i(4'b0),.motor_in1_o(),.motor_in2_o(),
        .command_valid_i(1'b0),.command_link_down_i(1'b0),.command_heartbeat_i(1'b0),
        .command_velocity_i(48'd0),.auto_mode_valid_i(1'b0),.auto_enable_i(1'b0),
        .task_fragment_valid_i(task_valid),.task_fragment_i(task_data),.command_token_i(64'd0),
        .navigation_valid_o(),.navigation_payload_o(navigation_payload),
        .local_plan_valid_o(),.local_plan_payload_o(local_plan_payload),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(lidar_valid),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(lidar_payload),
        .fusion_payload_o(fusion_payload),.sensor_health_o(),
        .cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());

    always @(posedge loc_clk) begin
        if (!reset_n) begin
            lidar_packets<=0;
            captured_lidar<=0;
        end else if (lidar_valid) begin
            lidar_packets<=lidar_packets+1;
            captured_lidar<=lidar_payload;
        end
    end

    task automatic send_point(input [72:0] record_data);
        begin
            @(negedge loc_clk);
            {capture_stamp,hit,strength,angle,distance}=record_data;
            sample=1;
            @(negedge loc_clk);sample=0;
        end
    endtask

    task automatic load(input [11:0] addr,output [31:0] value);
        begin
            @(negedge clk);dut.cpu.address=addr;dut.cpu.valid=1;dut.cpu.writing=0;
            #1;value=dut.cpu.io_rdata_i;
            @(negedge clk);dut.cpu.valid=0;
        end
    endtask

    task automatic store(input [11:0] addr,input [3:0] strobe);
        begin
            @(negedge clk);
            dut.cpu.address=addr;dut.cpu.data=1;dut.cpu.strobe=strobe;dut.cpu.valid=1;dut.cpu.writing=1;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;dut.cpu.strobe=4'hf;
        end
    endtask

    task automatic expect_shared(input [72:0] record_data);
        begin
            load(12'h144,read_value);
            if(read_value!=={16'd0,record_data[39:32],6'd0,record_data[40],1'b1})
                $fatal(1,"共用队列META不匹配，实际值=%h",read_value);
            load(12'h140,read_value);
            if(read_value!==record_data[31:0])$fatal(1,"共用队列HEAD不匹配，实际值=%h",read_value);
            load(12'h14c,read_value);
            if(read_value!==record_data[72:41])$fatal(1,"共用队列时间戳不匹配，实际值=%h",read_value);
        end
    endtask

    task automatic check_empty;
        begin
            load(12'h144,read_value);
            if(read_value[0]!==1'b0)$fatal(1,"共用队列应为空");
        end
    endtask

    task automatic guard_command;
        begin
            @(negedge clk);force dut.output_commit=1'b1;
            @(negedge clk);release dut.output_commit;
            repeat(8)@(negedge clk);
        end
    endtask

    initial begin
        force dut.point_sample=sample;
        force dut.point_hit=hit;
        force dut.point_angle=angle;
        force dut.point_distance=distance;
        force dut.point_strength=strength;
        force dut.capture_us33=capture_stamp;
        force dut.packet_error=packet_error;
        repeat(5)@(negedge clk);reset_n=1;
        repeat(8)@(negedge clk);

        // 每5度仅保留最近有效回波，保持原始角度、强度与采集时间。
        for(i=0;i<16;i=i+1)begin
            expected[i]={32'(100+i),1'(i!=2),8'(8'h40+i),16'(100+i*100),16'(i==2?0:1000-i*10)};
            send_point(expected[i]);
        end
        wait(dut.raw_samples==16);repeat(5)@(negedge clk);
        expect_shared(expected[3]);
        store(12'h148,4'h7);expect_shared(expected[3]);
        store(12'h148,4'hf);expect_shared(expected[8]);
        store(12'h148,4'hf);expect_shared(expected[13]);
        store(12'h148,4'hf);check_empty();

        // 原始33MHz扫描遥测仍保留全部16点，不经过约简队列。
        if(lidar_packets!=1)$fatal(1,"原始雷达遥测包数量错误：%0d",lidar_packets);
        for(i=0;i<16;i=i+1)begin
            point_record={expected[i][31:16],expected[i][15:0],expected[i][39:32],4'd0,4'(i),16'd0};
            if(captured_lidar[1023-i*64 -: 64]!==point_record)
                $fatal(1,"原始雷达遥测第%0d点错误",i);
        end
        if(fusion_payload[367:365]!==3'b110 ||
           (navigation_payload[263:256]&8'hf0)!==8'ha0)
            $fatal(1,"融合或导航遥测缺少grid-v1标记");
        // 旧原始队列MMIO已经移除。
        load(12'h304,read_value);if(read_value!==0)$fatal(1,"旧地图队列仍可读");

        // 整个区间没有回波时，仍输出一个带真实时间的无回波记录。
        send_point({32'd200,1'b0,8'd3,16'd2100,16'd0});
        send_point({32'd201,1'b0,8'd4,16'd2600,16'd0});
        wait(dut.raw_samples==18);repeat(5)@(negedge clk);
        expect_shared(expected[15]);store(12'h148,4'hf);
        expect_shared({32'd200,1'b0,8'd3,16'd2100,16'd0});store(12'h148,4'hf);check_empty();

        // 128条RAM加一个预取槽；满队列拒收不影响原始guard。
        reset_n=0;repeat(6)@(negedge loc_clk);reset_n=1;repeat(8)@(negedge clk);
        for(i=0;i<130;i=i+1)begin
            expected[i]={32'(10000+i),1'b1,8'(i),16'(i==0?33000:i%2?600:100),16'(1000+i)};
            send_point(expected[i]);
        end
        wait(dut.raw_samples==130);repeat(5)@(negedge clk);
        if(!dut.lidar_queue_full||!dut.lidar_available)$fatal(1,"共用队列未按预期填满");
        load(12'h150,read_value);if(read_value!==0)$fatal(1,"未溢出时丢点非零");
        expect_shared(expected[0]);
        force dut.hard_kill=1'b0;
        force dut.committed_velocity={16'd100,32'd0};
        guard_command();
        if(dut.guard_stop||fusion_payload[359]!==1'b0||dut.guard_velocity[47:32]!==16'd100)
            $fatal(1,"远点保护或状态位未放行");
        send_point({32'd20001,1'b1,8'd7,16'd1000,16'd120});
        wait(dut.raw_samples==131);repeat(4)@(negedge clk);
        guard_command();
        if(!dut.guard_stop||fusion_payload[359]!==1'b1||dut.guard_velocity!==0)
            $fatal(1,"队列满时近障保护或独立状态位丢失");
        send_point({32'd20002,1'b0,8'd8,16'd1500,16'd0});
        wait(dut.raw_samples==132);repeat(4)@(negedge clk);
        load(12'h150,read_value);if(read_value!==2)$fatal(1,"溢出计数错误：%0d",read_value);
        expect_shared(expected[0]);
        load(12'h160,read_value);if(read_value!==0)$fatal(1,"空任务FIFO状态错误");

        // 满队列同拍pop与聚合输出拒收新点，随后释放的空位可接收新点。
        send_point({32'd20003,1'b1,8'd9,16'd2000,16'd1300});
        @(negedge clk);
        while(!dut.polar_valid)@(negedge clk);
        dut.cpu.address=12'h148;dut.cpu.data=1;dut.cpu.strobe=4'hf;dut.cpu.valid=1;dut.cpu.writing=1;
        @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        repeat(3)@(negedge clk);
        load(12'h150,read_value);if(read_value!==3)$fatal(1,"满队列pop同拍丢点未计数");
        send_point({32'd30000,1'b1,8'd42,16'd2500,16'd1444});
        wait(dut.raw_samples==134);repeat(5)@(negedge clk);
        for(i=1;i<129;i=i+1)begin expect_shared(expected[i]);store(12'h148,4'hf);end
        expect_shared({32'd20003,1'b1,8'd9,16'd2000,16'd1300});store(12'h148,4'hf);check_empty();

        // 上游错误与队列本地溢出分开计数。
        @(negedge loc_clk);packet_error=1;
        @(negedge loc_clk);packet_error=0;
        wait(dut.radar_error_count==1);repeat(4)@(negedge clk);
        load(12'h154,read_value);if(read_value!==1)$fatal(1,"缺少上游错误");
        @(negedge clk);force dut.radar_cdc_error=1'b1;
        @(negedge clk);release dut.radar_cdc_error;
        repeat(4)@(negedge clk);
        load(12'h154,read_value);if(read_value!==2)$fatal(1,"缺少跨域错误");
        load(12'h150,read_value);if(read_value!==3)$fatal(1,"上游错误计入了FIFO丢点");
        if(fusion_payload[223:192]!==32'd3)$fatal(1,"融合遥测共用队列丢点错误");
        check_empty();
        release dut.committed_velocity;release dut.hard_kill;
        $display("PASS 共用约简FIFO：最近回波、无回波、MMIO、溢出、独立保护、原始遥测");
        $finish;
    end
    initial begin #5000000;$fatal(1,"共用雷达FIFO测试超时");end
endmodule

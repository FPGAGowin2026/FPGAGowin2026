`timescale 1ns/1ps
// 采集/轮速/保护保留RTL；CPU与控制同为40MHz，导航长期状态保存在CPU RAM。
// IMU/雷达/健康输出在33MHz；电机/融合/导航输出在control_clk，各自在本域字节化。
module bx72_lite_robot_core #(
    parameter integer CONTROL_CLK_HZ=40000000
)(
    input wire control_clk,loc_clk33m,reset_n,
    input wire n10_uart_rx,imu_miso,
    output wire imu_cs_n,imu_sclk,imu_mosi,
    input wire [3:0] encoder_a_i,encoder_b_i,
    output wire [3:0] motor_in1_o,motor_in2_o,
    input wire command_valid_i,command_link_down_i,command_heartbeat_i,
    input wire [47:0] command_velocity_i,
    input wire auto_mode_valid_i,auto_enable_i,task_fragment_valid_i,
    input wire [255:0] task_fragment_i,
    input wire [63:0] command_token_i,
    output wire navigation_valid_o,output wire [543:0] navigation_payload_o,
    output wire local_plan_valid_o,output wire [319:0] local_plan_payload_o,
    output wire local_grid_valid_o,output wire [1663:0] local_grid_payload_o,
    input wire local_grid_ready_i,
    output wire auto_mode_o,fusion_valid_o,imu_valid_o,motor_valid_o,lidar_valid_o,
    output wire [255:0] imu_payload_o,output wire [703:0] motor_payload_o,
    output wire [1151:0] lidar_payload_o,output wire [831:0] fusion_payload_o,
    output wire [319:0] sensor_health_o,
    output wire cpu_ready_o,cpu_fault_o,cpu_busy_o,control_reset_o,
    output wire [7:0] cpu_fault_code_o
);
    (* ASYNC_REG="TRUE" *) reg [1:0] reset_ctrl,reset33;
    always @(posedge control_clk or negedge reset_n)
        if(!reset_n)reset_ctrl<=0;else reset_ctrl<={reset_ctrl[0],1'b1};
    always @(posedge loc_clk33m or negedge reset_n)
        if(!reset_n)reset33<=0;else reset33<={reset33[0],1'b1};
    wire rst_ctrl=!reset_ctrl[1],rst33=!reset33[1];
    assign control_reset_o=rst_ctrl;
    reg [5:0] us_phase;
    reg [31:0] now_us;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin us_phase<=0;now_us<=0;end
        else if(us_phase==CONTROL_CLK_HZ/1000000-1)begin us_phase<=0;now_us<=now_us+1'b1;end
        else us_phase<=us_phase+1'b1;
    end

    // 采集时刻统一使用40MHz产生的微秒时基；Gray跨域只引入有限同步延迟。
    reg [31:0] us_gray;
    (* ASYNC_REG="TRUE" *) reg [31:0] us_gray_meta33,us_gray_sync33;
    function automatic [31:0] gray_decode(input [31:0] value);
        integer k;
        begin
            gray_decode[31]=value[31];
            for(k=30;k>=0;k=k-1)gray_decode[k]=gray_decode[k+1]^value[k];
        end
    endfunction
    always @(posedge control_clk)
        if(rst_ctrl)us_gray<=0;else us_gray<=(now_us>>1)^now_us;
    always @(posedge loc_clk33m)
        if(rst33)begin us_gray_meta33<=0;us_gray_sync33<=0;end
        else begin us_gray_meta33<=us_gray;us_gray_sync33<=us_gray_meta33;end
    wire [31:0] capture_us33=gray_decode(us_gray_sync33);

    wire imu_checked,imu_ok,imu_configured,imu_pulse;
    wire [7:0] imu_identity;
    wire [31:0] imu_errors;
    wire [15:0] imu_sequence;
    wire signed [15:0] gx,gy,gz,ax,ay,az;
    reg [31:0] imu_count;
    always @(posedge loc_clk33m)
        if(rst33)imu_count<=0;else if(imu_pulse)imu_count<=imu_count+1'b1;
    imu660rc_spi_reader #(.CLK_HZ(33333333),.SPI_HZ(1000000),
        .POWERUP_CYCLES(500000),.RETRY_CYCLES(500000)) imu_reader(
        .clk(loc_clk33m),.rst(rst33),.spi_miso_i(imu_miso),
        .spi_cs_n_o(imu_cs_n),.spi_sck_o(imu_sclk),.spi_mosi_o(imu_mosi),
        .imu_present_o(imu_ok),.identity_checked_o(imu_checked),.identity_error_count_o(imu_errors),
        .imu_configured_o(imu_configured),.sample_valid_o(imu_pulse),
        .accel_x_o(ax),.accel_y_o(ay),.accel_z_o(az),
        .gyro_x_o(gx),.gyro_y_o(gy),.gyro_z_o(gz),
        .sample_sequence_o(imu_sequence),.who_am_i_o(imu_identity));
    wire imu_arrived;
    wire [159:0] imu_frame_ctrl;
    wire [127:0] imu_data_ctrl=imu_frame_ctrl[127:0];
    fusion_mailbox #(.WIDTH(160)) imu_cdc(.src_clk(loc_clk33m),.dst_clk(control_clk),.rst_n(reset_n),
        .send_i(imu_pulse),.data_i({capture_us33,imu_count+32'd1,gx,gy,gz,ax,ay,az}),
        .ready_o(),.valid_o(imu_arrived),.data_o(imu_frame_ctrl),.dropped_o());
    (* ASYNC_REG="TRUE" *) reg [1:0] imu_ok_sync;
    always @(posedge control_clk)
        if(rst_ctrl)imu_ok_sync<=0;else imu_ok_sync<={imu_ok_sync[0],imu_ok&&imu_configured};
    reg [31:0] imu_stamp;
    always @(posedge control_clk)
        if(rst_ctrl)imu_stamp<=0;else if(imu_arrived)imu_stamp<=imu_frame_ctrl[159:128];

    wire [7:0] uart_byte;
    wire uart_valid,uart_error,packet_valid,point_sample,point_hit;
    wire [31:0] packet_count,crc_count,length_count,angle_error_count;
    wire [15:0] point_angle,point_distance;
    wire [7:0] point_strength;
    reg [31:0] uart_count,uart_errors,point_count;
    uart_8n1_rx #(.CLK_HZ(33333333),.BAUD(230400)) radar_uart_rx(
        .clk(loc_clk33m),.reset_n(reset33[1]),.rx_i(n10_uart_rx),
        .data_o(uart_byte),.data_valid_o(uart_valid),.framing_error_o(uart_error));
    n10_uart_packet_parser #(.CHECK_CRC(1)) radar_parser(
        .clk(loc_clk33m),.reset_n(reset33[1]),.byte_data_i(uart_byte),.byte_valid_i(uart_valid),
        .packet_valid_o(packet_valid),.packet_count_o(packet_count),.crc_error_count_o(crc_count),
        .length_error_count_o(length_count),.angle_error_count_o(angle_error_count),.point_sample_valid_o(point_sample),
        .point_valid_o(point_hit),.point_angle_cdeg_o(point_angle),
        .point_distance_mm_o(point_distance),.point_strength_o(point_strength));
    reg [31:0] old_crc,old_length,old_angle_error;
    wire packet_error=uart_error || old_crc!=crc_count || old_length!=length_count || old_angle_error!=angle_error_count;
    always @(posedge loc_clk33m) begin
        if(rst33)begin uart_count<=0;uart_errors<=0;point_count<=0;old_crc<=0;old_length<=0;old_angle_error<=0;end
        else begin
            old_crc<=crc_count;old_length<=length_count;old_angle_error<=angle_error_count;
            if(uart_valid)uart_count<=uart_count+1'b1;
            if(uart_error)uart_errors<=uart_errors+1'b1;
            if(point_hit)point_count<=point_count+1'b1;
        end
    end
    // 一个经CRC验证的包共16样本；无回波样本保留角度覆盖信息。
    reg [3:0] point_index;
    reg [959:0] lidar_prefix;
    reg [1151:0] lidar_telemetry;
    reg lidar_telemetry_valid;
    wire [63:0] point_record={point_angle,point_distance,point_strength,4'd0,point_index,16'd0};
    always @(posedge loc_clk33m) begin
        if(rst33)begin point_index<=0;lidar_prefix<=0;lidar_telemetry<=0;lidar_telemetry_valid<=0;end
        else begin
            lidar_telemetry_valid<=0;
            if(packet_valid)point_index<=0;
            if(point_sample)begin
                lidar_prefix<={lidar_prefix[895:0],point_record};
                point_index<=point_index+1'b1;
                if(point_index==15)begin
                    lidar_telemetry<={packet_count,crc_count,length_count,32'd0,lidar_prefix,point_record};
                    lidar_telemetry_valid<=1;
                end
            end
        end
    end
    wire radar_full,radar_overflow,radar_arrived_raw;
    wire [73:0] radar_frame50_raw;
    reg radar_arrived;
    reg [73:0] radar_frame_ctrl;
    wire [41:0] radar_data_ctrl=radar_frame_ctrl[41:0];
    async_frame_fifo #(.DATA_WIDTH(74),.ADDR_WIDTH(5),.REGISTER_FULL(1)) radar_cdc(
        .wr_clk_i(loc_clk33m),.wr_rst_n_i(reset33[1]),.wr_valid_i(point_sample||packet_error),
        .wr_data_i({capture_us33,packet_error,point_hit&&!packet_error,point_strength,point_angle,point_distance}),
        .wr_full_o(radar_full),.wr_overflow_o(radar_overflow),
        .rd_clk_i(control_clk),.rd_rst_n_i(reset_ctrl[1]),.rd_ready_i(1'b1),
        .rd_data_o(radar_frame50_raw),.rd_valid_o(radar_arrived_raw),.rd_empty_o());
    // 在FIFO块RAM出口原子锁存数据/有效标记，切断RAM→错误译码→电机保护的大扇出链。
    // 仅增加一个40MHz周期，采集时间戳不改；错误到达本域寄存级后仍直接停车。
    always @(posedge control_clk) begin
        if(rst_ctrl)begin radar_arrived<=0;radar_frame_ctrl<=0;end
        else begin
            radar_arrived<=radar_arrived_raw;
            if(radar_arrived_raw)radar_frame_ctrl<=radar_frame50_raw;
        end
    end
    reg radar_overflow_toggle;
    always @(posedge loc_clk33m)
        if(rst33)radar_overflow_toggle<=0;else if(radar_overflow)radar_overflow_toggle<=~radar_overflow_toggle;
    (* ASYNC_REG="TRUE" *) reg [2:0] radar_overflow_sync;
    always @(posedge control_clk)
        if(rst_ctrl)radar_overflow_sync<=0;else radar_overflow_sync<={radar_overflow_sync[1:0],radar_overflow_toggle};
    wire radar_cdc_error=radar_overflow_sync[2]!=radar_overflow_sync[1];
    wire radar_sample_ctrl=radar_arrived&&!radar_data_ctrl[41];
    wire radar_error_ctrl=(radar_arrived&&radar_data_ctrl[41])||radar_cdc_error;
    reg [31:0] radar_stamp,radar_error_count;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin radar_stamp<=0;radar_error_count<=0;end
        else begin
            if(radar_sample_ctrl)radar_stamp<=radar_frame_ctrl[73:42];
            if(radar_error_ctrl)radar_error_count<=radar_error_count+1'b1;
        end
    end

    wire io_valid,io_write,io_ready;
    wire [11:0] io_addr;
    wire [31:0] io_wdata;
    wire [3:0] io_wstrb;
    reg [31:0] io_rdata;
    wire [31:0] cpu_heartbeat;
    assign io_ready=1'b1;
    assign cpu_busy_o=io_valid;
    wire io_store=io_valid&&io_ready&&io_write&&(&io_wstrb);
    wire io_commit_output=io_store&&io_addr==12'h20c&&io_wdata==32'h42583732;
    // CPU、MMIO、轮速与独立保护使用同一真实40MHz控制时钟。
    bx72_nav_platform #(.CLK_HZ(CONTROL_CLK_HZ)) cpu(
        .clk(control_clk),.rst(rst_ctrl),.cpu_ready_o(cpu_ready_o),
        .cpu_fault_o(cpu_fault_o),.fault_code_o(cpu_fault_code_o),.heartbeat_o(cpu_heartbeat),
        .io_valid_o(io_valid),.io_write_o(io_write),.io_addr_o(io_addr),.io_wdata_o(io_wdata),
        .io_wstrb_o(io_wstrb),.io_ready_i(io_ready),.io_rdata_i(io_rdata));
    wire lidar_available,lidar_queue_full;
    wire [72:0] lidar_head;
    reg [31:0] lidar_drops;
    wire polar_valid;
    wire [72:0] polar_record;
    reg [31:0] raw_samples, reduced_samples;
    bx72_polar_reduce polar_reduce(
        .clk(control_clk),.rst(rst_ctrl),.sample_i(radar_sample_ctrl),
        .hit_i(radar_data_ctrl[40]),.error_i(radar_error_ctrl),
        .angle_i(radar_data_ctrl[31:16]),.distance_i(radar_data_ctrl[15:0]),
        .strength_i(radar_data_ctrl[39:32]),.stamp_i(radar_frame_ctrl[73:42]),
        .valid_o(polar_valid),.record_o(polar_record));
    bx72_lite_queue #(.WIDTH(73),.ADDR_W(7)) lidar_queue(
        .clk(control_clk),.rst(rst_ctrl),.push_i(polar_valid),.data_i(polar_record),
        .full_o(lidar_queue_full),.pop_i(io_store&&io_addr==12'h148),
        .valid_o(lidar_available),.head_o(lidar_head));
    // 该队列由CPU一次消费，同时更新地图和CSM；无第二条原始建图支路。
    wire task_available,task_full;
    wire [255:0] task_head;
    reg task_overflow;
    wire task_overflow_ack;
    wire [31:0] command_sequence;
    reg [31:0] mode_sequence,link_stamp,manual_stamp;
    wire session_replaced,command_link_drop;
    reg [31:0] task_fifo_epoch;
    // 同一个控制域内按网络到达顺序处理接管和任务：切换撤销旧任务分片。
    // 软件读取前后核对该代次，并用相同代次POP，避免误弹出切换后的新分片。
    wire task_source_change=command_valid_i||auto_mode_valid_i||
                            session_replaced||command_link_drop;
    // 丢片当拍清空预取槽和FIFO；CPU完成零速提交前不接收后续残片。
    wire task_overflow_event=task_fragment_valid_i&&task_full&&!task_overflow&&!task_source_change;
    wire task_flush=task_overflow||task_overflow_event||task_source_change;
    bx72_lite_queue #(.WIDTH(256),.ADDR_W(4)) task_queue(
        .clk(control_clk),.rst(rst_ctrl||task_flush),.push_i(task_fragment_valid_i&&!task_flush),.data_i(task_fragment_i),
        .full_o(task_full),.pop_i(io_store&&io_addr==12'h184&&io_wdata==task_fifo_epoch&&!task_flush),
        .valid_o(task_available),.head_o(task_head));
    always @(posedge control_clk) begin
        if(rst_ctrl)begin lidar_drops<=0;task_overflow<=0;task_fifo_epoch<=0;raw_samples<=0;reduced_samples<=0;end
        else begin
            if(task_source_change||task_overflow_event)task_fifo_epoch<=task_fifo_epoch+1'b1;
            if(radar_sample_ctrl)raw_samples<=raw_samples+1'b1;
            if(polar_valid)reduced_samples<=reduced_samples+1'b1;
            if(polar_valid&&lidar_queue_full)lidar_drops<=lidar_drops+1'b1;
            if(task_overflow_event)task_overflow<=1;
            else if(task_overflow_ack)task_overflow<=0;
        end
    end

    reg auto_enabled,staged_auto_enabled,link_seen;
    // 原始序号可自然回绕为0，是否曾收到命令应独立记录。
    reg manual_command_seen;
    reg [31:0] manual_motion_sequence;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin manual_command_seen<=1'b0;manual_motion_sequence<=0;end
        else if(command_valid_i)begin
            manual_command_seen<=1'b1;
            // 与普通零速屏障区分；短暂非零后归零也不能漏掉人工接管。
            if(command_velocity_i!=48'd0)manual_motion_sequence<=manual_motion_sequence+1'b1;
        end
    end
    reg [31:0] boot_id;
    reg [63:0] session_token;
    reg session_known;
    wire [47:0] remote_velocity;
    reg [31:0] nav_status,output_command_seq,staged_output_command_seq,staged_output_mode_seq;
    wire link_fresh,manual_fresh;
    wire link_down=command_link_down_i||!link_fresh;
    wire manual_expired=!manual_fresh;
    wire command_output_current;
    wire manual_epoch_output_current;
    // 新会话先撤销上一会话的输出资格，直到新命令与CPU提交重新确认。
    // 分组异或归约表达完整64位差异，保持会话变化当拍闭锁。
    wire [7:0] session_token_diff /* synthesis syn_keep = 1 */;
    genvar session_byte;
    generate for (session_byte=0; session_byte<8; session_byte=session_byte+1) begin : session_compare
        assign session_token_diff[session_byte]=|(command_token_i[session_byte*8 +: 8]^session_token[session_byte*8 +: 8]);
    end endgenerate
    wire session_changed=|session_token_diff;
    assign session_replaced=command_heartbeat_i&&session_known&&session_changed;
    // 断链输入仍直达硬件停车；边沿只撤销此前快照/暂存输出的资格。
    reg command_link_down_q;
    always @(posedge control_clk) begin
        if(rst_ctrl)command_link_down_q<=1'b1;
        else command_link_down_q<=command_link_down_i;
    end
    assign command_link_drop=command_link_down_i&&!command_link_down_q;
    reg snapshot_after_fence,staged_after_fence,output_after_fence;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin
            // 启动固件先提交零速，再进行第一轮正常快照。
            snapshot_after_fence<=1'b1;staged_after_fence<=1'b1;output_after_fence<=1'b1;
        end else if(session_replaced||command_link_drop)begin
            snapshot_after_fence<=1'b0;staged_after_fence<=1'b0;output_after_fence<=1'b0;
        end else begin
            if(io_store&&io_addr==12'h108)begin
                snapshot_after_fence<=1'b1;staged_after_fence<=1'b0;
            end else if(io_store&&io_addr==12'h2a4)
                staged_after_fence<=snapshot_after_fence;
            if(io_commit_output)output_after_fence<=staged_after_fence;
        end
    end
    wire manual_source_ready=!auto_enabled&&manual_fresh&&!link_down&&!auto_mode_valid_i&&!session_replaced;
    wire manual_task_activity=task_available||task_fragment_valid_i;
    reg manual_output_confirmed,manual_command_after_invalidation,manual_source_ready_q;
    // 连续非零人工命令属于同一运动代次；原始序号仍逐包递增供CPU仲裁。
    // 首次非零、零速、模式、任务、会话及链路边界仍要求新代次确认。
    // 已确认输出仅在原有新鲜度及独立保护允许时保持至后续CPU提交。
    // 取消/到达等终态仅用于展示；CPU已确认的人工输出不能被终态反复清零。
    bx72_manual_command_epoch manual_command_epoch(
        .clk(control_clk),.rst(rst_ctrl),.command_valid_i(command_valid_i),
        .velocity_i(command_velocity_i),
        .reuse_allowed_i(manual_source_ready),
        .task_activity_i(manual_task_activity),
        .output_sequence_i(output_command_seq),.sequence_o(command_sequence),
        .velocity_o(remote_velocity),.output_current_o(manual_epoch_output_current));
    // 来源失效、模式或任务切换撤销旧人工输出；仅心跳恢复或CPU重复写回
    // 不能重新放行。须先收到新的人工命令，再确认它对应的真实CPU提交。
    // 自主输出仍由其当前命令/模式序号和既有新鲜度、保护链路约束。
    // 已确认的零输出可恢复只读就绪；它不会把旧非零人工意图重新放行。
    assign command_output_current=output_after_fence&&manual_epoch_output_current&&
        (auto_enabled||manual_output_confirmed||committed_velocity==48'd0);
    always @(posedge control_clk) begin
        if(rst_ctrl)begin
            manual_output_confirmed<=0;manual_command_after_invalidation<=0;manual_source_ready_q<=0;
        end else begin
            manual_source_ready_q<=manual_source_ready;
            if(command_link_down_i||session_replaced||(manual_source_ready_q&&!manual_source_ready)||
               manual_task_activity||auto_mode_valid_i)begin
                manual_output_confirmed<=0;manual_command_after_invalidation<=0;
            end else begin
                if(command_valid_i)begin
                    manual_command_after_invalidation<=1;
                    if((command_velocity_i==48'd0)!=(remote_velocity==48'd0))
                        manual_output_confirmed<=0;
                end
                if(io_commit_output&&staged_after_fence&&!staged_auto_enabled&&manual_source_ready&&
                   manual_command_after_invalidation&&!command_valid_i&&
                   staged_output_command_seq==command_sequence&&staged_output_mode_seq==mode_sequence)
                    manual_output_confirmed<=1;
            end
        end
    end
    always @(posedge control_clk) begin
        if(rst_ctrl)begin auto_enabled<=0;staged_auto_enabled<=0;link_seen<=0;mode_sequence<=0;link_stamp<=0;manual_stamp<=0;end
        else begin
            if(io_store&&io_addr==12'h2a0)staged_auto_enabled<=io_wdata[0];
            // 自主标记不能先于速度和确认序号生效，避免提前绕过人工租约。
            if(io_commit_output&&staged_after_fence&&
               staged_output_command_seq==command_sequence&&staged_output_mode_seq==mode_sequence)
                auto_enabled<=staged_auto_enabled;
            if(command_heartbeat_i)begin link_seen<=1;link_stamp<=now_us;end
            if(auto_mode_valid_i)begin auto_enabled<=auto_enable_i;mode_sequence<=mode_sequence+1'b1;end
            if(command_valid_i)begin
                auto_enabled<=0;manual_stamp<=now_us;
            end
            if(command_link_down_i||session_replaced)begin auto_enabled<=0;link_seen<=0;end
            // 让旧会话已暂存的自主输出也不能在会话切换后重新激活。
            if(session_replaced)mode_sequence<=mode_sequence+1'b1;
        end
    end
    // 与已验收网络会话绑定的易失epoch，任务必须读回本次值后提交。
    // 到达时刻参与混合，避免重启后把固定boot=1的延迟旧任务当成当前任务。
    always @(posedge control_clk)begin
        if(rst_ctrl||command_link_down_i)begin boot_id<=0;session_token<=0;session_known<=0;end
        else if(command_heartbeat_i&&(!session_known||session_changed))begin
            session_known<=1;session_token<=command_token_i;
            boot_id<=(now_us^command_token_i[63:32]^command_token_i[31:0]^32'h42583732)|32'd1;
        end
    end
    bx72_deadline_age #(.TIMEOUT_US(200000)) link_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(link_stamp),
        .valid_i(link_seen),.age_us_o(),.fresh_o(link_fresh));
    bx72_deadline_age #(.TIMEOUT_US(200000)) manual_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(manual_stamp),
        .valid_i(manual_command_seen),.age_us_o(),.fresh_o(manual_fresh));
    assign auto_mode_o=auto_enabled;

    wire wheel_valid;
    wire signed [31:0] wheel_vx,wheel_vy,wheel_wz;
    wire [127:0] wheel_ticks;
    wire [3:0] wheel_invalid;
    reg [31:0] wheel_sequence,wheel_stamp;
    reg signed [31:0] wheel_x_latest,wheel_y_latest,wheel_w_latest;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin wheel_sequence<=0;wheel_stamp<=0;wheel_x_latest<=0;wheel_y_latest<=0;wheel_w_latest<=0;end
        // 无效采样不刷新最后有效轮速，连续缺样由既有期限判定。
        else if(wheel_valid&&wheel_invalid==0)begin
            wheel_sequence<=wheel_sequence+1'b1;wheel_stamp<=now_us;
            wheel_x_latest<=wheel_vx;wheel_y_latest<=wheel_vy;wheel_w_latest<=wheel_wz;
        end
    end
    // 超时后锁存失效，仅新样本能恢复，避免计时器回绕后旧数据复活。
    reg wheel_sample_live,imu_sample_live;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin wheel_sample_live<=0;imu_sample_live<=0;end
        else begin
            if(wheel_valid&&wheel_invalid==0)wheel_sample_live<=1;
            else if(wheel_sample_live&&(now_us-wheel_stamp)>=500000)wheel_sample_live<=0;
            if(imu_arrived)imu_sample_live<=1;
            else if(imu_sample_live&&(now_us-imu_stamp)>=500000)imu_sample_live<=0;
        end
    end
    wire [31:0] wheel_age,imu_age;
    reg [31:0] radar_age;
    wire wheel_ok,imu_fresh;
    bx72_deadline_age #(.TIMEOUT_US(500000)) wheel_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(wheel_stamp),
        .valid_i(wheel_sample_live&&wheel_sequence!=0),.age_us_o(wheel_age),.fresh_o(wheel_ok));
    bx72_deadline_age #(.TIMEOUT_US(500000)) imu_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(imu_stamp),
        .valid_i(imu_sample_live&&imu_ok_sync[1]&&imu_data_ctrl[127:96]!=0),.age_us_o(imu_age),.fresh_o(imu_fresh));
    always @(posedge control_clk)
        if(rst_ctrl)radar_age<=32'hffffffff;else radar_age<=now_us-radar_stamp;
    wire guard_stop,guard_slow,radar_fresh,hard_kill;
    // kill解除后guard内部有两拍残留，等待流水稳定后再向CPU反馈真实保护。
    // 真实guard与电机驱动始终使用包含hard_kill的完整停车信号。
    reg [2:0] hard_kill_clear;
    always @(posedge control_clk)
        if(rst_ctrl||hard_kill)hard_kill_clear<=0;
        else hard_kill_clear<={hard_kill_clear[1:0],1'b1};
    wire cpu_guard_stop=guard_stop&&!hard_kill&&hard_kill_clear[2];
    wire [31:0] live_status={24'd0,auto_enabled,lidar_available,task_available,wheel_ok,
        cpu_guard_stop,radar_fresh,imu_fresh,link_down};
    reg [31:0] snap_now,snap_status,snap_wheel_seq,snap_wheel_stamp,snap_imu_stamp;
    reg [31:0] snap_command_seq,snap_mode_seq,snap_manual_motion_seq;
    reg [47:0] snap_remote_velocity;
    reg [127:0] snap_imu;
    reg signed [31:0] snap_vx,snap_vy,snap_wz;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin snap_now<=0;snap_status<=1;snap_imu<=0;snap_wheel_seq<=0;snap_wheel_stamp<=0;snap_imu_stamp<=0;snap_vx<=0;snap_vy<=0;snap_wz<=0;
            snap_command_seq<=0;snap_mode_seq<=0;snap_manual_motion_seq<=0;snap_remote_velocity<=0;end
        else if(io_store&&io_addr==12'h108)begin
            snap_now<=now_us;snap_status<=live_status;snap_imu<=imu_data_ctrl;
            snap_wheel_seq<=wheel_sequence;snap_vx<=wheel_x_latest;snap_vy<=wheel_y_latest;snap_wz<=wheel_w_latest;
            snap_wheel_stamp<=wheel_stamp;snap_imu_stamp<=imu_stamp;
            snap_command_seq<=command_sequence;snap_mode_seq<=mode_sequence;
            snap_manual_motion_seq<=manual_motion_sequence;snap_remote_velocity<=remote_velocity;
        end
    end

    reg signed [31:0] output_vx,output_vy,output_wz;
    reg [47:0] committed_velocity;
    reg signed [31:0] pose_x,pose_y,pose_yaw,pose_roll,pose_pitch;
    reg [31:0] pose_flags,pose_stamp,pose_updates,nav_task,nav_progress;
    reg [31:0] output_stamp;
    reg [31:0] output_mode_seq;
    reg output_seen,output_commit,pose_commit,nav_commit;
    assign task_overflow_ack=io_store&&io_addr==12'h188;
    reg [31:0] cov_diag[0:6];
    reg signed [31:0] gyro_bias,ekf_vx,ekf_vy,ekf_wz;
    reg [31:0] max_control_gap,csm_accepted,csm_rejected,csm_status,software_selftest;
    reg [31:0] nav_goal_x,nav_goal_y,nav_expiry,map_version,search_expansions,map_age,last_task_id;
    reg [31:0] plan_stamp,plan_x0,plan_y0,plan_x1,plan_y1,plan_valid;
    reg [1:0] nav_task_mode;
    reg [31:0] local_grid_words[0:51];
    integer grid_word;
    genvar grid_lane;
    generate for(grid_lane=0;grid_lane<52;grid_lane=grid_lane+1)begin:g_grid_payload
        assign local_grid_payload_o[1663-grid_lane*32-:32]=local_grid_words[grid_lane];
    end endgenerate
    assign local_grid_valid_o=io_store&&io_addr==12'h6d0&&io_wdata==32'h42583732&&
                              (local_grid_ready_i===1'b1)&&!rst_ctrl;
    always @(posedge control_clk)begin
        if(rst_ctrl)begin
            for(grid_word=0;grid_word<52;grid_word=grid_word+1)local_grid_words[grid_word]<=0;
        end else if(io_store&&io_addr[11:8]==4'h6&&io_addr[7:2]<52&&io_addr[1:0]==0&&
                    (local_grid_ready_i===1'b1))local_grid_words[io_addr[7:2]]<=io_wdata;
    end
    integer diag_index;
    wire output_bounds=output_vx>=-400&&output_vx<=400&&output_vy>=-400&&output_vy<=400&&output_wz>=-800&&output_wz<=800;
    always @(posedge control_clk) begin
        if(rst_ctrl)begin
            output_vx<=0;output_vy<=0;output_wz<=0;committed_velocity<=0;output_stamp<=0;output_seen<=0;output_commit<=0;
            output_command_seq<=0;output_mode_seq<=0;staged_output_command_seq<=0;staged_output_mode_seq<=0;
            pose_x<=0;pose_y<=0;pose_yaw<=0;pose_roll<=0;pose_pitch<=0;
            pose_flags<=0;pose_stamp<=0;pose_updates<=0;pose_commit<=0;
            nav_status<=0;nav_task<=0;nav_progress<=0;nav_commit<=0;
            for(diag_index=0;diag_index<7;diag_index=diag_index+1)cov_diag[diag_index]<=0;
            gyro_bias<=0;ekf_vx<=0;ekf_vy<=0;ekf_wz<=0;max_control_gap<=0;csm_accepted<=0;csm_rejected<=0;csm_status<=0;software_selftest<=0;
            nav_goal_x<=0;nav_goal_y<=0;nav_expiry<=0;map_version<=0;search_expansions<=0;map_age<=0;last_task_id<=0;
            nav_task_mode<=1;
            plan_stamp<=0;plan_x0<=0;plan_y0<=0;plan_x1<=0;plan_y1<=0;plan_valid<=0;
        end else begin
            output_commit<=0;pose_commit<=0;nav_commit<=0;
            if(io_store)case(io_addr)
                12'h200:output_vx<=io_wdata;
                12'h204:output_vy<=io_wdata;
                12'h208:output_wz<=io_wdata;
                12'h20c:if(io_wdata==32'h42583732)begin
                    committed_velocity<=output_bounds?{output_vx[15:0],output_vy[15:0],output_wz[15:0]}:48'd0;
                    // 序号和速度同一commit生效，防止提前放行上一帧速度。
                    output_command_seq<=staged_output_command_seq;
                    output_mode_seq<=staged_output_mode_seq;
                    output_commit<=1;output_seen<=1;output_stamp<=now_us;
                end
                12'h210:pose_x<=io_wdata;
                12'h214:pose_y<=io_wdata;
                12'h218:pose_yaw<=io_wdata;
                12'h21c:pose_roll<=io_wdata;
                12'h220:pose_pitch<=io_wdata;
                12'h224:pose_flags<=io_wdata;
                12'h228:pose_stamp<=io_wdata;
                12'h22c:if(io_wdata==32'h42583732)begin pose_commit<=1;pose_updates<=pose_updates+1'b1;end
                12'h2b0:plan_stamp<=io_wdata;
                12'h2b4:plan_x0<=io_wdata;
                12'h2b8:plan_y0<=io_wdata;
                12'h2bc:plan_x1<=io_wdata;
                12'h2c0:plan_y1<=io_wdata;
                12'h2c4:plan_valid<=io_wdata;
                12'h230:nav_status<=io_wdata;
                12'h234:nav_task<=io_wdata;
                12'h238:nav_progress<=io_wdata;
                12'h23c:if(io_wdata==32'h42583732)nav_commit<=1;
                12'h240:cov_diag[0]<=io_wdata;
                12'h244:cov_diag[1]<=io_wdata;
                12'h248:cov_diag[2]<=io_wdata;
                12'h24c:cov_diag[3]<=io_wdata;
                12'h250:cov_diag[4]<=io_wdata;
                12'h254:cov_diag[5]<=io_wdata;
                12'h258:cov_diag[6]<=io_wdata;
                12'h25c:gyro_bias<=io_wdata;
                12'h260:ekf_vx<=io_wdata;
                12'h264:ekf_vy<=io_wdata;
                12'h268:ekf_wz<=io_wdata;
                12'h26c:max_control_gap<=io_wdata;
                12'h270:csm_accepted<=io_wdata;
                12'h274:csm_rejected<=io_wdata;
                12'h278:csm_status<=io_wdata;
                12'h27c:software_selftest<=io_wdata;
                12'h284:nav_goal_x<=io_wdata;
                12'h288:nav_goal_y<=io_wdata;
                12'h28c:nav_expiry<=io_wdata;
                12'h290:map_version<=io_wdata;
                12'h294:search_expansions<=io_wdata;
                12'h298:map_age<=io_wdata;
                12'h29c:last_task_id<=io_wdata;
                12'h2a4:staged_output_command_seq<=io_wdata;
                12'h2a8:staged_output_mode_seq<=io_wdata;
                12'h2ac:nav_task_mode<=(io_wdata==2||io_wdata==3)?io_wdata[1:0]:2'd1;
                default:begin end
            endcase
        end
    end
    always @* begin
        io_rdata=0;
        case(io_addr)
            12'h100:io_rdata=snap_now;
            12'h104:io_rdata=snap_status;
            12'h108:io_rdata=now_us;
            12'h110:io_rdata=snap_imu[127:96];
            12'h114:io_rdata=snap_imu[95:64];
            12'h118:io_rdata=snap_imu[63:32];
            12'h11c:io_rdata=snap_imu[31:0];
            12'h120:io_rdata=snap_wheel_seq;
            12'h124:io_rdata=snap_vx;
            12'h128:io_rdata=snap_vy;
            12'h12c:io_rdata=snap_wz;
            12'h130:io_rdata=snap_wheel_stamp;
            12'h134:io_rdata=snap_imu_stamp;
            12'h140:io_rdata=lidar_head[31:0];
            12'h144:io_rdata={16'd0,lidar_head[39:32],6'd0,lidar_head[40],lidar_available};
            12'h14c:io_rdata=lidar_head[72:41];
            12'h150:io_rdata=lidar_drops;
            12'h154:io_rdata=radar_error_count;
            12'h158:io_rdata=raw_samples;
            12'h15c:io_rdata=reduced_samples;
            12'h160:io_rdata={30'd0,task_overflow,task_available};
            12'h164:io_rdata=task_head[255:224];
            12'h168:io_rdata=task_head[223:192];
            12'h16c:io_rdata=task_head[191:160];
            12'h170:io_rdata=task_head[159:128];
            12'h174:io_rdata=task_head[127:96];
            12'h178:io_rdata=task_head[95:64];
            12'h17c:io_rdata=task_head[63:32];
            12'h180:io_rdata=task_head[31:0];
            12'h18c:io_rdata=task_fifo_epoch;
            12'h190:io_rdata=snap_command_seq;
            12'h194:io_rdata=snap_remote_velocity[47:16];
            12'h198:io_rdata={16'd0,snap_remote_velocity[15:0]};
            12'h19c:io_rdata=snap_mode_seq;
            12'h1a0:io_rdata=boot_id;
            12'h1a4:io_rdata=32'd1;
            12'h1a8:io_rdata=session_token[31:0];
            12'h1ac:io_rdata=session_token[63:32];
            12'h1b0:io_rdata=snap_manual_motion_seq;
            12'h27c:io_rdata=software_selftest;
            12'h6d4:io_rdata={31'd0,(local_grid_ready_i===1'b1)};
            default:io_rdata=0;
        endcase
    end
    wire [31:0] pose_age;
    wire output_fresh,pose_fresh;
    bx72_deadline_age output_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(output_stamp),
        .valid_i(output_seen),.age_us_o(),.fresh_o(output_fresh));
    bx72_deadline_age pose_deadline(
        .clk(control_clk),.rst(rst_ctrl),.now_us_i(now_us),.last_us_i(pose_stamp),
        .valid_i(pose_flags[0]),.age_us_o(pose_age),.fresh_o(pose_fresh));
    // 软件健康及32位序号比较在本域先汇合寄存，再驱动保护/电机的大扇出端口。
    // 额外20ns仍在新鲜度提前1us撤销的余量内；复位、CPU故障及
    // 物理断链继续直接闭锁，不等待此寄存级。
    wire software_motion_stop_next=!cpu_ready_o||
            !command_output_current||output_mode_seq!=mode_sequence||
            (!auto_enabled&&(link_down||manual_expired))||task_overflow||!output_seen||
            !output_fresh||!pose_fresh||!imu_fresh||!wheel_ok||!output_bounds;
    reg software_motion_stop;
    always @(posedge control_clk) begin
        if(rst_ctrl)software_motion_stop<=1;
        else software_motion_stop<=software_motion_stop_next;
    end
    assign hard_kill=rst_ctrl||cpu_fault_o||command_link_down_i||session_replaced||software_motion_stop;
    wire guard_command_valid;
    wire [47:0] guard_velocity;
    // 只要已提交命令仍有所有权且未过期，持续让guard用最新雷达复核。
    // COMMIT当拍停用旧输入，下一拍以新速度和同批序号重新开始。
    wire guard_recheck=output_seen && output_fresh && !hard_kill &&
        !software_motion_stop_next && !io_commit_output;
    wire guard_input_valid=output_commit || guard_recheck;
    bx72_lite_guard #(.CLK_HZ(CONTROL_CLK_HZ)) guard(.clk(control_clk),.rst(rst_ctrl),.sample_valid_i(radar_sample_ctrl),.point_valid_i(radar_sample_ctrl&&radar_data_ctrl[40]),
        .angle_cdeg_i(radar_data_ctrl[31:16]),.distance_mm_i(radar_data_ctrl[15:0]),.packet_error_i(radar_error_ctrl),
        .command_valid_i(guard_input_valid),.command_velocity_i(committed_velocity),
        .kill_i(hard_kill),.command_valid_o(guard_command_valid),.command_velocity_o(guard_velocity),
        .stop_o(guard_stop),.slow_o(guard_slow),.lidar_fresh_o(radar_fresh));
    wire motor_command_valid=guard_command_valid;
    wire [47:0] motor_command_velocity=guard_velocity;
    wire motor_telemetry_ctrl;
    wire [703:0] motor_data_ctrl;
    mecanum_motor_closed_loop_core #(.CLK_HZ(CONTROL_CLK_HZ),.ENABLE_DEBUG_UART(0),.TRUSTED_GUARDED_INPUT(1)) motor_control_u(
        .clk_50m_i(control_clk),.user_reset_i(rst_ctrl),.debug_uart_rx_i(1'b1),.debug_uart_tx_o(),
        .encoder_a_i(encoder_a_i),.encoder_b_i(encoder_b_i),
        .remote_valid_i(motor_command_valid),.remote_link_down_i(hard_kill||guard_stop),.remote_velocity_i(motor_command_velocity),
        .motor_in1_o(motor_in1_o),.motor_in2_o(motor_in2_o),
        .odometry_valid_o(wheel_valid),.odometry_vx_o(wheel_vx),.odometry_vy_o(wheel_vy),.odometry_wz_o(wheel_wz),
        .odometry_ticks_o(wheel_ticks),.odometry_invalid_o(wheel_invalid),
        .telemetry_valid_o(motor_telemetry_ctrl),.telemetry_payload_o(motor_data_ctrl));
    // 三类控制遥测由控制域字节化后跨域，不再复制全宽payload到33MHz。
    assign motor_valid_o=motor_telemetry_ctrl;
    assign motor_payload_o=motor_data_ctrl;

    // bit31为毫米/毫弧度，bit30为本次完整架构格式，bit29保留为零。
    // bit28为启动算法自检成功；bit27:16保留自检失败mask。
    // bit15与bit14同时置位标记grid-v1；单独bit14或bit15留给旧架构。
    reg [23:0] diagnostic_divider;
    wire diagnostic_tick=diagnostic_divider==CONTROL_CLK_HZ/5-1;
    always @(posedge control_clk)begin
        if(rst_ctrl||diagnostic_tick)diagnostic_divider<=0;
        else diagnostic_divider<=diagnostic_divider+1'b1;
    end
    wire published_pose_valid=pose_flags[0]&&pose_fresh&&cpu_ready_o&&!cpu_fault_o&&imu_fresh&&wheel_ok;
    wire [31:0] lite_flags=32'hc000c000|{3'd0,(software_selftest==32'h80000000),software_selftest[11:0],16'd0}|
        (pose_flags&32'h00003ffe)|{31'd0,published_pose_valid}|
        {21'd0,guard_slow,guard_stop,radar_fresh,auto_enabled,3'd0,cpu_fault_o,1'b0};
    wire [831:0] fusion_data_ctrl={pose_x,pose_y,pose_yaw,ekf_vx,ekf_vy,ekf_wz,gyro_bias,
        pose_roll,pose_pitch,cov_diag[0],cov_diag[1],cov_diag[2],cov_diag[3],cov_diag[4],
        lite_flags,pose_updates,{24'd0,cpu_fault_code_o},max_control_gap,pose_stamp,
        lidar_drops,radar_error_count,csm_accepted,imu_age,radar_age,
        (nav_status|32'h00500000),csm_rejected};
    assign fusion_valid_o=pose_commit||((!cpu_ready_o||cpu_fault_o)&&diagnostic_tick);
    assign fusion_payload_o=fusion_data_ctrl;
    // bit7表示当前布局，bit5标记grid-v1，bit6保留给旧极坐标导航。
    wire [7:0] nav_flags=8'ha0|{5'd0,(software_selftest==32'h80000000),published_pose_valid,1'b0};
    wire [15:0] nav_expansions16=search_expansions>65535?16'hffff:search_expansions[15:0];
    // 0x41..0x4d报告真实闭锁原因；nav_flags[0]保留为零。
    wire [12:0] navigation_faults={
        nav_status[31],
        (command_link_down_i||(!auto_enabled&&(link_down||manual_expired))),
        nav_status[29],nav_status[30],task_overflow,
        (!command_output_current||output_mode_seq!=mode_sequence),
        !output_bounds,(!output_seen||!output_fresh),!radar_fresh,!wheel_ok,
        !imu_fresh,(!pose_fresh||!pose_flags[0]),
        (!cpu_ready_o||cpu_fault_o||software_selftest!=32'h80000000)};
    wire [7:0] navigation_error_next;
    reg [7:0] navigation_error;
    bx72_navigation_fault_encode navigation_fault_encoder(
        .faults_i(navigation_faults),
        .guard_stop_i(guard_stop),.error_o(navigation_error_next));
    always @(posedge control_clk)
        if(rst_ctrl)navigation_error<=8'h41;else navigation_error<=navigation_error_next;
    wire [543:0] nav_data_ctrl={boot_id,32'd1,nav_task,last_task_id,nav_expiry,now_us,session_token,
        nav_status[7:0],navigation_error,(auto_enabled?{6'd0,nav_task_mode}:8'd1),nav_flags,
        guard_velocity,nav_expansions16,nav_goal_x,nav_goal_y,nav_progress,map_age,pose_age,map_version};
    assign local_plan_valid_o=nav_commit;
    assign local_plan_payload_o={boot_id,32'd1,nav_task,plan_stamp,plan_x0,plan_y0,plan_x1,plan_y1,
                                 28'd0,1'b1,guard_stop,1'b0,plan_valid[0],map_version};
    assign navigation_valid_o=nav_commit;
    assign navigation_payload_o=nav_data_ctrl;
    assign imu_valid_o=imu_pulse;
    assign imu_payload_o={4'd0,imu_checked,imu_ok,imu_configured,1'b1,imu_identity,imu_sequence,
        gx,gy,gz,ax,ay,az,imu_count+32'd1,imu_errors,32'd0,32'd0};
    assign lidar_valid_o=lidar_telemetry_valid;
    assign lidar_payload_o=lidar_telemetry;
    assign sensor_health_o={uart_count,uart_errors,packet_count,crc_count,length_count,point_count,
        28'd0,imu_checked,imu_ok,imu_configured,(imu_count!=0),24'd0,imu_identity,imu_count,imu_errors};
endmodule

// 保留逐包命令序号给固件仲裁，同时记录上一次会改变人工运动含义的命令。
// 仅在同一有效来源、人工空闲模式且没有待处理任务时，连续非零速度
// 可复用该运动代次的已确认输出。零速及来源、任务或模式边界要求重新确认。
module bx72_manual_command_epoch(
    input wire clk,rst,command_valid_i,
    input wire [47:0] velocity_i,
    input wire reuse_allowed_i,task_activity_i,
    input wire [31:0] output_sequence_i,
    output reg [31:0] sequence_o,
    output reg [47:0] velocity_o,
    output wire output_current_o
);
    reg [31:0] effective_sequence;
    reg task_since_command;
    wire same_motion_epoch=(velocity_i==velocity_o)||
        ((velocity_i!=48'd0)&&(velocity_o!=48'd0));
    wire repeat_motion=same_motion_epoch&&
        reuse_allowed_i&&!task_activity_i&&!task_since_command;
    wire [31:0] current_distance=sequence_o-effective_sequence;
    wire [31:0] output_distance=output_sequence_i-effective_sequence;
    wire output_covers_motion=reuse_allowed_i&&
        !task_activity_i&&!task_since_command&&!current_distance[31]&&
        output_distance<=current_distance;
    assign output_current_o=(output_sequence_i==sequence_o)||output_covers_motion;

    always @(posedge clk) begin
        if(rst)begin
            sequence_o<=0;effective_sequence<=0;velocity_o<=0;task_since_command<=0;
        end else begin
            if(command_valid_i)begin
                sequence_o<=sequence_o+1'b1;
                velocity_o<=velocity_i;
                if(!repeat_motion)effective_sequence<=sequence_o+1'b1;
            end
            if(task_activity_i)task_since_command<=1'b1;
            else if(command_valid_i&&!repeat_motion)task_since_command<=1'b0;
        end
    end
endmodule

// 只编码遥测原因，不改变独立保护和PWM闭锁通路。
// 低编号优先：CPU、pose、IMU、wheel、lidar、输出年龄、范围、序号、
// 任务溢出、地图输入缺口、滚动屏障、命令链路、软件强制停车；0x4e为近障保护。
module bx72_navigation_fault_encode(
    input wire [12:0] faults_i,
    input wire guard_stop_i,
    output reg [7:0] error_o
);
    integer fault_index;
    always @* begin
        error_o=guard_stop_i?8'h4e:8'd0;
        for(fault_index=12;fault_index>=0;fault_index=fault_index-1)
            if(faults_i[fault_index])error_o=8'h41+fault_index[7:0];
    end
endmodule

// 同步块RAM加一个预取槽；软件peek与pop分离，禁止组合RAM大译码。
module bx72_lite_queue #(parameter integer WIDTH=32,ADDR_W=8)(
    input wire clk,rst,push_i,input wire [WIDTH-1:0] data_i,output wire full_o,
    input wire pop_i,output reg valid_o,output reg [WIDTH-1:0] head_o
);
    localparam integer DEPTH=1<<ADDR_W;
    reg [WIDTH-1:0] memory[0:DEPTH-1] /* synthesis syn_ramstyle="block_ram" */;
    reg [ADDR_W-1:0] wp,rp;
    reg [ADDR_W:0] count;
    wire push=push_i&&!full_o;
    wire fetch=(!valid_o||pop_i)&&(count!=0);
    assign full_o=count==DEPTH;
    always @(posedge clk) begin
        if(push)memory[wp]<=data_i;
        if(fetch)head_o<=memory[rp];
        if(rst)begin wp<=0;rp<=0;count<=0;valid_o<=0;end
        else begin
            if(push)wp<=wp+1'b1;
            if(fetch)begin rp<=rp+1'b1;valid_o<=1;end
            else if(pop_i)valid_o<=0;
            case({push,fetch})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:begin end
            endcase
        end
    end
endmodule

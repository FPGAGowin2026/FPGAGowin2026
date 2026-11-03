`timescale 1ns/1ps
// 原生采集/控制源经50MHz包总线、125MHz CDC到UDP/GMII的完整遥测链路。
module nav_full_telemetry_path_tb;
`ifdef BX72_VENDOR_PRIMITIVES
    GSR GSR(.GSRI(1'b1));
`endif
    reg reset_p=1'b1;
    reg raw_clk=0,control_clk=0,bus_clk=0,tx_clk=0;
    always #15 raw_clk=~raw_clk;       // 33.333 MHz sensor producer
    always #12.5 control_clk=~control_clk; // 40 MHz navigation producer
    always #10 bus_clk=~bus_clk;       // 50 MHz common packet bus
    always #4 tx_clk=~tx_clk;         // 125 MHz Ethernet TX

    (* ASYNC_REG="TRUE" *) reg [1:0] bus_reset_sync=2'b11;
    always @(posedge bus_clk or posedge reset_p)
        if(reset_p)bus_reset_sync<=2'b11;
        else bus_reset_sync<={bus_reset_sync[0],1'b0};
    wire bus_reset=bus_reset_sync[1];

    reg raw_imu_valid=0,raw_lidar_valid=0,raw_health_valid=0;
    reg control_motor_valid=0,control_fusion_valid=0,control_navigation_valid=0;
    reg [255:0] imu_payload=0;
    reg [1151:0] lidar_payload=0;
    reg [895:0] health_payload=0;
    reg [703:0] motor_payload=0;
    reg [831:0] fusion_payload=0;
    reg [543:0] navigation_payload=0;
    wire raw_valid,raw_nonzero,raw_start,raw_read,raw_tx_reset;
    wire [7:0] raw_length,raw_data;
    wire [159:0] raw_dropped;
    wire control_valid,control_nonzero,control_start,control_read,control_tx_reset;
    wire [7:0] control_length,control_data;
    wire [159:0] control_dropped;

    bx72_telemetry #(.SRC_CLK_HZ(33333333),.CHANNEL_MASK(6'b010101)) raw_telemetry (
        .src_clk(raw_clk),.tx_clk(bus_clk),.reset_n(!reset_p),
        .imu_valid_i(raw_imu_valid),.imu_payload_i(imu_payload),
        .motor_valid_i(1'b0),.motor_payload_i(704'd0),
        .lidar_valid_i(raw_lidar_valid),.lidar_payload_i(lidar_payload),
        .fusion_valid_i(1'b0),.fusion_payload_i(832'd0),
        .health_valid_i(raw_health_valid),.health_payload_i(health_payload),
        .navigation_valid_i(1'b0),.navigation_payload_i(544'd0),
        .tx_valid_o(raw_valid),.tx_length_o(raw_length),.tx_length_nonzero_o(raw_nonzero),
        .tx_data_o(raw_data),.tx_start_i(raw_start),.tx_read_i(raw_read),
        .dropped_o(raw_dropped),.src_reset_i(1'b0),.tx_reset_i(1'b0),.tx_reset_o(raw_tx_reset)
    );
    bx72_telemetry #(.SRC_CLK_HZ(40000000),.CHANNEL_MASK(6'b101010)) control_telemetry (
        .src_clk(control_clk),.tx_clk(bus_clk),.reset_n(!reset_p),
        .imu_valid_i(1'b0),.imu_payload_i(256'd0),
        .motor_valid_i(control_motor_valid),.motor_payload_i(motor_payload),
        .lidar_valid_i(1'b0),.lidar_payload_i(1152'd0),
        .fusion_valid_i(control_fusion_valid),.fusion_payload_i(fusion_payload),
        .health_valid_i(1'b0),.health_payload_i(896'd0),
        .navigation_valid_i(control_navigation_valid),.navigation_payload_i(navigation_payload),
        .tx_valid_o(control_valid),.tx_length_o(control_length),.tx_length_nonzero_o(control_nonzero),
        .tx_data_o(control_data),.tx_start_i(control_start),.tx_read_i(control_read),
        .dropped_o(control_dropped),.src_reset_i(1'b0),.tx_reset_i(1'b0),.tx_reset_o(control_tx_reset)
    );

    wire mux_valid,mux_nonzero;
    wire [7:0] mux_length,mux_data;
    wire cdc_packet_start,cdc_packet_read;
    wire cdc_valid,cdc_nonzero;
    wire [7:0] cdc_length,cdc_data;
    reg sender_allow=0,copy_allow=0;
    bx72_telemetry_mux packet_mux (
        .clk(bus_clk),.reset_i(bus_reset),
        .source0_valid_i(raw_valid),.source0_length_nonzero_i(raw_nonzero),
        .source0_length_i(raw_length),.source0_data_i(raw_data),
        .source0_start_o(raw_start),.source0_read_o(raw_read),
        .source1_valid_i(control_valid),.source1_length_nonzero_i(control_nonzero),
        .source1_length_i(control_length),.source1_data_i(control_data),
        .source1_start_o(control_start),.source1_read_o(control_read),
        .tx_valid_o(mux_valid),.tx_length_nonzero_o(mux_nonzero),
        .tx_length_o(mux_length),.tx_data_o(mux_data),
        .tx_start_i(cdc_packet_start),.tx_read_i(cdc_packet_read)
    );
    bx72_telemetry_packet_cdc packet_cdc (
        .src_clk(bus_clk),.src_reset_i(bus_reset),.tx_clk(tx_clk),.reset_n(!reset_p),
        .packet_valid_i(copy_allow && mux_valid),.packet_length_nonzero_i(copy_allow && mux_nonzero),
        .packet_length_i(mux_length),.packet_data_i(mux_data),
        .packet_start_o(cdc_packet_start),.packet_read_o(cdc_packet_read),
        .tx_valid_o(cdc_valid),.tx_length_nonzero_o(cdc_nonzero),
        .tx_length_o(cdc_length),.tx_data_o(cdc_data),
        .tx_start_i(sender_allow ? sender_telemetry_start : 1'b0),
        .tx_read_i(sender_allow ? sender_telemetry_read : 1'b0)
    );

    wire sender_telemetry_start,sender_telemetry_read;
    wire sender_reply_ready,camera_ready;
    wire rgmii_gtxc,rgmii_txen,phy_rst_n,phy_configured,phy_link_up;
    wire [3:0] rgmii_txd;
    wire [31:0] packet_count,packet_count_gray,video_packet_count,video_packet_count_gray;
    wire video_fifo_overflow,video_source_underflow,video_frame_marker_error;
    eth_udp_video_sender #(
        .IMAGE_WIDTH(8),.IMAGE_HEIGHT(8),.MAX_PIXELS_PER_PACKET(16),
        .FIFO_ADDR_WIDTH(4),.STATUS_INTERVAL_CYCLES(100000000),.TELEMETRY_ENABLE(1)
    ) sender (
        .clk50m(bus_clk),.clk125m(tx_clk),.reset_p(reset_p),.cam_pclk(raw_clk),
        .camera_vsync(1'b0),.camera_pixel_valid(1'b0),.camera_pixel(16'd0),.camera_timestamp(32'd0),
        .reply_valid(1'b0),.reply_data(256'd0),.reply_ready(sender_reply_ready),
        .rx_diagnostics_i(128'd0),.rx_diagnostics_raw_i(1'b0),
        .telemetry_valid(sender_allow && cdc_valid),
        .telemetry_length({8'd0,cdc_length}),
        .telemetry_length_nonzero(sender_allow && cdc_nonzero),.telemetry_data(cdc_data),
        .telemetry_start(sender_telemetry_start),.telemetry_read_request(sender_telemetry_read),
        .camera_pixel_ready(camera_ready),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen),.eth_rst_n(phy_rst_n),.phy_configured(phy_configured),
        .phy_link_up(phy_link_up),.packet_count(packet_count),.packet_count_gray(packet_count_gray),
        .video_packet_count(video_packet_count),.video_packet_count_gray(video_packet_count_gray),
        .video_fifo_overflow(video_fifo_overflow),.video_source_underflow(video_source_underflow),
        .video_frame_marker_error(video_frame_marker_error)
    );

    function automatic [31:0] sample_word(input integer channel_id,input integer word_index);
        begin sample_word=32'hA5000000 | (channel_id<<20) | word_index; end
    endfunction
    function automatic integer app_length(input integer kind);
        begin
            case(kind)
                3:app_length=48;
                4:app_length=104;
                5:app_length=160;
                6:app_length=120;
                7:app_length=128;
                8:app_length=84;
                default:app_length=0;
            endcase
        end
    endfunction
    reg [31:0] expected_timestamp [0:8];
    function automatic [7:0] expected_app_byte(
        input integer kind,input integer byte_index,input [31:0] timestamp_us
    );
        reg [7:0] header_byte;
        reg [31:0] word_value;
        integer word_index,byte_in_word;
        begin
            case(byte_index)
                0:header_byte=8'h42; 1:header_byte=8'h58;
                2:header_byte=8'h37; 3:header_byte=8'h32;
                4:header_byte=8'd1; 5:header_byte=kind[7:0];
                6:header_byte=app_length(kind)>>8; 7:header_byte=app_length(kind);
                8,9,10,11:header_byte=8'd0;
                12:header_byte=timestamp_us[31:24];
                13:header_byte=timestamp_us[23:16];
                14:header_byte=timestamp_us[15:8];
                15:header_byte=timestamp_us[7:0];
                default:header_byte=8'd0;
            endcase
            if(byte_index<16)expected_app_byte=header_byte;
            else begin
                word_index=(byte_index-16)/4;
                byte_in_word=(byte_index-16)%4;
                if((kind==3 && word_index==7) || (kind==4 && word_index==21) ||
                   (kind==5 && word_index==3))word_value=32'd0;
                else if(kind==6 && word_index==25)word_value=32'hD6C0FFEE;
                else if(kind==7 && word_index==27)word_value=32'h475200A9;
                else word_value=sample_word(kind-3,word_index);
                expected_app_byte=word_value[31-byte_in_word*8-:8];
            end
        end
    endfunction
    function automatic [31:0] crc32_byte(input [31:0] previous,input [7:0] value);
        reg [31:0] crc;
        begin
            crc=previous^value;
            for(integer k=0;k<8;k=k+1)crc=crc[0] ? (crc>>1)^32'hedb88320 : crc>>1;
            crc32_byte=crc;
        end
    endfunction

    reg [7:0] frame_bytes [0:2047];
    integer frame_length=0,frame_count=0;
    reg frame_active=0;
    reg [8:0] seen_kind=0;
    integer kind,udp_payload_length,expected_length,loop_index;
    reg [31:0] fcs,ip_sum;
    reg [7:0] expected_byte;
    always @(negedge tx_clk) if(!reset_p)begin
        if(sender.gmii_txen)begin
            if(!frame_active)begin
                if(frame_count>=6)$fatal(1,"unexpected extra GMII frame");
                frame_length=0;
                frame_active=1'b1;
            end
            frame_bytes[frame_length]=sender.gmii_txd;
            frame_length=frame_length+1;
        end else if(frame_active)begin
            if(frame_length<54)$fatal(1,"truncated telemetry Ethernet frame");
            for(loop_index=0;loop_index<7;loop_index=loop_index+1)
                if(frame_bytes[loop_index]!==8'h55)$fatal(1,"bad preamble byte %0d",loop_index);
            if(frame_bytes[7]!==8'hD5 || {frame_bytes[20],frame_bytes[21]}!==16'h0800 ||
               frame_bytes[22]!==8'h45)
                $fatal(1,"Ethernet/IPv4 header mismatch");
            if({frame_bytes[34],frame_bytes[35],frame_bytes[36],frame_bytes[37]}!==32'hC0A80002 ||
               {frame_bytes[38],frame_bytes[39],frame_bytes[40],frame_bytes[41]}!==32'hC0A80004)
                $fatal(1,"IPv4 endpoint mismatch");
            ip_sum=0;
            for(loop_index=0;loop_index<10;loop_index=loop_index+1)
                ip_sum=ip_sum+{frame_bytes[22+loop_index*2],frame_bytes[23+loop_index*2]};
            ip_sum=(ip_sum & 32'h0000FFFF)+(ip_sum>>16);
            ip_sum=(ip_sum & 32'h0000FFFF)+(ip_sum>>16);
            if(ip_sum[15:0]!==16'hFFFF)$fatal(1,"IPv4 checksum mismatch");
            if({frame_bytes[42],frame_bytes[43]}!==16'd5000 ||
               {frame_bytes[44],frame_bytes[45]}!==16'd6104 ||
               {frame_bytes[48],frame_bytes[49]}!==16'd0)
                $fatal(1,"UDP header mismatch");
            udp_payload_length={frame_bytes[46],frame_bytes[47]}-16'd8;
            if({frame_bytes[24],frame_bytes[25]}!==udp_payload_length+16'd28 ||
               frame_length!==udp_payload_length+54)
                $fatal(1,"IP/UDP/frame length mismatch");
            kind=frame_bytes[55];
            expected_length=app_length(kind);
            if(expected_length==0 || udp_payload_length!=expected_length)
                $fatal(1,"unexpected telemetry kind/length kind=%0d payload=%0d",kind,udp_payload_length);
            if(seen_kind[kind])$fatal(1,"duplicate telemetry kind %0d",kind);
            if({frame_bytes[50],frame_bytes[51],frame_bytes[52],frame_bytes[53]}!==32'h42583732 ||
               frame_bytes[54]!==8'd1 || {frame_bytes[56],frame_bytes[57]}!==expected_length[15:0] ||
               {frame_bytes[58],frame_bytes[59],frame_bytes[60],frame_bytes[61]}!==32'd0)
                $fatal(1,"BX72 application header mismatch kind=%0d",kind);
            for(loop_index=0;loop_index<expected_length;loop_index=loop_index+1)begin
                expected_byte=expected_app_byte(kind,loop_index,expected_timestamp[kind]);
                if(frame_bytes[50+loop_index]!==expected_byte)
                    $fatal(1,"telemetry payload mismatch kind=%0d byte=%0d got=%02x expected=%02x",
                        kind,loop_index,frame_bytes[50+loop_index],expected_byte);
            end
            fcs=32'hFFFFFFFF;
            for(loop_index=8;loop_index<frame_length-4;loop_index=loop_index+1)
                fcs=crc32_byte(fcs,frame_bytes[loop_index]);
            fcs=~fcs;
            if({frame_bytes[frame_length-1],frame_bytes[frame_length-2],
                frame_bytes[frame_length-3],frame_bytes[frame_length-4]}!==fcs)
                $fatal(1,"Ethernet FCS mismatch kind=%0d",kind);
            seen_kind[kind]=1'b1;
            frame_count=frame_count+1;
            frame_active=1'b0;
        end
    end

    integer word_index,guard;
    reg [7:0] held_mux_byte,held_cdc_byte;
    initial begin
        repeat(12)begin
            #1000;
            $display("TRACE telemetry t=%0t frames=%0d raw=%0d ctrl=%0d mux=%0d copy=%0d cdc=%0d remaining=%0d",$time,frame_count,
                raw_telemetry.packet_reader.tx_state,control_telemetry.packet_reader.tx_state,packet_mux.state,
                packet_cdc.src_state,packet_cdc.packet_reader.tx_state,packet_cdc.packet_reader.tx_remaining);
        end
    end
    initial begin
        for(word_index=0;word_index<8;word_index=word_index+1)
            imu_payload[255-word_index*32-:32]=sample_word(0,word_index);
        for(word_index=0;word_index<22;word_index=word_index+1)
            motor_payload[703-word_index*32-:32]=sample_word(1,word_index);
        for(word_index=0;word_index<36;word_index=word_index+1)
            lidar_payload[1151-word_index*32-:32]=sample_word(2,word_index);
        for(word_index=0;word_index<26;word_index=word_index+1)
            fusion_payload[831-word_index*32-:32]=sample_word(3,word_index);
        fusion_payload[31:0]=32'hD6C0FFEE;
        for(word_index=0;word_index<28;word_index=word_index+1)
            health_payload[895-word_index*32-:32]=sample_word(4,word_index);
        health_payload[31:0]=32'h475200A9;
        for(word_index=0;word_index<17;word_index=word_index+1)
            navigation_payload[543-word_index*32-:32]=sample_word(5,word_index);

        repeat(12)@(negedge bus_clk);
        reset_p=1'b0;
        wait(bus_reset===1'b0);
        wait(raw_telemetry.src_reset===1'b0 && control_telemetry.src_reset===1'b0 &&
             raw_tx_reset===1'b0 && control_tx_reset===1'b0);
        fork
            begin
                @(negedge raw_clk);raw_imu_valid=1;raw_lidar_valid=1;raw_health_valid=1;
                @(posedge raw_clk);#1;
                expected_timestamp[3]=raw_telemetry.snapshot_time[0];
                expected_timestamp[5]=raw_telemetry.snapshot_time[2];
                expected_timestamp[7]=raw_telemetry.snapshot_time[4];
                @(negedge raw_clk);raw_imu_valid=0;raw_lidar_valid=0;raw_health_valid=0;
            end
            begin
                @(negedge control_clk);control_motor_valid=1;control_fusion_valid=1;control_navigation_valid=1;
                @(posedge control_clk);#1;
                expected_timestamp[4]=control_telemetry.snapshot_time[1];
                expected_timestamp[6]=control_telemetry.snapshot_time[3];
                expected_timestamp[8]=control_telemetry.snapshot_time[5];
                @(negedge control_clk);control_motor_valid=0;control_fusion_valid=0;control_navigation_valid=0;
            end
        join

        // 暂停公共总线消费，检查mux在start前保持首字节。
        // 六个有限包不会填满生产版16槽descriptor；满FIFO另由CDC定向测试覆盖。
        guard=0;
        while(!mux_valid && guard<20000)begin
            @(negedge bus_clk);guard=guard+1;
        end
        if(guard>=20000)$fatal(1,"telemetry mux did not offer first packet");
        held_mux_byte=mux_data;
        if(raw_read || control_read || cdc_packet_read)
            $fatal(1,"unexpected read/descriptor state during downstream stall");
        repeat(12)begin
            @(negedge bus_clk);
            if(!mux_valid || cdc_packet_start || mux_data!==held_mux_byte ||
               raw_read || control_read)
                $fatal(1,"packet boundary changed while source bus was backpressured");
        end
        copy_allow=1'b1;
        guard=0;
        while(!cdc_valid && guard<20000)begin
            @(negedge tx_clk);guard=guard+1;
        end
        if(guard>=20000 || !cdc_nonzero)$fatal(1,"CDC did not offer complete packet");
        held_cdc_byte=cdc_data;
        repeat(12)begin
            @(negedge tx_clk);
            if(!cdc_valid || cdc_data!==held_cdc_byte)
                $fatal(1,"125 MHz CDC OFFER changed during sender backpressure");
        end
        sender_allow=1'b1;

        guard=0;
        while(frame_count<6 && guard<50000)begin
            @(negedge tx_clk);guard=guard+1;
        end
        if(guard>=50000)$fatal(1,"telemetry Ethernet frames timeout count=%0d",frame_count);
        repeat(200)@(negedge tx_clk);
        if(frame_count!=6 || seen_kind[8:3]!==6'b111111)
            $fatal(1,"missing telemetry kinds: count=%0d mask=%b",frame_count,seen_kind[8:3]);
        if(raw_dropped!==160'd0 || control_dropped!==160'd0)
            $fatal(1,"unexpected packetizer drops");
        $display("PASS: full dual-clock telemetry path, all six payloads, health signature, bounded backpressure, IPv4 checksum and Ethernet FCS");
        $finish;
    end

    initial begin
        #50000000;
        $fatal(1,"end-to-end telemetry path timeout frames=%0d",frame_count);
    end
endmodule

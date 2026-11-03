`timescale 1ns/1ps
// 连续队首变化覆盖握手到启动的间隔；逐字节检查真实GMII输出及FCS。
module nav_full_ack_telemetry_tb;
`ifdef BX72_VENDOR_PRIMITIVES
    // Gowin prim_sim.v 中 DFF/ODDR 通过全局实例 GSR.GSRO 读取启动复位。
    GSR GSR(.GSRI(1'b1));
`endif
    reg src=0,tx=0,reset_n=0,fusion_valid=0;
    always #15 src=~src;
    always #4 tx=~tx;
    integer tx_cycle_count=0;
    always @(posedge tx) tx_cycle_count=tx_cycle_count+1;
    reg [255:0] replies[0:2];
    reg [831:0] fusion_payload=0;
    integer accepted=0,ack_seen=0,fusion_seen=0;
    wire reply_ready,tvalid,tstart,tread,tnonzero;
    wire [7:0] tlength,tdata;
    wire rgmii_gtxc;
    wire [3:0] rgmii_txd;
    wire rgmii_txen;
    wire [255:0] reply_data=accepted<3 ? replies[accepted] : {256{1'b1}};
    bx72_telemetry telemetry(.src_clk(src),.tx_clk(tx),.reset_n(reset_n),
        .imu_valid_i(1'b0),.imu_payload_i(256'd0),.motor_valid_i(1'b0),.motor_payload_i(704'd0),
        .lidar_valid_i(1'b0),.lidar_payload_i(1152'd0),
        .fusion_valid_i(fusion_valid),.fusion_payload_i(fusion_payload),
        .health_valid_i(1'b0),.health_payload_i(896'd0),
        .navigation_valid_i(1'b0),.navigation_payload_i(544'd0),
        .tx_valid_o(tvalid),.tx_length_o(tlength),.tx_length_nonzero_o(tnonzero),.tx_data_o(tdata),
        .tx_start_i(tstart),.tx_read_i(tread),.dropped_o());
    eth_udp_video_sender #(.IMAGE_WIDTH(8),.IMAGE_HEIGHT(8),.MAX_PIXELS_PER_PACKET(16),
        .FIFO_ADDR_WIDTH(4),.STATUS_INTERVAL_CYCLES(1000),.TELEMETRY_ENABLE(1)) sender(
        .clk50m(src),.clk125m(tx),.reset_p(!reset_n),.cam_pclk(src),
        .camera_vsync(1'b0),.camera_pixel_valid(1'b0),.camera_pixel(16'd0),.camera_timestamp(32'd0),
        .reply_valid(accepted<3 && reset_n),.reply_data(reply_data),.reply_ready(reply_ready),
        .rx_diagnostics_i(128'd0),.rx_diagnostics_raw_i(1'b0),
        .telemetry_valid(tvalid),.telemetry_length({8'd0,tlength}),.telemetry_length_nonzero(tnonzero),
        .telemetry_data(tdata),.telemetry_start(tstart),.telemetry_read_request(tread),
        .rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
    always @(posedge tx) if(reset_n && reply_ready && accepted<3) accepted<=accepted+1;

    function automatic [15:0] crc16_byte(input [15:0] previous,input [7:0] value);
        reg [15:0] crc;
        begin
            crc=previous^{value,8'd0};
            for(integer k=0;k<8;k=k+1)crc=crc[15] ? (crc<<1)^16'h1021 : crc<<1;
            crc16_byte=crc;
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
    reg [7:0] bytes[0:2047];
    integer length=0;
    reg ethernet_active=0;
    reg [31:0] crc,final_crc;
    reg [7:0] gmii_history[0:32767];
    reg [7:0] rgmii_bytes[0:2047];
    integer gmii_frame_start[0:255],gmii_frame_end[0:255],gmii_frame_start_cycle[0:255];
    integer gmii_history_length=0,gmii_frame_count=0;
    integer rgmii_length=0,rgmii_frame_count=0;
    reg [3:0] rgmii_low_nibble;
    reg rgmii_rise_en=0,rgmii_frame_active=0;
    always @(negedge tx) if(reset_n) begin
        if(sender.gmii_txen) begin
            if(!ethernet_active) begin
                if(gmii_frame_count>=256)$fatal(1,"too many GMII frames in test");
                gmii_frame_start[gmii_frame_count]=gmii_history_length;
                gmii_frame_start_cycle[gmii_frame_count]=tx_cycle_count;
            end
            bytes[length]=sender.gmii_txd;length=length+1;ethernet_active=1;
            gmii_history[gmii_history_length]=sender.gmii_txd;
            gmii_history_length=gmii_history_length+1;
        end else if(ethernet_active) begin
            crc=32'hffffffff;
            for(integer i=8;i<length-4;i=i+1)crc=crc32_byte(crc,bytes[i]);
            final_crc=~crc;
            if({bytes[length-1],bytes[length-2],bytes[length-3],bytes[length-4]}!=final_crc)
                $fatal(1,"Ethernet FCS mismatch");
            if({bytes[44],bytes[45]}==6103) begin
                if(ack_seen>=3 || {bytes[46],bytes[47]}!=40) $fatal(1,"ACK length/count");
                for(integer i=0;i<32;i=i+1)
                    if(bytes[50+i]!==replies[ack_seen][255-i*8-:8])
                        $fatal(1,"ACK overwritten across launch: packet=%0d byte=%0d got=%h",ack_seen,i,bytes[50+i]);
                ack_seen=ack_seen+1;
            end else if({bytes[44],bytes[45]}==6104) begin
                if({bytes[46],bytes[47]}!=128 || {bytes[50],bytes[51],bytes[52],bytes[53]}!=32'h42583732 || bytes[55]!=6)
                    $fatal(1,"fusion packet header/length");
                for(integer i=0;i<104;i=i+1)
                    if(bytes[66+i]!==fusion_payload[831-i*8-:8])
                        $fatal(1,"fusion payload overwritten: byte=%0d got=%h",i,bytes[66+i]);
                fusion_seen=fusion_seen+1;
            end
            gmii_frame_end[gmii_frame_count]=gmii_history_length;
            gmii_frame_count=gmii_frame_count+1;
            ethernet_active=0;length=0;
        end
    end
    // 用实际转发时钟重组RGMII双沿字节，验证EN相位、字节顺序及包含FCS的整帧延迟。
    always @(posedge rgmii_gtxc) if(reset_n) begin
        #1;
        rgmii_rise_en=rgmii_txen;
        if(rgmii_txen) begin
            if(!rgmii_frame_active) begin
                if(rgmii_frame_count>=256)$fatal(1,"too many RGMII frames in test");
                // GMII在上升沿NBA发布；下一沿进入新增快照级。官方ODDR先
                // 同沿采样D0/D1，再经过两级内部寄存及双沿输出，首次可观察
                // 字节位于GMII发布后第4沿；新增快照只增加其中1个周期。
                if(!ethernet_active ||
                   tx_cycle_count-gmii_frame_start_cycle[rgmii_frame_count]!=4)
                    $fatal(1,"GMII-to-RGMII start latency mismatch frame=%0d cycles=%0d",
                        rgmii_frame_count,tx_cycle_count-gmii_frame_start_cycle[rgmii_frame_count]);
                rgmii_length=0;
                rgmii_frame_active=1;
            end
            rgmii_low_nibble=rgmii_txd;
        end
    end
    always @(negedge rgmii_gtxc) if(reset_n) begin
        #1;
        if(rgmii_txen!==rgmii_rise_en)
            $fatal(1,"RGMII TX_EN changed between edges without TX_ER");
        if(rgmii_rise_en && rgmii_txen) begin
            rgmii_bytes[rgmii_length]={rgmii_txd,rgmii_low_nibble};
            rgmii_length=rgmii_length+1;
        end else if(rgmii_frame_active) begin
            if(gmii_frame_count<=rgmii_frame_count)
                $fatal(1,"RGMII frame completed before its GMII source frame");
            if(rgmii_length != gmii_frame_end[rgmii_frame_count]-gmii_frame_start[rgmii_frame_count])
                $fatal(1,"GMII/RGMII frame length mismatch frame=%0d gmii=%0d rgmii=%0d",
                    rgmii_frame_count,
                    gmii_frame_end[rgmii_frame_count]-gmii_frame_start[rgmii_frame_count],rgmii_length);
            for(integer i=0;i<rgmii_length;i=i+1)
                if(rgmii_bytes[i]!==gmii_history[gmii_frame_start[rgmii_frame_count]+i])
                    $fatal(1,"GMII/RGMII byte mismatch frame=%0d byte=%0d gmii=%h rgmii=%h",
                        rgmii_frame_count,i,gmii_history[gmii_frame_start[rgmii_frame_count]+i],rgmii_bytes[i]);
            rgmii_frame_count=rgmii_frame_count+1;
            rgmii_frame_active=0;
        end
    end
    reg [15:0] crc16;
    initial begin
        for(integer n=0;n<3;n=n+1) begin
            replies[n]={32'h42583732,8'd1,8'd4,16'd32,32'h00000101+n,
                64'h1122334455667788+n,80'h0102030405060708090a+n,16'd0};
            crc16=16'hffff;
            for(integer i=0;i<30;i=i+1)crc16=crc16_byte(crc16,replies[n][255-i*8-:8]);
            replies[n][15:0]=crc16;
        end
        for(integer i=0;i<26;i=i+1)fusion_payload[831-i*32-:32]=32'h12340000+i;
        fusion_payload[831-14*32-:32]=32'hd0000001;
        fusion_payload[31:0]=32'h5678babe;
        repeat(8)@(negedge src);reset_n=1;
        repeat(8)@(negedge src);fusion_valid=1;
        @(negedge src);fusion_valid=0;
        wait(ack_seen==3 && fusion_seen==1);
        repeat(50)@(negedge tx);
        if(accepted!=3) $fatal(1,"ACK queue handshake count");
        if(rgmii_frame_count!=gmii_frame_count)
            $fatal(1,"GMII/RGMII frame drain mismatch gmii=%0d rgmii=%0d",gmii_frame_count,rgmii_frame_count);
        $display("PASS: queued ACKs, full fusion payload, GMII FCS and delayed RGMII nibble/EN stream");
        $finish;
    end
    initial begin #1000000;$fatal(1,"timeout ACK=%0d fusion=%0d",ack_seen,fusion_seen);end
endmodule

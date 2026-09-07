`timescale 1ns/1ps

// 低资源加速度倾角代理：IIR 平滑后把 X/Y 重力分量映射为屏幕像素偏移。
// 输出不是角度，也不是含磁力计/yaw 的完整姿态融合，命名刻意保留 proxy。
module choice3_attitude_tilt_proxy (
    input wire clk_i,input wire rst_n_i,input wire sample_valid_i,
    input wire signed [15:0] accel_x_i,input wire signed [15:0] accel_y_i,
    input wire signed [15:0] accel_z_i,
    output reg signed [9:0] roll_proxy_px_o,
    output reg signed [9:0] pitch_proxy_px_o,output reg valid_o
);
    reg signed [19:0] filt_x_q,filt_y_q,filt_z_q;
    reg [3:0] sample_count_q;
    reg signed [19:0] roll_scaled,pitch_scaled;
    function signed [9:0] clip80;
        input signed [19:0] value;
        begin
            if(value>80)clip80=10'sd80;
            else if(value<-80)clip80=-10'sd80;
            else clip80=value[9:0];
        end
    endfunction
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i)begin
            filt_x_q<=0;filt_y_q<=0;filt_z_q<=0;sample_count_q<=0;
            roll_proxy_px_o<=0;pitch_proxy_px_o<=0;valid_o<=0;
        end else if(sample_valid_i) begin
            filt_x_q<=filt_x_q+(({{4{accel_x_i[15]}},accel_x_i}-filt_x_q)>>>3);
            filt_y_q<=filt_y_q+(({{4{accel_y_i[15]}},accel_y_i}-filt_y_q)>>>3);
            filt_z_q<=filt_z_q+(({{4{accel_z_i[15]}},accel_z_i}-filt_z_q)>>>3);
            // ICM 配置为 ±16g 时约2048 LSB/g，映射约64 px/g。
            roll_scaled=filt_y_q>>>5;pitch_scaled=-(filt_x_q>>>5);
            roll_proxy_px_o<=clip80(roll_scaled);
            pitch_proxy_px_o<=clip80(pitch_scaled);
            if(sample_count_q<8)sample_count_q<=sample_count_q+1'b1;
            if(sample_count_q>=7)valid_o<=1;
        end
    end
endmodule

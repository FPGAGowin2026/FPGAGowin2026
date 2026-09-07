`timescale 1ns/1ps

// 姿态快照 CDC：源域先锁存多位总线再翻转 toggle；目的域把总线同步两拍，
// 事件额外延迟一拍后才提交，保证采样的是稳定快照而非裸跨域总线。
// dst_valid_o 是“已有姿态可显示”的 sticky valid，不是单拍脉冲。
module choice3_attitude_cdc (
    input wire src_clk_i,input wire src_rst_n_i,input wire src_valid_i,
    input wire signed [9:0] src_roll_i,input wire signed [9:0] src_pitch_i,
    input wire dst_clk_i,input wire dst_rst_n_i,
    output reg signed [9:0] dst_roll_o,output reg signed [9:0] dst_pitch_o,
    output reg dst_valid_o
);
    reg signed [9:0] src_roll_q,src_pitch_q;
    reg src_toggle_q;
    (* ASYNC_REG="TRUE" *) reg toggle_meta_q,toggle_sync_q,toggle_delay_q;
    (* ASYNC_REG="TRUE" *) reg signed [9:0] roll_meta_q,roll_sync_q;
    (* ASYNC_REG="TRUE" *) reg signed [9:0] pitch_meta_q,pitch_sync_q;
    reg toggle_previous_q;
    always @(posedge src_clk_i or negedge src_rst_n_i) begin
        if(!src_rst_n_i)begin src_roll_q<=0;src_pitch_q<=0;src_toggle_q<=0;end
        else if(src_valid_i)begin
            src_roll_q<=src_roll_i;src_pitch_q<=src_pitch_i;src_toggle_q<=~src_toggle_q;
        end
    end
    always @(posedge dst_clk_i or negedge dst_rst_n_i) begin
        if(!dst_rst_n_i)begin
            toggle_meta_q<=0;toggle_sync_q<=0;toggle_delay_q<=0;toggle_previous_q<=0;
            roll_meta_q<=0;roll_sync_q<=0;pitch_meta_q<=0;pitch_sync_q<=0;
            dst_roll_o<=0;dst_pitch_o<=0;dst_valid_o<=0;
        end else begin
            toggle_meta_q<=src_toggle_q;toggle_sync_q<=toggle_meta_q;
            toggle_delay_q<=toggle_sync_q;toggle_previous_q<=toggle_delay_q;
            roll_meta_q<=src_roll_q;roll_sync_q<=roll_meta_q;
            pitch_meta_q<=src_pitch_q;pitch_sync_q<=pitch_meta_q;
            if(toggle_delay_q!=toggle_previous_q)begin
                dst_roll_o<=roll_sync_q;dst_pitch_o<=pitch_sync_q;dst_valid_o<=1;
            end
        end
    end
endmodule

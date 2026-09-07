`timescale 1ns/1ps

// 跨域事件 toggle 的新鲜度监视。source_ready 失效时组合撤销 health。
module choice3_event_freshness #(
    parameter integer TIMEOUT_CYCLES=25_000_000
) (
    input wire clk_i,input wire rst_n_i,input wire event_toggle_async_i,
    input wire source_ready_i,output wire health_o,output reg event_pulse_o
);
    localparam integer CW=(TIMEOUT_CYCLES<2)?1:$clog2(TIMEOUT_CYCLES+1);
    (* ASYNC_REG="TRUE" *) reg event_meta_q,event_sync_q;
    reg event_previous_q,event_seen_q;
    reg [CW-1:0] age_q;
    assign health_o=source_ready_i&&event_seen_q&&(age_q<TIMEOUT_CYCLES);
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i)begin
            event_meta_q<=0;event_sync_q<=0;event_previous_q<=0;
            event_seen_q<=0;age_q<=TIMEOUT_CYCLES;event_pulse_o<=0;
        end else begin
            event_meta_q<=event_toggle_async_i;event_sync_q<=event_meta_q;
            event_previous_q<=event_sync_q;event_pulse_o<=0;
            if(event_sync_q!=event_previous_q)begin
                event_seen_q<=1;age_q<=0;event_pulse_o<=1;
            end else if(age_q<TIMEOUT_CYCLES)age_q<=age_q+1'b1;
            if(!source_ready_i)event_seen_q<=0;
        end
    end
endmodule

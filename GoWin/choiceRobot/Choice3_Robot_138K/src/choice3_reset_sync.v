`timescale 1ns/1ps

// 异步置复位、同步释放。每个独立时钟域单独实例化。
module choice3_reset_sync(input wire clk_i,input wire arst_n_i,output wire rst_n_o);
    (* ASYNC_REG="TRUE" *) reg [2:0] sync_q;
    always @(posedge clk_i or negedge arst_n_i) begin
        if(!arst_n_i)sync_q<=0;else sync_q<={sync_q[1:0],1'b1};
    end
    assign rst_n_o=sync_q[2];
endmodule

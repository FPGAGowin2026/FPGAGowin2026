`timescale 1ns/1ps

// 标准 DVI/HDMI TMDS 8b/10b 编码器。
module choice3_tmds_encoder (
    input wire pixel_clk_i,input wire rst_n_i,input wire de_i,
    input wire [1:0] control_i,input wire [7:0] data_i,
    output reg [9:0] code_o
);
    integer i;
    reg [3:0] ones_data,ones_qm;
    reg [8:0] qm;
    reg signed [5:0] disparity_q;
    reg use_xnor;
    reg signed [5:0] balance;
    always @* begin
        ones_data=0;for(i=0;i<8;i=i+1)ones_data=ones_data+data_i[i];
        use_xnor=(ones_data>4)||((ones_data==4)&&!data_i[0]);
        qm[0]=data_i[0];
        for(i=1;i<8;i=i+1) qm[i]=use_xnor?~(qm[i-1]^data_i[i]):(qm[i-1]^data_i[i]);
        qm[8]=!use_xnor;
        ones_qm=0;for(i=0;i<8;i=i+1)ones_qm=ones_qm+qm[i];
        balance=$signed({1'b0,ones_qm,1'b0})-6'sd8;
    end
    always @(posedge pixel_clk_i or negedge rst_n_i) begin
        if(!rst_n_i)begin code_o<=10'b1101010100;disparity_q<=0;end
        else if(!de_i)begin
            disparity_q<=0;
            case(control_i)
                2'b00:code_o<=10'b1101010100;2'b01:code_o<=10'b0010101011;
                2'b10:code_o<=10'b0101010100;default:code_o<=10'b1010101011;
            endcase
        end else if(disparity_q==0 || ones_qm==4) begin
            code_o[9]<=~qm[8];code_o[8]<=qm[8];
            code_o[7:0]<=qm[8]?qm[7:0]:~qm[7:0];
            disparity_q<=qm[8]?(disparity_q+balance):(disparity_q-balance);
        end else if((disparity_q>0 && ones_qm>4)||(disparity_q<0 && ones_qm<4)) begin
            code_o<={1'b1,qm[8],~qm[7:0]};
            disparity_q<=disparity_q-balance+(qm[8]?6'sd2:6'sd0);
        end else begin
            code_o<={1'b0,qm[8],qm[7:0]};
            disparity_q<=disparity_q+balance-(qm[8]?6'sd0:6'sd2);
        end
    end
endmodule

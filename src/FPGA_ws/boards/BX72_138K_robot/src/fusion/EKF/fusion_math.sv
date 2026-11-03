`timescale 1ns/1ps
// 有符号 48/32 位恢复除法，向零截断，48 拍；用于 K=P*65536/S。
module fusion_divider(input wire clk, rst, start_i,
    input wire signed [47:0] numerator_i, input wire signed [31:0] denominator_i,
    output reg done_o, output reg signed [31:0] quotient_o);
    reg busy, negative;
    reg [47:0] dividend, quotient;
    reg [31:0] divisor;
    reg [32:0] remainder;
    reg [5:0] count;
    wire [32:0] trial = {remainder[31:0], dividend[47]};
    wire ge = trial >= {1'b0, divisor};
    wire [47:0] next_q = {quotient[46:0], ge};
    always @(posedge clk) begin
        if (rst) begin busy<=0; done_o<=0; quotient_o<=0; end
        else begin
            done_o<=0;
            if (start_i && !busy) begin
                busy<=1; negative<=numerator_i[47] ^ denominator_i[31];
                dividend<=numerator_i[47] ? -numerator_i : numerator_i;
                divisor<=denominator_i[31] ? -denominator_i : denominator_i;
                remainder<=0; quotient<=0; count<=0;
            end else if (busy) begin
                dividend<={dividend[46:0],1'b0};
                remainder<=ge ? trial-{1'b0,divisor} : trial;
                quotient<=next_q;
                if (count==47) begin
                    busy<=0; done_o<=1;
                    if (divisor==0 || next_q > (negative ? 48'd2147483648 : 48'd2147483647))
                        quotient_o<=negative ? 32'sh80000000 : 32'sh7fffffff;
                    else quotient_o<=negative ? -$signed(next_q[31:0]) : next_q[31:0];
                end else count<=count+1'b1;
            end
        end
    end
endmodule

// 四轮测速的唯一正运动学实现；以分拍加法与共享恢复除法生成车体速度。
// 输入测量顺序固定为 FL/FR/RL/RR，速度单位 mm/s，角速度单位 mrad/s。
module fusion_mecanum_odometry #(parameter integer GEOMETRY_MM=178)(
    input wire clk,rst,sample_i,input wire [127:0] measured_i,
    output reg valid_o,output reg signed [31:0] vx_o,vy_o,wz_o);
    reg [2:0] phase;
    reg signed [32:0] plus_front,plus_rear,minus_front,minus_rear;
    reg signed [33:0] sum_x,sum_y,sum_w;
    reg signed [47:0] numerator;
    wire done;
    wire signed [31:0] quotient;
    wire signed [32:0] fl={measured_i[31],measured_i[0+:32]};
    wire signed [32:0] fr={measured_i[63],measured_i[32+:32]};
    wire signed [32:0] rl={measured_i[95],measured_i[64+:32]};
    wire signed [32:0] rr={measured_i[127],measured_i[96+:32]};
    fusion_divider div(.clk(clk),.rst(rst),.start_i(phase==3),
        .numerator_i(numerator),.denominator_i(4*GEOMETRY_MM),.done_o(done),.quotient_o(quotient));

    function automatic signed [31:0] divide_by_four(input signed [33:0] value);
        reg signed [33:0] adjusted;
        begin
            // 补偿算术右移的负数向下取整，保持有符号除法向零截断。
            adjusted=value[33] ? value+34'sd3 : value;
            divide_by_four=adjusted>>>2;
        end
    endfunction

    always @(posedge clk) begin
        if(rst)begin phase<=0;valid_o<=0;vx_o<=0;vy_o<=0;wz_o<=0;end
        else begin
            valid_o<=0;
            case(phase)
                0: if(sample_i)begin
                    plus_front<=fl+fr;plus_rear<=rl+rr;
                    minus_front<=-fl+fr;minus_rear<=rl-rr;phase<=1;
                end
                1:begin sum_x<=plus_front+plus_rear;sum_y<=minus_front+minus_rear;sum_w<=minus_front-minus_rear;phase<=2;end
                2:begin vx_o<=divide_by_four(sum_x);vy_o<=divide_by_four(sum_y);numerator<=$signed(sum_w)*48'sd1000;phase<=3;end
                3:phase<=4;
                4:if(done)begin wz_o<=quotient;valid_o<=1;phase<=0;end
                default:phase<=0;
            endcase
        end
    end
endmodule

// 请求/应答邮箱。源数据保持到应答返回，不对多位总线逐位同步。
module fusion_mailbox #(parameter integer WIDTH=96)(
    input wire src_clk, dst_clk, rst_n, input wire send_i,
    input wire [WIDTH-1:0] data_i, output wire ready_o,
    output reg valid_o, output reg [WIDTH-1:0] data_o,
    output reg [31:0] dropped_o);
    reg request, ack;
    (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync, req_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] src_reset_sync, dst_reset_sync;
    // 各域异步置位、同步释放；任一时钟暂停时，该域保持复位。
    always @(posedge src_clk or negedge rst_n)
        if(!rst_n) src_reset_sync<=0; else src_reset_sync<={src_reset_sync[0],1'b1};
    always @(posedge dst_clk or negedge rst_n)
        if(!rst_n) dst_reset_sync<=0; else dst_reset_sync<={dst_reset_sync[0],1'b1};
    wire src_rst_n=src_reset_sync[1], dst_rst_n=dst_reset_sync[1];
    reg [WIDTH-1:0] held;
    assign ready_o = src_rst_n && request == ack_sync[1];
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin request<=0; ack_sync<=0; held<=0; dropped_o<=0; end
        else begin
            ack_sync<={ack_sync[0],ack};
            if (send_i) begin
                if (ready_o) begin held<=data_i; request<=~request; end
                else dropped_o<=dropped_o+1'b1;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin req_sync<=0; ack<=0; valid_o<=0; data_o<=0; end
        else begin
            req_sync<={req_sync[0],request}; valid_o<=0;
            if(req_sync[1]!=ack) begin data_o<=held; valid_o<=1; ack<=req_sync[1]; end
        end
    end
endmodule

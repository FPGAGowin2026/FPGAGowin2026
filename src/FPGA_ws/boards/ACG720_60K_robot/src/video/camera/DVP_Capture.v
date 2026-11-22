/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : DVP_Capture
// Description   : CMOS摄像头DVP接口控制逻辑
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module DVP_Capture (
    input wire Rst_n, PCLK, Vsync, Href,
    input wire [7:0] Data,
    output wire DataValid,
    output reg [15:0] DataPixel,
    output wire DataHs, DataVs
);
    reg vsync_q, vsync_prev, href_q;
    reg [7:0] data_q;
    reg byte_phase, pixel_valid, line_active, frame_active;
    reg [3:0] warmup_frames;
    reg capture_enabled;

    always @(posedge PCLK) begin
        vsync_q <= Vsync;
        vsync_prev <= vsync_q;
        href_q <= Href;
        data_q <= Data;
        line_active <= href_q;
        frame_active <= !vsync_q;
    end

    always @(posedge PCLK or negedge Rst_n) begin
        if (!Rst_n) begin
            byte_phase <= 0;
            DataPixel <= 0;
            pixel_valid <= 0;
            warmup_frames <= 0;
            capture_enabled <= 0;
        end else begin
            byte_phase <= href_q && !byte_phase;
            if (!byte_phase) DataPixel[15:8] <= data_q;
            else DataPixel[7:0] <= data_q;
            pixel_valid <= byte_phase && href_q;

            // Detect frame edges after the input register, keeping raw VSYNC
            // off the counter enable path. Warmup still ends in VSYNC blanking.
            if (!vsync_prev && vsync_q && warmup_frames < 10)
                warmup_frames <= warmup_frames + 1'b1;
            capture_enabled <= warmup_frames >= 10;
        end
    end

    assign DataValid = pixel_valid && capture_enabled;
    assign DataHs = line_active && capture_enabled;
    assign DataVs = frame_active && capture_enabled;
endmodule

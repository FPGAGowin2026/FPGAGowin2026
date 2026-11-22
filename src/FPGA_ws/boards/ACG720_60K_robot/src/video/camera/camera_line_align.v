module camera_line_align (
    input wire clk, reset_n,
    input wire frame_in, line_in, valid_in,
    input wire [15:0] pixel_in,
    output reg frame_out, line_out, valid_out,
    output reg [15:0] pixel_out
);
    localparam [9:0] WIDTH = 10'd800;
    localparam [9:0] MIN_SHORT_LINE = WIDTH - 10'd8;
    reg [9:0] pixel_count;

    wire input_line = frame_in && line_in;
    wire pad_line = !input_line && line_out &&
                    pixel_count >= MIN_SHORT_LINE && pixel_count < WIDTH;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            frame_out <= 0;
            line_out <= 0;
            valid_out <= 0;
            pixel_out <= 0;
            pixel_count <= 0;
        end else begin
            frame_out <= frame_in || pad_line;
            line_out <= input_line || pad_line;
            valid_out <= 0;
            if (input_line) begin
                if (valid_in && pixel_count < WIDTH) begin
                    pixel_out <= pixel_in;
                    valid_out <= 1;
                    pixel_count <= pixel_count + 1'b1;
                end
            end else if (pad_line) begin
                // 短行在水平消隐期重复末像素，后续行仍从正确列开始。
                valid_out <= 1;
                pixel_count <= pixel_count + 1'b1;
            end else begin
                pixel_count <= 0;
            end
        end
    end
endmodule

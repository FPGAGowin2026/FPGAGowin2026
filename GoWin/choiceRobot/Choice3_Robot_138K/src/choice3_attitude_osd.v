`timescale 1ns/1ps

// 640x480 画面左上角的 5x7 字形姿态角 OSD：R/P/Y + 符号 + 三位整数度。
(* syn_hier = "hard", keep_hierarchy = "yes" *) module choice3_attitude_osd (
    input  wire clk_i,
    input  wire rst_n_i,
    input  wire [9:0] x_i,
    input  wire [9:0] y_i,
    input  wire signed [15:0] roll_deg_i,
    input  wire signed [15:0] pitch_deg_i,
    input  wire signed [15:0] yaw_deg_i,
    input  wire valid_i,
    output reg hit_o,
    output reg [7:0] red_o,
    output reg [7:0] green_o,
    output reg [7:0] blue_o
);
    localparam [3:0] GLYPH_MINUS=4'd10, GLYPH_R=4'd11,
                     GLYPH_P=4'd12, GLYPH_Y=4'd13, GLYPH_BLANK=4'd14;

    reg [1:0] line_q;
    reg [2:0] row_q;
    reg [2:0] col_q;
    reg [2:0] char_q;
    reg in_cell_q;
    reg signed [15:0] angle_q;
    reg [15:0] magnitude_q;
    reg [3:0] glyph_q;
    reg [4:0] bits_q;
    reg [9:0] row_offset_q,col_offset_q;
    reg [15:0] hundreds_q,tens_q,ones_q;
    reg hit_d;
    reg [7:0] red_d,green_d,blue_d;

    function [4:0] glyph_row;
        input [3:0] glyph;
        input [2:0] row;
        begin
            case (glyph)
                0: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;default:glyph_row=5'b01110;endcase
                1: case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=5'b01110;endcase
                2: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;default:glyph_row=5'b11111;endcase
                3: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;default:glyph_row=5'b11110;endcase
                4: case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;default:glyph_row=5'b00010;endcase
                5: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;default:glyph_row=5'b11110;endcase
                6: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;default:glyph_row=5'b01110;endcase
                7: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;default:glyph_row=5'b01000;endcase
                8: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;default:glyph_row=5'b01110;endcase
                9: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;default:glyph_row=5'b01110;endcase
                GLYPH_MINUS: glyph_row=(row==3)?5'b11111:5'b00000;
                GLYPH_R: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;default:glyph_row=5'b10001;endcase
                GLYPH_P: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;default:glyph_row=5'b10000;endcase
                GLYPH_Y: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=5'b00100;endcase
                default: glyph_row=5'b00000;
            endcase
        end
    endfunction

    always @* begin
        line_q=0; row_q=0; col_q=0; char_q=0; in_cell_q=0;
        row_offset_q=0;col_offset_q=0;hundreds_q=0;tens_q=0;ones_q=0;
        angle_q=roll_deg_i; magnitude_q=0; glyph_q=GLYPH_BLANK; bits_q=0;
        if (y_i>=10'd24 && y_i<10'd38) begin line_q=0;row_offset_q=y_i-10'd24;angle_q=roll_deg_i;end
        else if (y_i>=10'd48 && y_i<10'd62) begin line_q=1;row_offset_q=y_i-10'd48;angle_q=pitch_deg_i;end
        else if (y_i>=10'd72 && y_i<10'd86) begin line_q=2;row_offset_q=y_i-10'd72;angle_q=yaw_deg_i;end
        else line_q=3;
        row_q=row_offset_q[3:1];

        if (x_i>=10'd8 && x_i<10'd18) begin char_q=0;col_offset_q=x_i-10'd8;in_cell_q=1;end
        else if (x_i>=10'd20 && x_i<10'd30) begin char_q=1;col_offset_q=x_i-10'd20;in_cell_q=1;end
        else if (x_i>=10'd32 && x_i<10'd42) begin char_q=2;col_offset_q=x_i-10'd32;in_cell_q=1;end
        else if (x_i>=10'd44 && x_i<10'd54) begin char_q=3;col_offset_q=x_i-10'd44;in_cell_q=1;end
        else if (x_i>=10'd56 && x_i<10'd66) begin char_q=4;col_offset_q=x_i-10'd56;in_cell_q=1;end
        col_q=col_offset_q[3:1];

        magnitude_q = angle_q[15] ? (~angle_q + 1'b1) : angle_q;
        if (magnitude_q>16'd999) magnitude_q=16'd999;
        hundreds_q=magnitude_q/16'd100;
        tens_q=(magnitude_q/16'd10)%16'd10;
        ones_q=magnitude_q%16'd10;
        case (char_q)
            0: glyph_q=(line_q==0)?GLYPH_R:(line_q==1)?GLYPH_P:GLYPH_Y;
            1: glyph_q=angle_q[15]?GLYPH_MINUS:GLYPH_BLANK;
            2: glyph_q=hundreds_q[3:0];
            3: glyph_q=tens_q[3:0];
            default: glyph_q=ones_q[3:0];
        endcase
        bits_q=glyph_row(glyph_q,row_q);
        hit_d=valid_i && line_q!=3 && in_cell_q && bits_q[4-col_q];
        red_d=0;green_d=0;blue_d=0;
        case(line_q)
            0: red_d=8'hff;
            1: green_d=8'hff;
            2: begin green_d=8'hff;blue_d=8'hff;end
            default: begin end
        endcase
    end

    // Registered pixel output keeps the OSD as an auditable synthesis block.
    // VGA feeds pre-pipeline counters, so this stage remains aligned with the
    // one-cycle synchronous framebuffer/control pipeline.
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i)begin hit_o<=0;red_o<=0;green_o<=0;blue_o<=0;end
        else begin hit_o<=hit_d;red_o<=red_d;green_o<=green_d;blue_o<=blue_d;end
    end
endmodule

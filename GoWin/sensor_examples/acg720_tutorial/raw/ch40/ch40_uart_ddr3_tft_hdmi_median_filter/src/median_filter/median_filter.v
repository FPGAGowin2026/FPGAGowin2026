/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : median_filter
// Description   : 中值滤波模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module median_filter
#(
  parameter DATA_WIDTH = 8
)
(
  input                      clk,    //pixel clk
  input                      reset_p,
  input     [DATA_WIDTH-1:0] data_in,
  input                      data_in_valid,
  input                      data_in_hs,
  input                      data_in_vs,

  output    [DATA_WIDTH-1:0] data_out,
  output reg                 data_out_valid,
  output reg                 data_out_hs,
  output reg                 data_out_vs
);
//line data
wire [DATA_WIDTH-1:0] line0_data;
wire [DATA_WIDTH-1:0] line1_data;
wire [DATA_WIDTH-1:0] line2_data;
//matrix 3x3 data
reg  [DATA_WIDTH-1:0] row0_col0;
reg  [DATA_WIDTH-1:0] row0_col1;
reg  [DATA_WIDTH-1:0] row0_col2;

reg  [DATA_WIDTH-1:0] row1_col0;
reg  [DATA_WIDTH-1:0] row1_col1;
reg  [DATA_WIDTH-1:0] row1_col2;

reg  [DATA_WIDTH-1:0] row2_col0;
reg  [DATA_WIDTH-1:0] row2_col1;
reg  [DATA_WIDTH-1:0] row2_col2;

wire [DATA_WIDTH-1:0] line0_max;
wire [DATA_WIDTH-1:0] line0_mid;
wire [DATA_WIDTH-1:0] line0_min;

wire [DATA_WIDTH-1:0] line1_max;
wire [DATA_WIDTH-1:0] line1_mid;
wire [DATA_WIDTH-1:0] line1_min;

wire [DATA_WIDTH-1:0] line2_max;
wire [DATA_WIDTH-1:0] line2_mid;
wire [DATA_WIDTH-1:0] line2_min;

wire [DATA_WIDTH-1:0] max_max;
wire [DATA_WIDTH-1:0] max_mid;
wire [DATA_WIDTH-1:0] max_min;

wire [DATA_WIDTH-1:0] mid_max;
wire [DATA_WIDTH-1:0] mid_mid;
wire [DATA_WIDTH-1:0] mid_min;

wire [DATA_WIDTH-1:0] min_max;
wire [DATA_WIDTH-1:0] min_mid;
wire [DATA_WIDTH-1:0] min_min;

wire [DATA_WIDTH-1:0] matrix_mid;
//
reg   data_in_valid_dly1;
reg   data_in_valid_dly2;
reg   data_in_valid_dly3;
reg   data_in_hs_dly1;
reg   data_in_hs_dly2;
reg   data_in_hs_dly3;
reg   data_in_vs_dly1;
reg   data_in_vs_dly2;
reg   data_in_vs_dly3;

//3xline data
shift_register_2taps
#(
  .DATA_WIDTH ( DATA_WIDTH )
)shift_register_2taps(
  .clk           (clk           ),
  .Reset         (reset_p       ),
  .shiftin       (data_in       ),
  .shiftin_valid (data_in_valid ),

  .shiftout      (              ),
  .taps0x        (line0_data    ),
  .taps1x        (line1_data    )
);

assign line2_data = data_in;

//----------------------------------------------------
// matrix 3x3 data
// row0_col0   row0_col1   row0_col2
// row1_col0   row1_col1   row1_col2
// row2_col0   row2_col1   row2_col2
//----------------------------------------------------
always @(posedge clk or posedge reset_p) begin
  if(reset_p) begin
    row0_col0 <= 'd0;
    row0_col1 <= 'd0;
    row0_col2 <= 'd0;

    row1_col0 <= 'd0;
    row1_col1 <= 'd0;
    row1_col2 <= 'd0;

    row2_col0 <= 'd0;
    row2_col1 <= 'd0;
    row2_col2 <= 'd0;
  end
  else if(data_in_hs && data_in_vs)
    if(data_in_valid) begin
      row0_col2 <= line0_data;
      row0_col1 <= row0_col2;
      row0_col0 <= row0_col1;

      row1_col2 <= line1_data;
      row1_col1 <= row1_col2;
      row1_col0 <= row1_col1;

      row2_col2 <= line2_data;
      row2_col1 <= row2_col2;
      row2_col0 <= row2_col1;
    end
    else begin
      row0_col2 <= row0_col2;
      row0_col1 <= row0_col1;
      row0_col0 <= row0_col0;

      row1_col2 <= row1_col2;
      row1_col1 <= row1_col1;
      row1_col0 <= row1_col0;

      row2_col2 <= row2_col2;
      row2_col1 <= row2_col1;
      row2_col0 <= row2_col0;
    end
  else begin
    row0_col0 <= 'd0;
    row0_col1 <= 'd0;
    row0_col2 <= 'd0;

    row1_col0 <= 'd0;
    row1_col1 <= 'd0;
    row1_col2 <= 'd0;

    row2_col0 <= 'd0;
    row2_col1 <= 'd0;
    row2_col2 <= 'd0;
  end
end

always @(posedge clk)
begin
  data_in_valid_dly1 <= data_in_valid;
  data_in_valid_dly2 <= data_in_valid_dly1;
  data_in_valid_dly3 <= data_in_valid_dly2;

  data_in_hs_dly1    <= data_in_hs;
  data_in_hs_dly2    <= data_in_hs_dly1;
  data_in_hs_dly3    <= data_in_hs_dly2;

  data_in_vs_dly1    <= data_in_vs;
  data_in_vs_dly2    <= data_in_vs_dly1;
  data_in_vs_dly3    <= data_in_vs_dly2;
end

//----------------------------------------------------
// line0 of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_line0
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly1 ),
  .data0_in      (row0_col0          ),
  .data1_in      (row0_col1          ),
  .data2_in      (row0_col2          ),

  .data_max_out  (line0_max          ),
  .data_mid_out  (line0_mid          ),
  .data_min_out  (line0_min          ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// line1 of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_line1
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly1 ),
  .data0_in      (row1_col0          ),
  .data1_in      (row1_col1          ),
  .data2_in      (row1_col2          ),

  .data_max_out  (line1_max          ),
  .data_mid_out  (line1_mid          ),
  .data_min_out  (line1_min          ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// line1 of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_line2
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly1 ),
  .data0_in      (row2_col0          ),
  .data1_in      (row2_col1          ),
  .data2_in      (row2_col2          ),

  .data_max_out  (line2_max          ),
  .data_mid_out  (line2_mid          ),
  .data_min_out  (line2_min          ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// max of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_max
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly2 ),
  .data0_in      (line0_max          ),
  .data1_in      (line1_max          ),
  .data2_in      (line2_max          ),

  .data_max_out  (max_max            ),
  .data_mid_out  (max_mid            ),
  .data_min_out  (max_min            ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// mid of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_mid
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly2 ),
  .data0_in      (line0_mid          ),
  .data1_in      (line1_mid          ),
  .data2_in      (line2_mid          ),

  .data_max_out  (mid_max            ),
  .data_mid_out  (mid_mid            ),
  .data_min_out  (mid_min            ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// min of (max mid min)
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_min
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly2 ),
  .data0_in      (line0_min          ),
  .data1_in      (line1_min          ),
  .data2_in      (line2_min          ),

  .data_max_out  (min_max            ),
  .data_mid_out  (min_mid            ),
  .data_min_out  (min_min            ),
  .data_out_valid(                   )
);

//----------------------------------------------------
// matrix 3x3 of mid
//----------------------------------------------------
sort
#(
  .DATA_WIDTH (DATA_WIDTH)
)sort_matrix_mid
(
  .clk           (clk                ),    //pixel clk
  .reset_p       (reset_p            ),
  .data_in_valid (data_in_valid_dly3 ),
  .data0_in      (max_min            ),
  .data1_in      (mid_mid            ),
  .data2_in      (min_max            ),

  .data_max_out  (                   ),
  .data_mid_out  (matrix_mid         ),
  .data_min_out  (                   ),
  .data_out_valid(                   )
);

//----------------------------------------------------
//result
//----------------------------------------------------
assign data_out = matrix_mid;

always @(posedge clk)
begin
  data_out_valid <= data_in_valid_dly3;
  data_out_hs    <= data_in_hs_dly3;
  data_out_vs    <= data_in_vs_dly3;
end

endmodule
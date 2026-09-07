/////////////////////////////////////////////////////////////////////////////////
// Company: 姝︽眽鑺矾鎭掔鎶�鏈夐檺鍏徃
// Engineer: 灏忔鍝ュ洟闃�
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: char_extract
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module char_extract
#(
  parameter H_Visible_area    = 800, //灞忓箷鏄剧ず鍖哄煙瀹藉害
  parameter V_Visible_area    = 480, //灞忓箷鏄剧ず鍖哄煙楂樺害
  parameter CHAR_WIDTH        = 16 , //鍗曚釜瀛楃鏄剧ず瀹藉害
  parameter CHAR_HEIGHT       = 32 , //鍗曚釜瀛楃鏄剧ず楂樺害
  parameter ROW_DISP_CHAR_NUM = 10 , //涓�琛屾樉绀哄瓧绗︿釜鏁�
  parameter COL_DISP_CHAR_NUM = 2  , //鏄剧ず瀛楃琛屾暟
  parameter CHAR_ROM_ADDR_W   = 5  , //瀛樺偍瀛楃ROM鍦板潃浣嶅锛宭og2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
  parameter DISP_DATA_W       = 16   //鍥剧墖鍍忕礌鐐规暟鎹綅瀹�
)
(
  clk_ctrl,
  reset_n,
  char_disp_hbegin,
  char_disp_vbegin,
  disp_back_color,
  disp_char_color,
  Frame_Begin,
  rom_addra,
  rom_data,
  disp_data_req,
  visible_hcount,
  visible_vcount,
  disp_data
);
localparam DISP_CHAR_TOTAL_W = CHAR_WIDTH  * ROW_DISP_CHAR_NUM;
localparam DISP_CHAR_TOTAL_H = CHAR_HEIGHT * COL_DISP_CHAR_NUM;

  input                         clk_ctrl;         //鏃堕挓杈撳叆锛屼笌TFT灞忔椂閽熶繚鎸佷竴鑷�
  input                         reset_n;          //澶嶄綅淇″彿锛屼綆鐢靛钩鏈夋晥

  input  [15:0]                 char_disp_hbegin; //寰呮樉绀哄浘鐗囧乏涓婅绗竴涓儚绱犵偣鍦═FT灞忕殑琛屽悜鍧愭爣
  input  [15:0]                 char_disp_vbegin; //寰呮樉绀哄浘鐗囧乏涓婅绗竴涓儚绱犵偣鍦═FT灞忕殑鍦哄悜鍧愭爣
  input  [DISP_DATA_W-1:0]      disp_back_color;  //鏄剧ず鐨勮儗鏅鑹�
  input  [DISP_DATA_W-1:0]      disp_char_color;  //鏄剧ず瀛楃鐨勯鑹�

  input                         Frame_Begin;      //涓�甯у浘鍍忚捣濮嬫爣璇嗕俊鍙凤紝clk_ctrl鏃堕挓鍩�
  output[CHAR_ROM_ADDR_W-1:0]   rom_addra;        //璇诲瓧绗︽暟鎹畆om鍦板潃                  
  input [DISP_CHAR_TOTAL_W-1:0] rom_data;         //璇诲嚭瀛楃鏁版嵁
  input                         disp_data_req;    //
  input  [11:0]                 visible_hcount;   //TFT鍙鍖哄煙琛屾壂鎻忚鏁板櫒
  input  [11:0]                 visible_vcount;   //TFT鍙鍖哄煙鍦烘壂鎻忚鏁板櫒
  output [DISP_DATA_W-1:0]      disp_data;        //寰呮樉绀烘暟鎹�

  reg  [CHAR_ROM_ADDR_W-1:0]   rom_addra;        //璇诲浘鐗囨暟鎹畆om鍦板潃  
  wire                         h_exceed;
  wire                         v_exceed;
  wire                         char_h_disp;
  wire                         char_v_disp;
  wire                         char_disp;
  wire [15:0]                  hcount_max;
  reg  [DISP_CHAR_TOTAL_W-1:0] char_flag;

  //鍒ゆ柇璁剧疆鐨勬樉绀虹殑璧峰浣嶇疆鏄惁浼氬鑷存樉绀鸿秴鍑鸿寖鍥�
  assign h_exceed = char_disp_hbegin + DISP_CHAR_TOTAL_W > H_Visible_area - 1'b1;
  assign v_exceed = char_disp_vbegin + DISP_CHAR_TOTAL_H > V_Visible_area - 1'b1;
  //涓嶅悓鐨勮缃儏鍐碉紝鏄剧ず鍖哄煙鍋氫笉鍚岀殑澶勭悊
  assign char_h_disp = h_exceed ? (visible_hcount >= char_disp_hbegin && visible_hcount < H_Visible_area):
                                  (visible_hcount >= char_disp_hbegin && visible_hcount < char_disp_hbegin + DISP_CHAR_TOTAL_W);  
  
  assign char_v_disp = v_exceed ? (visible_vcount >= char_disp_vbegin && visible_vcount < V_Visible_area):
                                  (visible_vcount >= char_disp_vbegin && visible_vcount < char_disp_vbegin + DISP_CHAR_TOTAL_H);
     
  assign char_disp = disp_data_req && char_h_disp && char_v_disp;
  
  assign hcount_max = h_exceed ? (H_Visible_area - 1'b1):(char_disp_hbegin + DISP_CHAR_TOTAL_W - 1'b1);
     
  always@(posedge clk_ctrl or negedge reset_n)
  begin
    if(!reset_n)
      rom_addra <= 'd0;
    else if(Frame_Begin)
      rom_addra <= 'd0; 
    else if((visible_hcount == hcount_max) && char_disp)
      rom_addra <= rom_addra + 1'b1;
    else
      rom_addra <= rom_addra;
  end

  always@(posedge clk_ctrl or negedge reset_n)
  begin
    if(!reset_n)
      char_flag <= 'd0;
    else if(char_disp)
      char_flag <= {char_flag[DISP_CHAR_TOTAL_W-2:0],1'b0};
    else
      char_flag <= rom_data;
  end

  assign disp_data = (char_disp && char_flag[DISP_CHAR_TOTAL_W-1]) ? disp_char_color : disp_back_color;

 endmodule
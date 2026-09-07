module key_control
(
  input CLK125M,
  input Reset_n,
  input key_ch,
  input key_fre,
  input key_wave,

  output reg [2:0] channel,  //channel：0：通道1；1：通道2
  output reg [2:0] CH_Fword_Sel_A,
  output reg [2:0] CH_Fword_Sel_B,
  output reg [1:0] Mode_Sel_A,
  output reg [1:0] Mode_Sel_B, 
  output reg [1:0] led
);
    wire [2:0]Key_Flag;
    wire [2:0]Key_State;
    reg [31:0] cnt;

    //按键S0
    key_filter key_filter0(
        .Clk(CLK125M),
        .Reset_n(Reset_n),
        .Key(key_wave),
        .Key_Flag(Key_Flag[0]),
        .Key_State(Key_State[0])
    );   
    
    //按键S1
    key_filter key_filter1(
        .Clk(CLK125M),
        .Reset_n(Reset_n),
        .Key(key_fre),
        .Key_Flag(Key_Flag[1]),
        .Key_State(Key_State[1])
    );  
    
    //按键S2
    key_filter key_filter2(
        .Clk(CLK125M),
        .Reset_n(Reset_n),
        .Key(key_ch),
        .Key_Flag(Key_Flag[2]),
        .Key_State(Key_State[2])
    );  
    
    //按键S0控制波形
    always@(posedge CLK125M or negedge Reset_n)
    if(!Reset_n) begin
        Mode_Sel_A <= 2'd0;
        Mode_Sel_B <= 2'd0;
    end
    else if(Key_Flag[0] && (Key_State[0] == 0)) begin
        if(channel == 3'd0) begin
            Mode_Sel_A <= Mode_Sel_A + 1'd1;
            if(Mode_Sel_A >= 2'd2)
                Mode_Sel_A <= 2'd0;
        end
        else if(channel == 3'd1) begin
            Mode_Sel_B <= Mode_Sel_B + 1'd1; 
            if(Mode_Sel_B >= 2'd2)
                Mode_Sel_B <= 2'd0;
        end
    end
    else begin
        Mode_Sel_A <= Mode_Sel_A;
        Mode_Sel_B <= Mode_Sel_B;
    end

    //按键S1控制频率
    always@(posedge CLK125M or negedge Reset_n)
    if(!Reset_n) begin
        CH_Fword_Sel_A <= 3'd0;
        CH_Fword_Sel_B <= 3'd0;
    end
    else if(Key_Flag[1] && (Key_State[1] == 0)) begin
        if(channel == 3'd0)
            CH_Fword_Sel_A <= CH_Fword_Sel_A + 1'd1;
        else if(channel == 3'd1)
            CH_Fword_Sel_B <= CH_Fword_Sel_B + 1'd1;
    end
    else begin
        CH_Fword_Sel_A <= CH_Fword_Sel_A;
        CH_Fword_Sel_B <= CH_Fword_Sel_B;
    end

    //按键S2控制通道0还是通道1
    always@(posedge CLK125M or negedge Reset_n)
    if(!Reset_n)
        channel <= 3'd0;
    else if(Key_Flag[2] && (Key_State[2] == 0))
        channel <= channel + 1'd1;
    else if(channel >= 3'd2)
        channel <= 3'd0;
    

    always@(posedge CLK125M or negedge Reset_n)
    begin
    if (!Reset_n)        
        cnt <= 32'd0;
    else if (cnt >= 32'd62499999)
        cnt <= 32'd0;
    else
        cnt <= cnt + 1'd1;
    end
    
    always @ (posedge CLK125M or negedge Reset_n)
    begin
    if (!Reset_n) 
        led <= 2'd0;
    else if (cnt == 32'd62499999) begin
        if(channel == 2'd0) begin
            led[0] <= ~led[0];
            led[1] <= 1'd1;
        end
        else if(channel == 2'd1) begin
            led[1] <= ~led[1];
            led[0] <= 1'd1;
        end
    end
    else
        led <= led;
    end

endmodule
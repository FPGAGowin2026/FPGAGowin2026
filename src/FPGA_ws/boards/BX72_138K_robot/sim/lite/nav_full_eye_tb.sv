`timescale 1ns/1ps
// 比较分拍前后的最终选窗结果；各自等待done，不把新增流水延迟当作算法差异。
module nav_full_eye_tb;
    reg clk=0,reset_n=0,start=0;
    always #10 clk=~clk;
    reg [1023:0] pass_map=0,blocked_map=0;
    wire done,ref_done;
    wire [46:0] result,ref_result;
    ov5647_autocal_eye_select dut(
        .byte_clk(clk),.reset_n(reset_n),.start(start),.pass_map(pass_map),.blocked_map(blocked_map),
        .busy(),.done(done),.found(result[46]),
        .top_out(result[45:41]),.bottom_out(result[40:36]),.left_out(result[35:31]),.right_out(result[30:26]),
        .center_row_index(result[25:21]),.center_col_index(result[20:16]),
        .center_tap_lane0(result[15:8]),.center_tap_lane1(result[7:0]));
    ov5647_autocal_eye_select_baseline reference(
        .byte_clk(clk),.reset_n(reset_n),.start(start),.pass_map(pass_map),.blocked_map(blocked_map),
        .busy(),.done(ref_done),.found(ref_result[46]),
        .top_out(ref_result[45:41]),.bottom_out(ref_result[40:36]),.left_out(ref_result[35:31]),.right_out(ref_result[30:26]),
        .center_row_index(ref_result[25:21]),.center_col_index(ref_result[20:16]),
        .center_tap_lane0(ref_result[15:8]),.center_tap_lane1(ref_result[7:0]));
    integer cases=0,cycles;
    reg saw_new,saw_old;
    reg [46:0] saved_new,saved_old;
    task automatic compare_case;
        begin
            @(negedge clk);start=1;saw_new=0;saw_old=0;cycles=0;
            @(negedge clk);start=0;
            while((!saw_new||!saw_old)&&cycles<250000)begin
                @(negedge clk);cycles=cycles+1;
                if(done)begin saw_new=1;saved_new=result;end
                if(ref_done)begin saw_old=1;saved_old=ref_result;end
            end
            if(!saw_new||!saw_old)$fatal(1,"选窗未完成 case=%0d cycles=%0d",cases,cycles);
            if(saved_new!==saved_old)$fatal(1,"分拍选窗改变结果 case=%0d dut=%h ref=%h",cases,saved_new,saved_old);
            cases=cases+1;
        end
    endtask
    integer x,y,n;
    reg [31:0] random_word=32'h72c5184b;
    initial begin
        repeat(5)@(negedge clk);reset_n=1;
        compare_case();
        pass_map={1024{1'b1}};compare_case();
        pass_map=0;
        for(y=7;y<=21;y=y+1)for(x=4;x<=13;x=x+1)pass_map[y*32+x]=1;
        compare_case();
        blocked_map[12*32+8]=1;compare_case();
        blocked_map=0;pass_map=0;
        for(y=0;y<32;y=y+1)for(x=0;x<32;x=x+1)pass_map[y*32+x]=(x+y)%2;
        compare_case();
        for(n=0;n<6;n=n+1)begin
            for(x=0;x<1024;x=x+1)begin
                random_word={random_word[30:0],random_word[31]^random_word[21]^random_word[1]^random_word[0]};
                pass_map[x]=random_word[0]|random_word[1];
                blocked_map[x]=random_word[2]&random_word[3]&random_word[4];
            end
            compare_case();
        end
        $display("PASS: nav_full_eye cases=%0d final_rectangle_ties_and_center_match",cases);
        $finish;
    end
    initial begin #50000000;$fatal(1,"选窗测试全局超时");end
endmodule

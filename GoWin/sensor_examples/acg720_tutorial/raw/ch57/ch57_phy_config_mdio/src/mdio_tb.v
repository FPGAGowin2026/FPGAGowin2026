`timescale 1ns/1ns
module mdio_tb;
    reg         clk;
    reg         rst_n;
    wire        eth_rst_n;
    wire        eth_mdc;
    wire        eth_mdio;
    wire [15:0] rddata;
    wire        init_done;
    
    pullup PUP(eth_mdio);  //模拟外部上拉电阻
    
    phy_reg_config 
    #(
        .speed(2'b01)
    )
    u_phy_config(
        .clk          (clk       ),
        .rst_n        (rst_n     ),
        .phy_rst_n    (eth_rst_n ),
        .mdc          (eth_mdc   ),
        .mdio         (eth_mdio  ),
        .rddata       (rddata    ),
        .phy_init_done(init_done )
    );
    
    always #10 clk = ~clk;
    initial begin
        clk = 1;
        rst_n = 0;
        #201;
        rst_n = 1;
        
        @(posedge init_done);
        #2000;
        $stop;
    end
endmodule

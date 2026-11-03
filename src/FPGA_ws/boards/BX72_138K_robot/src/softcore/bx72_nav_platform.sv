module bx72_nav_rom (
    input wire clk,
    input wire i_en_i, input wire [12:0] i_addr_i, output wire [63:0] i_data_o,
    input wire d_en_i, input wire [12:0] d_addr_i, output wire [63:0] d_data_o
);
    // 旧本地规划ROM缺少此ABI，必须先用当前固件生成器重新生成。
    generate if (bx72_nav_boot_pkg::ROM_ABI_VERSION != 2) begin : g_wrong_firmware_abi
        BX72_REGENERATE_FIRMWARE_ROM required();
    end endgenerate
    localparam integer ROM_WORDS = bx72_nav_boot_pkg::ROM_WORDS;
    localparam integer ROM_DWORDS = (ROM_WORDS + 1) / 2;

    // 一个bank覆盖8KiB镜像；实际数组深度固定为1024个64位字。
    // ROM_BANKS直接由镜像字数推导，因此48KiB镜像只实例化六个bank。
    localparam integer ROM_BANK_DWORDS = 1024;
    localparam integer ROM_BANKS = (ROM_DWORDS + ROM_BANK_DWORDS - 1) / ROM_BANK_DWORDS;

    wire [63:0] i_bank_data [0:ROM_BANKS-1];
    wire [63:0] d_bank_data [0:ROM_BANKS-1];
    reg [2:0] i_bank_sel_q, d_bank_sel_q;
    reg i_data_valid_q, d_data_valid_q;

    initial begin
        i_bank_sel_q = 3'd0;
        d_bank_sel_q = 3'd0;
        i_data_valid_q = 1'b0;
        d_data_valid_q = 1'b0;
    end

    // 每个bank独立推断一块双读端口同步ROM。读数据寄存器与bank选择寄存器
    // 同沿更新，输出端组合选择已寄存的bank数据，保持原有一拍读延迟。
    genvar bank;
    generate
        for (bank = 0; bank < ROM_BANKS; bank = bank + 1) begin : g_rom_bank
            localparam integer BANK_BASE_DWORD = bank * ROM_BANK_DWORDS;
            localparam [2:0] BANK_SELECT = bank;
            (* syn_ramstyle="block_ram" *) reg [63:0] words [0:ROM_BANK_DWORDS-1];
            reg [63:0] i_data_q, d_data_q;
            integer word_offset;

            initial begin
                for (word_offset = 0; word_offset < ROM_BANK_DWORDS; word_offset = word_offset + 1) begin
                    if ((BANK_BASE_DWORD + word_offset) < ROM_DWORDS) begin
                        if (((BANK_BASE_DWORD + word_offset) * 2 + 1) < ROM_WORDS)
                            words[word_offset] = {
                                bx72_nav_boot_pkg::boot_word((BANK_BASE_DWORD + word_offset) * 2 + 1),
                                bx72_nav_boot_pkg::boot_word((BANK_BASE_DWORD + word_offset) * 2)
                            };
                        else
                            words[word_offset] = {
                                32'd0,
                                bx72_nav_boot_pkg::boot_word((BANK_BASE_DWORD + word_offset) * 2)
                            };
                    end else begin
                        words[word_offset] = 64'd0;
                    end
                end
            end

            always @(posedge clk) begin
                if (i_en_i && (i_addr_i < ROM_DWORDS) &&
                    (i_addr_i[12:10] == BANK_SELECT))
                    i_data_q <= words[i_addr_i[9:0]];
                if (d_en_i && (d_addr_i < ROM_DWORDS) &&
                    (d_addr_i[12:10] == BANK_SELECT))
                    d_data_q <= words[d_addr_i[9:0]];
            end

            assign i_bank_data[bank] = i_data_q;
            assign d_bank_data[bank] = d_data_q;
        end
    endgenerate

    always @(posedge clk) begin
        // 有效位屏蔽未初始化的bank输出；关闭读使能时bank选择和有效位均保持。
        // 越过ROM_WORDS的请求在对应沿将有效位置零，输出为0。
        if (i_en_i) begin
            i_bank_sel_q <= i_addr_i[12:10];
            i_data_valid_q <= (i_addr_i < ROM_DWORDS);
        end
        if (d_en_i) begin
            d_bank_sel_q <= d_addr_i[12:10];
            d_data_valid_q <= (d_addr_i < ROM_DWORDS);
        end
    end

    assign i_data_o = i_data_valid_q ? i_bank_data[i_bank_sel_q] : 64'd0;
    assign d_data_o = d_data_valid_q ? d_bank_data[d_bank_sel_q] : 64'd0;
endmodule
module bx72_nav_axi (
    input  wire        clk,
    input  wire        rst,

    // I AXI：CPU输出方向反转后由本模块接收AR并返回R。
    input  wire        iBusAxi_arvalid,
    output reg         iBusAxi_arready,
    input  wire [31:0] iBusAxi_araddr,
    input  wire [0:0]  iBusAxi_arid,
    input  wire [3:0]  iBusAxi_arregion,
    input  wire [7:0]  iBusAxi_arlen,
    input  wire [2:0]  iBusAxi_arsize,
    input  wire [1:0]  iBusAxi_arburst,
    input  wire [0:0]  iBusAxi_arlock,
    input  wire [3:0]  iBusAxi_arcache,
    input  wire [3:0]  iBusAxi_arqos,
    input  wire [2:0]  iBusAxi_arprot,
    output reg         iBusAxi_rvalid,
    input  wire        iBusAxi_rready,
    output reg  [31:0] iBusAxi_rdata,
    output reg  [0:0]  iBusAxi_rid,
    output reg  [1:0]  iBusAxi_rresp,
    output reg         iBusAxi_rlast,

    // D AXI：64位数据总线，支持独立AW/W握手及一笔读或写事务。
    input  wire        dBusAxi_awvalid,
    output reg         dBusAxi_awready,
    input  wire [31:0] dBusAxi_awaddr,
    input  wire [0:0]  dBusAxi_awid,
    input  wire [3:0]  dBusAxi_awregion,
    input  wire [7:0]  dBusAxi_awlen,
    input  wire [2:0]  dBusAxi_awsize,
    input  wire [1:0]  dBusAxi_awburst,
    input  wire [0:0]  dBusAxi_awlock,
    input  wire [3:0]  dBusAxi_awcache,
    input  wire [3:0]  dBusAxi_awqos,
    input  wire [2:0]  dBusAxi_awprot,
    input  wire        dBusAxi_wvalid,
    output reg         dBusAxi_wready,
    input  wire [63:0] dBusAxi_wdata,
    input  wire [7:0]  dBusAxi_wstrb,
    input  wire        dBusAxi_wlast,
    output reg         dBusAxi_bvalid,
    input  wire        dBusAxi_bready,
    output reg  [0:0]  dBusAxi_bid,
    output reg  [1:0]  dBusAxi_bresp,
    input  wire        dBusAxi_arvalid,
    output reg         dBusAxi_arready,
    input  wire [31:0] dBusAxi_araddr,
    input  wire [0:0]  dBusAxi_arid,
    input  wire [3:0]  dBusAxi_arregion,
    input  wire [7:0]  dBusAxi_arlen,
    input  wire [2:0]  dBusAxi_arsize,
    input  wire [1:0]  dBusAxi_arburst,
    input  wire [0:0]  dBusAxi_arlock,
    input  wire [3:0]  dBusAxi_arcache,
    input  wire [3:0]  dBusAxi_arqos,
    input  wire [2:0]  dBusAxi_arprot,
    output reg         dBusAxi_rvalid,
    input  wire        dBusAxi_rready,
    output reg  [63:0] dBusAxi_rdata,
    output reg  [0:0]  dBusAxi_rid,
    output reg  [1:0]  dBusAxi_rresp,
    output reg         dBusAxi_rlast,

    // 双读ROM：地址/en沿采样，数据沿后NBA更新；I和D互不争用。
    output reg         i_rom_en_o,
    output reg  [12:0] i_rom_addr_o,
    input  wire [63:0] i_rom_data_i,
    output reg         d_rom_en_o,
    output reg  [12:0] d_rom_addr_o,
    input  wire [63:0] d_rom_data_i,

    // D侧32位单拍MMIO通道。地址保持原byte offset，bit2选择64位AXI lane。
    output reg         mmio_valid_o,
    output reg         mmio_write_o,
    output reg  [11:0] mmio_addr_o,
    output reg  [31:0] mmio_wdata_o,
    output reg  [3:0]  mmio_wstrb_o,
    input  wire        mmio_ready_i,
    input  wire        mmio_rsp_valid_i,
    output reg         mmio_rsp_ready_o,
    input  wire [31:0] mmio_rdata_i,
    input  wire [1:0]  mmio_resp_i
);
    localparam [31:0] ROM_BASE   = 32'h80000000;
    localparam [31:0] ROM_LIMIT  = 32'h80000000 + bx72_nav_boot_pkg::ROM_BYTES;
    localparam [31:0] RAM_BASE   = 32'h40000000;
    localparam [31:0] RAM_LIMIT  = 32'h40008000;
    localparam [31:0] MMIO_BASE  = 32'hf0000000;
    localparam [31:0] MMIO_LIMIT = 32'hf0001000;

    localparam [1:0] T_NONE = 2'd0, T_ROM = 2'd1, T_RAM = 2'd2, T_MMIO = 2'd3;
    localparam [1:0] RESP_OKAY = 2'b00, RESP_SLVERR = 2'b10, RESP_DECERR = 2'b11;

    localparam [1:0] I_IDLE = 2'd0, I_FETCH = 2'd1, I_RESP = 2'd2, I_ERR = 2'd3;
    reg [1:0] i_state;
    reg [31:0] i_addr_q, i_start_addr_q;
    reg [7:0] i_len_q, i_index_q;
    reg [2:0] i_size_q;
    reg [1:0] i_burst_q, i_resp_q;
    reg [0:0] i_id_q;

    localparam [3:0] D_IDLE = 4'd0,
                     D_R_FETCH = 4'd1,
                     D_R_RESP = 4'd2,
                     D_R_ERR = 4'd3,
                     D_R_MMIO_REQ = 4'd4,
                     D_R_MMIO_WAIT = 4'd5,
                     D_W_COLLECT = 4'd6,
                     D_W_COMMIT_STAGE = 4'd7,
                     D_W_COMMIT_RAM_READ = 4'd8,
                     D_W_COMMIT_RAM_WRITE = 4'd9,
                     D_W_MMIO_REQ = 4'd10,
                     D_W_MMIO_WAIT = 4'd11,
                     D_B_RESP = 4'd12;
    reg [3:0] d_state;
    reg prefer_write;

    reg [31:0] d_read_addr_q, d_read_start_q;
    reg [7:0] d_read_len_q, d_read_index_q;
    reg [2:0] d_read_size_q;
    reg [1:0] d_read_burst_q, d_read_resp_q;
    reg [1:0] d_read_target_q;
    reg [0:0] d_read_id_q;
    reg [63:0] d_mmio_read_data_q;

    reg [31:0] d_write_start_q, d_write_addr_q, d_commit_addr_q;
    reg [7:0] d_write_len_q, d_write_index_q, d_commit_index_q;
    reg [2:0] d_write_size_q;
    reg [1:0] d_write_burst_q, d_write_resp_q, d_write_target_q;
    reg [0:0] d_write_id_q;
    reg d_write_req_ok, d_write_bad;
    reg [31:0] d_mmio_write_data_q;
    reg [7:0] d_mmio_write_strobe_q;
    reg [11:0] d_mmio_addr_q;

    // 读写RAM与暂存RAM都采用真正同步端口。写burst暂存完并通过检查前，
    // 目标RAM不写任何数据，故非法WSTRB/WLAST不会留下半笔可见写入。
    reg ram_a_en, ram_a_we;
    reg [11:0] ram_a_addr;
    reg [63:0] ram_a_wdata;
    wire [63:0] ram_a_rdata;
    bx72_nav_ram #(.DATA_W(64), .ADDR_W(12)) d_data_ram (
        .clk(clk), .a_en(ram_a_en), .a_we(ram_a_we),
        .a_addr(ram_a_addr), .a_wdata(ram_a_wdata), .a_rdata(ram_a_rdata),
        .b_en(1'b0), .b_addr(12'd0), .b_rdata()
    );

    reg stage_a_en;
    reg [7:0] stage_a_addr;
    reg [71:0] stage_a_wdata;
    reg stage_b_en;
    reg [7:0] stage_b_addr;
    wire [71:0] stage_b_rdata;
    bx72_nav_ram #(.DATA_W(72), .ADDR_W(8)) d_write_stage (
        .clk(clk), .a_en(stage_a_en), .a_we(1'b1),
        .a_addr(stage_a_addr), .a_wdata(stage_a_wdata), .a_rdata(),
        .b_en(stage_b_en), .b_addr(stage_b_addr), .b_rdata(stage_b_rdata)
    );

    function automatic [1:0] region_of;
        input [31:0] addr;
        begin
            if ((addr >= ROM_BASE) && (addr < ROM_LIMIT)) region_of = T_ROM;
            else if ((addr >= RAM_BASE) && (addr < RAM_LIMIT)) region_of = T_RAM;
            else if ((addr >= MMIO_BASE) && (addr < MMIO_LIMIT)) region_of = T_MMIO;
            else region_of = T_NONE;
        end
    endfunction

    // 只接受AXI允许的FIXED/INCR及2/4/8/16拍WRAP；长度、SIZE、对齐和4KiB
    // 约束都使用完整地址验证。此检查必须发生在后续[11:3]RAM/ROM地址切片前。
    function automatic burst_shape_ok;
        input [31:0] addr;
        input [7:0] len;
        input [2:0] size;
        input [1:0] burst;
        input [2:0] max_size;
        reg [32:0] beat_bytes, beats, span, end_exclusive, last_byte, wrap_base;
        begin
            burst_shape_ok = 1'b0;
            if ((size <= max_size) && (burst != 2'b11)) begin
                beat_bytes = 33'd1 << size;
                beats = {25'd0,len} + 33'd1;
                if (({1'b0,addr} & (beat_bytes - 33'd1)) == 0) begin
                    if (burst == 2'b10) begin
                        if ((len == 8'd1) || (len == 8'd3) ||
                            (len == 8'd7) || (len == 8'd15)) begin
                            span = beats << size;
                            wrap_base = {1'b0,addr} & ~(span - 33'd1);
                            end_exclusive = wrap_base + span;
                            last_byte = end_exclusive - 33'd1;
                            if (!end_exclusive[32] &&
                                (addr[31:12] == last_byte[31:12]))
                                burst_shape_ok = 1'b1;
                        end
                    end else if ((burst == 2'b00) && (len > 8'd15)) begin
                        // AXI4 FIXED最多16拍；只有INCR扩展到256拍。
                        burst_shape_ok = 1'b0;
                    end else begin
                        if (burst == 2'b00) end_exclusive = {1'b0,addr} + beat_bytes;
                        else end_exclusive = {1'b0,addr} + (beats << size);
                        last_byte = end_exclusive - 33'd1;
                        if (!end_exclusive[32] &&
                            (addr[31:12] == last_byte[31:12]))
                            burst_shape_ok = 1'b1;
                    end
                end
            end
        end
    endfunction

    function automatic burst_range_ok;
        input [31:0] addr;
        input [7:0] len;
        input [2:0] size;
        input [1:0] burst;
        input [31:0] region_base;
        input [31:0] region_limit;
        reg [32:0] beat_bytes, beats, span, first_byte, end_exclusive, wrap_base;
        begin
            beat_bytes = 33'd1 << size;
            beats = {25'd0,len} + 33'd1;
            first_byte = {1'b0,addr};
            if (burst == 2'b00) end_exclusive = {1'b0,addr} + beat_bytes;
            else if (burst == 2'b10) begin
                span = beats << size;
                wrap_base = {1'b0,addr} & ~(span - 33'd1);
                first_byte = wrap_base;
                end_exclusive = wrap_base + span;
            end else end_exclusive = {1'b0,addr} + (beats << size);
            burst_range_ok = !end_exclusive[32] &&
                (first_byte >= {1'b0,region_base}) &&
                (end_exclusive <= {1'b0,region_limit});
        end
    endfunction

    function automatic [31:0] next_burst_addr;
        input [31:0] current_addr;
        input [31:0] start_addr;
        input [7:0] len;
        input [2:0] size;
        input [1:0] burst;
        reg [32:0] beat_bytes, beats, span, wrap_base, wrap_limit, next_full;
        begin
            beat_bytes = 33'd1 << size;
            beats = {25'd0,len} + 33'd1;
            next_full = {1'b0,current_addr};
            if (burst == 2'b01) next_full = {1'b0,current_addr} + beat_bytes;
            else if (burst == 2'b10) begin
                span = beats << size;
                wrap_base = {1'b0,start_addr} & ~(span - 33'd1);
                wrap_limit = wrap_base + span;
                next_full = {1'b0,current_addr} + beat_bytes;
                if (next_full >= wrap_limit) next_full = wrap_base;
            end
            next_burst_addr = next_full[31:0];
        end
    endfunction

    function automatic [7:0] transfer_lane_mask;
        input [2:0] size;
        input [2:0] lane;
        begin
            case (size)
                3'd0: transfer_lane_mask = 8'b00000001 << lane;
                3'd1: transfer_lane_mask = 8'b00000011 << lane;
                3'd2: transfer_lane_mask = 8'b00001111 << lane;
                3'd3: transfer_lane_mask = 8'hff << lane;
                default: transfer_lane_mask = 8'd0;
            endcase
        end
    endfunction

    function automatic [63:0] merge_write_lanes;
        input [63:0] old_word;
        input [63:0] new_word;
        input [7:0] strobe;
        integer lane;
        begin
            for (lane = 0; lane < 8; lane = lane + 1)
                merge_write_lanes[lane*8 +: 8] = strobe[lane] ?
                    new_word[lane*8 +: 8] : old_word[lane*8 +: 8];
        end
    endfunction

    wire [1:0] i_req_region = region_of(iBusAxi_araddr);
    wire [1:0] d_ar_region = region_of(dBusAxi_araddr);
    wire [1:0] d_aw_region = region_of(dBusAxi_awaddr);
    wire i_req_shape_ok = burst_shape_ok(iBusAxi_araddr, iBusAxi_arlen,
                                         iBusAxi_arsize, iBusAxi_arburst, 3'd2);
    // GoRV32的非缓存RV32访问仍使用64位native AXI拍宽，ARADDR/AWADDR
    // 保留所选32位字地址。MMIO ABI是32位寄存器；返回/写入所选半拍，
    // 不把native拍宽误判为未对齐访问，也不接受双字寄存器写入。
    wire d_ar_native_mmio = (d_ar_region == T_MMIO) &&
        (dBusAxi_arlen == 0) && (dBusAxi_arsize == 3'd3) &&
        (dBusAxi_arburst == 2'b01) && (dBusAxi_araddr[1:0] == 0);
    wire d_aw_native_mmio = (d_aw_region == T_MMIO) &&
        (dBusAxi_awlen == 0) && (dBusAxi_awsize == 3'd3) &&
        (dBusAxi_awburst == 2'b01) && (dBusAxi_awaddr[1:0] == 0);
    // AXI INCR首拍允许非自然对齐。GoRV32对RAM字节/半字/字访问也保留
    // 原始地址并声明native64拍宽；存储体按8字节字寻址，WSTRB保护首地址前字节。
    wire d_ar_native_mem = ((d_ar_region == T_RAM) || (d_ar_region == T_ROM)) &&
        (dBusAxi_arlen == 0) && (dBusAxi_arsize == 3'd3) && (dBusAxi_arburst == 2'b01);
    wire d_aw_native_mem = ((d_aw_region == T_RAM) || (d_aw_region == T_ROM)) &&
        (dBusAxi_awlen == 0) && (dBusAxi_awsize == 3'd3) && (dBusAxi_awburst == 2'b01);
    wire [31:0] d_ar_range_addr = d_ar_native_mem ? {dBusAxi_araddr[31:3],3'd0} : dBusAxi_araddr;
    wire [31:0] d_aw_range_addr = d_aw_native_mem ? {dBusAxi_awaddr[31:3],3'd0} : dBusAxi_awaddr;
    wire d_ar_shape_ok = d_ar_native_mmio || d_ar_native_mem || burst_shape_ok(dBusAxi_araddr, dBusAxi_arlen,
                                        dBusAxi_arsize, dBusAxi_arburst, 3'd3);
    wire d_aw_shape_ok = d_aw_native_mmio || d_aw_native_mem || burst_shape_ok(dBusAxi_awaddr, dBusAxi_awlen,
                                        dBusAxi_awsize, dBusAxi_awburst, 3'd3);
    wire d_w_lane_ok = (d_write_target_q == T_MMIO) ?
        (dBusAxi_wstrb == (d_write_addr_q[2] ? 8'hf0 : 8'h0f)) :
        ((dBusAxi_wstrb & ~transfer_lane_mask(d_write_size_q,
                                              d_write_addr_q[2:0])) == 8'd0);
    wire [63:0] mmio_read_lane_data = d_read_addr_q[2] ?
        {mmio_rdata_i,32'd0} : {32'd0,mmio_rdata_i};

    // AXI sideband字段不改变本地RAM行为；lock被明确拒绝，region/cache/qos/prot忽略。
    wire unused_axi_sidebands = ^{iBusAxi_arregion, iBusAxi_arcache,
        iBusAxi_arqos, iBusAxi_arprot, dBusAxi_awregion, dBusAxi_awcache,
        dBusAxi_awqos, dBusAxi_awprot, dBusAxi_arregion, dBusAxi_arcache,
        dBusAxi_arqos, dBusAxi_arprot};

    // 同步存储端口请求、MMIO请求均只由已锁存的FSM状态产生。
    always @* begin
        i_rom_en_o = 1'b0;
        i_rom_addr_o = 13'd0;
        d_rom_en_o = 1'b0;
        d_rom_addr_o = 13'd0;
        ram_a_en = 1'b0;
        ram_a_we = 1'b0;
        ram_a_addr = 12'd0;
        ram_a_wdata = 64'd0;
        stage_a_en = 1'b0;
        stage_a_addr = 8'd0;
        stage_a_wdata = 72'd0;
        stage_b_en = 1'b0;
        stage_b_addr = 8'd0;
        mmio_valid_o = 1'b0;
        mmio_write_o = 1'b0;
        mmio_addr_o = 12'd0;
        mmio_wdata_o = 32'd0;
        mmio_wstrb_o = 4'd0;
        mmio_rsp_ready_o = 1'b0;

        if (!rst) begin
            if (i_state == I_FETCH) begin
                i_rom_en_o = 1'b1;
                i_rom_addr_o = i_addr_q[15:3];
            end
            if (d_state == D_R_FETCH) begin
                if (d_read_target_q == T_ROM) begin
                    d_rom_en_o = 1'b1;
                    d_rom_addr_o = d_read_addr_q[15:3];
                end else if (d_read_target_q == T_RAM) begin
                    ram_a_en = 1'b1;
                    ram_a_addr = d_read_addr_q[14:3];
                end
            end
            if ((d_state == D_W_COLLECT) && dBusAxi_wvalid && dBusAxi_wready &&
                d_write_req_ok && !d_write_bad && (d_write_target_q == T_RAM) &&
                d_w_lane_ok) begin
                stage_a_en = 1'b1;
                stage_a_addr = d_write_index_q;
                stage_a_wdata = {dBusAxi_wstrb,dBusAxi_wdata};
            end
            if (d_state == D_W_COMMIT_STAGE) begin
                stage_b_en = 1'b1;
                stage_b_addr = d_commit_index_q;
            end
            if (d_state == D_W_COMMIT_RAM_READ) begin
                ram_a_en = 1'b1;
                ram_a_addr = d_commit_addr_q[14:3];
            end
            if (d_state == D_W_COMMIT_RAM_WRITE) begin
                ram_a_en = 1'b1;
                ram_a_we = 1'b1;
                ram_a_addr = d_commit_addr_q[14:3];
                ram_a_wdata = merge_write_lanes(ram_a_rdata,
                                                 stage_b_rdata[63:0],
                                                 stage_b_rdata[71:64]);
            end
            if (d_state == D_R_MMIO_REQ) begin
                mmio_valid_o = 1'b1;
                mmio_addr_o = d_read_addr_q[11:0];
            end
            if (d_state == D_W_MMIO_REQ) begin
                mmio_valid_o = 1'b1;
                mmio_write_o = 1'b1;
                mmio_addr_o = d_mmio_addr_q;
                mmio_wdata_o = d_mmio_write_data_q;
                mmio_wstrb_o = d_write_start_q[2] ?
                    d_mmio_write_strobe_q[7:4] : d_mmio_write_strobe_q[3:0];
            end
            if ((d_state == D_R_MMIO_WAIT) || (d_state == D_W_MMIO_WAIT))
                mmio_rsp_ready_o = 1'b1;
        end
    end

    // AXI channel outputs. R/B payloads remain registered or RAM-backed and stable
    // until the corresponding ready handshake; invalid reads return every LEN beat.
    always @* begin
        iBusAxi_arready = !rst && (i_state == I_IDLE);
        iBusAxi_rvalid = (i_state == I_RESP) || (i_state == I_ERR);
        iBusAxi_rdata = 32'd0;
        iBusAxi_rid = i_id_q;
        iBusAxi_rresp = i_resp_q;
        iBusAxi_rlast = (i_index_q == i_len_q);
        if (i_state == I_RESP)
            iBusAxi_rdata = i_addr_q[2] ? i_rom_data_i[63:32] : i_rom_data_i[31:0];

        dBusAxi_awready = 1'b0;
        dBusAxi_arready = 1'b0;
        dBusAxi_wready = !rst && (d_state == D_W_COLLECT);
        dBusAxi_bvalid = (d_state == D_B_RESP);
        dBusAxi_bid = d_write_id_q;
        dBusAxi_bresp = d_write_resp_q;
        dBusAxi_rvalid = (d_state == D_R_RESP) || (d_state == D_R_ERR);
        dBusAxi_rdata = 64'd0;
        dBusAxi_rid = d_read_id_q;
        dBusAxi_rresp = d_read_resp_q;
        dBusAxi_rlast = (d_read_index_q == d_read_len_q);

        if (!rst && (d_state == D_IDLE)) begin
            if (dBusAxi_awvalid && dBusAxi_arvalid) begin
                dBusAxi_awready = prefer_write;
                dBusAxi_arready = !prefer_write;
            end else if (dBusAxi_awvalid) dBusAxi_awready = 1'b1;
            else if (dBusAxi_arvalid) dBusAxi_arready = 1'b1;
        end

        if (d_state == D_R_RESP) begin
            if (d_read_target_q == T_RAM) dBusAxi_rdata = ram_a_rdata;
            else if (d_read_target_q == T_ROM) dBusAxi_rdata = d_rom_data_i;
            else if (d_read_target_q == T_MMIO) dBusAxi_rdata = d_mmio_read_data_q;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            i_state <= I_IDLE;
            i_addr_q <= 32'd0;
            i_start_addr_q <= 32'd0;
            i_len_q <= 8'd0;
            i_index_q <= 8'd0;
            i_size_q <= 3'd0;
            i_burst_q <= 2'd0;
            i_resp_q <= RESP_OKAY;
            i_id_q <= 1'b0;

            d_state <= D_IDLE;
            prefer_write <= 1'b1;
            d_read_addr_q <= 32'd0;
            d_read_start_q <= 32'd0;
            d_read_len_q <= 8'd0;
            d_read_index_q <= 8'd0;
            d_read_size_q <= 3'd0;
            d_read_burst_q <= 2'd0;
            d_read_resp_q <= RESP_OKAY;
            d_read_target_q <= T_NONE;
            d_read_id_q <= 1'b0;
            d_mmio_read_data_q <= 64'd0;
            d_write_start_q <= 32'd0;
            d_write_addr_q <= 32'd0;
            d_commit_addr_q <= 32'd0;
            d_write_len_q <= 8'd0;
            d_write_index_q <= 8'd0;
            d_commit_index_q <= 8'd0;
            d_write_size_q <= 3'd0;
            d_write_burst_q <= 2'd0;
            d_write_resp_q <= RESP_OKAY;
            d_write_target_q <= T_NONE;
            d_write_id_q <= 1'b0;
            d_write_req_ok <= 1'b0;
            d_write_bad <= 1'b0;
            d_mmio_write_data_q <= 32'd0;
            d_mmio_write_strobe_q <= 8'd0;
            d_mmio_addr_q <= 12'd0;
        end else begin
            if (i_state == I_IDLE) begin
                if (iBusAxi_arvalid && iBusAxi_arready) begin
                    i_addr_q <= iBusAxi_araddr;
                    i_start_addr_q <= iBusAxi_araddr;
                    i_len_q <= iBusAxi_arlen;
                    i_index_q <= 8'd0;
                    i_size_q <= iBusAxi_arsize;
                    i_burst_q <= iBusAxi_arburst;
                    i_id_q <= iBusAxi_arid;
                    if (iBusAxi_arlock != 1'b0) begin
                        i_resp_q <= RESP_SLVERR;
                        i_state <= I_ERR;
                    end else if (i_req_region == T_NONE) begin
                        i_resp_q <= RESP_DECERR;
                        i_state <= I_ERR;
                    end else if ((i_req_region != T_ROM) ||
                                 (iBusAxi_arsize != 3'd2) || !i_req_shape_ok) begin
                        i_resp_q <= RESP_SLVERR;
                        i_state <= I_ERR;
                    end else if (!burst_range_ok(iBusAxi_araddr,iBusAxi_arlen,
                                                 iBusAxi_arsize,iBusAxi_arburst,
                                                 ROM_BASE,ROM_LIMIT)) begin
                        i_resp_q <= RESP_DECERR;
                        i_state <= I_ERR;
                    end else begin
                        i_resp_q <= RESP_OKAY;
                        i_state <= I_FETCH;
                    end
                end
            end else if (i_state == I_FETCH) begin
                // ROM同步读在本沿NBA更新，下一状态看到的数据属于本拍地址。
                i_state <= I_RESP;
            end else if ((i_state == I_RESP) || (i_state == I_ERR)) begin
                if (iBusAxi_rvalid && iBusAxi_rready) begin
                    if (i_index_q == i_len_q) i_state <= I_IDLE;
                    else begin
                        i_index_q <= i_index_q + 1'b1;
                        i_addr_q <= next_burst_addr(i_addr_q,i_start_addr_q,
                            i_len_q,i_size_q,i_burst_q);
                        if (i_state == I_RESP) i_state <= I_FETCH;
                    end
                end
            end

            if (d_state == D_IDLE) begin
                if (dBusAxi_awvalid && dBusAxi_awready) begin
                    prefer_write <= 1'b0;
                    d_write_start_q <= dBusAxi_awaddr;
                    d_write_addr_q <= dBusAxi_awaddr;
                    d_write_len_q <= dBusAxi_awlen;
                    d_write_index_q <= 8'd0;
                    d_write_size_q <= dBusAxi_awsize;
                    d_write_burst_q <= dBusAxi_awburst;
                    d_write_id_q <= dBusAxi_awid;
                    d_write_target_q <= d_aw_region;
                    d_write_bad <= 1'b0;
                    if (dBusAxi_awlock != 1'b0) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_SLVERR;
                    end else if (d_aw_region == T_NONE) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_DECERR;
                    end else if (!d_aw_shape_ok) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_SLVERR;
                    end else if ((d_aw_region == T_MMIO) &&
                                 ((dBusAxi_awlen != 8'd0) ||
                                  ((dBusAxi_awsize != 3'd2) && !d_aw_native_mmio) ||
                                  (dBusAxi_awburst == 2'b10))) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_SLVERR;
                    end else if (!burst_range_ok(d_aw_range_addr,dBusAxi_awlen,
                                                  d_aw_native_mmio ? 3'd2 : dBusAxi_awsize,dBusAxi_awburst,
                                                  (d_aw_region == T_ROM) ? ROM_BASE :
                                                  ((d_aw_region == T_RAM) ? RAM_BASE : MMIO_BASE),
                                                  (d_aw_region == T_ROM) ? ROM_LIMIT :
                                                  ((d_aw_region == T_RAM) ? RAM_LIMIT : MMIO_LIMIT))) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_DECERR;
                    end else if (d_aw_region == T_ROM) begin
                        d_write_req_ok <= 1'b0;
                        d_write_resp_q <= RESP_SLVERR;
                    end else begin
                        d_write_req_ok <= 1'b1;
                        d_write_resp_q <= RESP_OKAY;
                    end
                    d_state <= D_W_COLLECT;
                end else if (dBusAxi_arvalid && dBusAxi_arready) begin
                    prefer_write <= 1'b1;
                    d_read_addr_q <= dBusAxi_araddr;
                    d_read_start_q <= dBusAxi_araddr;
                    d_read_len_q <= dBusAxi_arlen;
                    d_read_index_q <= 8'd0;
                    d_read_size_q <= dBusAxi_arsize;
                    d_read_burst_q <= dBusAxi_arburst;
                    d_read_id_q <= dBusAxi_arid;
                    d_read_target_q <= d_ar_region;
                    if (dBusAxi_arlock != 1'b0) begin
                        d_read_resp_q <= RESP_SLVERR;
                        d_state <= D_R_ERR;
                    end else if (d_ar_region == T_NONE) begin
                        d_read_resp_q <= RESP_DECERR;
                        d_state <= D_R_ERR;
                    end else if (!d_ar_shape_ok) begin
                        d_read_resp_q <= RESP_SLVERR;
                        d_state <= D_R_ERR;
                    end else if ((d_ar_region == T_MMIO) &&
                                 ((dBusAxi_arlen != 8'd0) ||
                                  ((dBusAxi_arsize != 3'd2) && !d_ar_native_mmio) ||
                                  (dBusAxi_arburst == 2'b10))) begin
                        d_read_resp_q <= RESP_SLVERR;
                        d_state <= D_R_ERR;
                    end else if (!burst_range_ok(d_ar_range_addr,dBusAxi_arlen,
                                                  d_ar_native_mmio ? 3'd2 : dBusAxi_arsize,dBusAxi_arburst,
                                                  (d_ar_region == T_ROM) ? ROM_BASE :
                                                  ((d_ar_region == T_RAM) ? RAM_BASE : MMIO_BASE),
                                                  (d_ar_region == T_ROM) ? ROM_LIMIT :
                                                  ((d_ar_region == T_RAM) ? RAM_LIMIT : MMIO_LIMIT))) begin
                        d_read_resp_q <= RESP_DECERR;
                        d_state <= D_R_ERR;
                    end else begin
                        d_read_resp_q <= RESP_OKAY;
                        if (d_ar_region == T_MMIO) d_state <= D_R_MMIO_REQ;
                        else d_state <= D_R_FETCH;
                    end
                end
            end else if (d_state == D_R_FETCH) begin
                // d_rom/data_ram请求在本沿采样，数据NBA更新后进入响应态。
                d_state <= D_R_RESP;
            end else if (d_state == D_R_RESP) begin
                if (dBusAxi_rvalid && dBusAxi_rready) begin
                    if (d_read_index_q == d_read_len_q) d_state <= D_IDLE;
                    else begin
                        d_read_index_q <= d_read_index_q + 1'b1;
                        d_read_addr_q <= next_burst_addr(d_read_addr_q,
                            d_read_start_q,d_read_len_q,d_read_size_q,d_read_burst_q);
                        d_state <= D_R_FETCH;
                    end
                end
            end else if (d_state == D_R_ERR) begin
                if (dBusAxi_rvalid && dBusAxi_rready) begin
                    if (d_read_index_q == d_read_len_q) d_state <= D_IDLE;
                    else d_read_index_q <= d_read_index_q + 1'b1;
                end
            end else if (d_state == D_R_MMIO_REQ) begin
                if (mmio_valid_o && mmio_ready_i) d_state <= D_R_MMIO_WAIT;
            end else if (d_state == D_R_MMIO_WAIT) begin
                if (mmio_rsp_valid_i && mmio_rsp_ready_o) begin
                    d_mmio_read_data_q <= mmio_read_lane_data;
                    d_read_resp_q <= mmio_resp_i;
                    d_state <= D_R_RESP;
                end
            end else if (d_state == D_W_COLLECT) begin
                if (dBusAxi_wvalid && dBusAxi_wready) begin
                    if (d_write_req_ok && !d_w_lane_ok) d_write_bad <= 1'b1;
                    if (dBusAxi_wlast && (d_write_index_q != d_write_len_q)) begin
                        if (d_write_req_ok) d_write_resp_q <= RESP_SLVERR;
                        d_state <= D_B_RESP;
                    end else if (d_write_index_q == d_write_len_q) begin
                        if (!d_write_req_ok) d_state <= D_B_RESP;
                        else if (d_write_bad || !d_w_lane_ok || !dBusAxi_wlast) begin
                            d_write_resp_q <= RESP_SLVERR;
                            d_state <= D_B_RESP;
                        end else if (d_write_target_q == T_RAM) begin
                            d_commit_index_q <= 8'd0;
                            d_commit_addr_q <= d_write_start_q;
                            d_state <= D_W_COMMIT_STAGE;
                        end else if (d_write_target_q == T_MMIO) begin
                            d_mmio_addr_q <= d_write_start_q[11:0];
                            d_mmio_write_data_q <= d_write_start_q[2] ?
                                dBusAxi_wdata[63:32] : dBusAxi_wdata[31:0];
                            d_mmio_write_strobe_q <= dBusAxi_wstrb;
                            d_state <= D_W_MMIO_REQ;
                        end else begin
                            d_write_resp_q <= RESP_SLVERR;
                            d_state <= D_B_RESP;
                        end
                    end else begin
                        d_write_index_q <= d_write_index_q + 1'b1;
                        d_write_addr_q <= next_burst_addr(d_write_addr_q,
                            d_write_start_q,d_write_len_q,d_write_size_q,d_write_burst_q);
                    end
                end
            end else if (d_state == D_W_COMMIT_STAGE) begin
                d_state <= D_W_COMMIT_RAM_READ;
            end else if (d_state == D_W_COMMIT_RAM_READ) begin
                d_state <= D_W_COMMIT_RAM_WRITE;
            end else if (d_state == D_W_COMMIT_RAM_WRITE) begin
                if (d_commit_index_q == d_write_len_q) begin
                    d_write_resp_q <= RESP_OKAY;
                    d_state <= D_B_RESP;
                end else begin
                    d_commit_index_q <= d_commit_index_q + 1'b1;
                    d_commit_addr_q <= next_burst_addr(d_commit_addr_q,
                        d_write_start_q,d_write_len_q,d_write_size_q,d_write_burst_q);
                    d_state <= D_W_COMMIT_STAGE;
                end
            end else if (d_state == D_W_MMIO_REQ) begin
                if (mmio_valid_o && mmio_ready_i) d_state <= D_W_MMIO_WAIT;
            end else if (d_state == D_W_MMIO_WAIT) begin
                if (mmio_rsp_valid_i && mmio_rsp_ready_o) begin
                    d_write_resp_q <= mmio_resp_i;
                    d_state <= D_B_RESP;
                end
            end else if (d_state == D_B_RESP) begin
                if (dBusAxi_bvalid && dBusAxi_bready) d_state <= D_IDLE;
            end
        end
    end
endmodule
module bx72_nav_platform #(
    parameter integer CLK_HZ = 40000000
) (
    input wire clk,
    input wire rst,
    output reg cpu_ready_o,
    output reg cpu_fault_o,
    output reg [7:0] fault_code_o,
    output reg [31:0] heartbeat_o,
    output reg io_valid_o,
    output reg io_write_o,
    output reg [11:0] io_addr_o,
    output reg [31:0] io_wdata_o,
    output reg [3:0] io_wstrb_o,
    input wire io_ready_i,
    input wire [31:0] io_rdata_i
);
    wire  iBusAxi_arvalid;
    wire  iBusAxi_arready;
    wire [31:0] iBusAxi_araddr;
    wire [0:0] iBusAxi_arid;
    wire [3:0] iBusAxi_arregion;
    wire [7:0] iBusAxi_arlen;
    wire [2:0] iBusAxi_arsize;
    wire [1:0] iBusAxi_arburst;
    wire [0:0] iBusAxi_arlock;
    wire [3:0] iBusAxi_arcache;
    wire [3:0] iBusAxi_arqos;
    wire [2:0] iBusAxi_arprot;
    wire  iBusAxi_rvalid;
    wire  iBusAxi_rready;
    wire [31:0] iBusAxi_rdata;
    wire [0:0] iBusAxi_rid;
    wire [1:0] iBusAxi_rresp;
    wire  iBusAxi_rlast;
    wire  dBusAxi_awvalid;
    wire  dBusAxi_awready;
    wire [31:0] dBusAxi_awaddr;
    wire [0:0] dBusAxi_awid;
    wire [3:0] dBusAxi_awregion;
    wire [7:0] dBusAxi_awlen;
    wire [2:0] dBusAxi_awsize;
    wire [1:0] dBusAxi_awburst;
    wire [0:0] dBusAxi_awlock;
    wire [3:0] dBusAxi_awcache;
    wire [3:0] dBusAxi_awqos;
    wire [2:0] dBusAxi_awprot;
    wire  dBusAxi_wvalid;
    wire  dBusAxi_wready;
    wire [63:0] dBusAxi_wdata;
    wire [7:0] dBusAxi_wstrb;
    wire  dBusAxi_wlast;
    wire  dBusAxi_bvalid;
    wire  dBusAxi_bready;
    wire [0:0] dBusAxi_bid;
    wire [1:0] dBusAxi_bresp;
    wire  dBusAxi_arvalid;
    wire  dBusAxi_arready;
    wire [31:0] dBusAxi_araddr;
    wire [0:0] dBusAxi_arid;
    wire [3:0] dBusAxi_arregion;
    wire [7:0] dBusAxi_arlen;
    wire [2:0] dBusAxi_arsize;
    wire [1:0] dBusAxi_arburst;
    wire [0:0] dBusAxi_arlock;
    wire [3:0] dBusAxi_arcache;
    wire [3:0] dBusAxi_arqos;
    wire [2:0] dBusAxi_arprot;
    wire  dBusAxi_rvalid;
    wire  dBusAxi_rready;
    wire [63:0] dBusAxi_rdata;
    wire [0:0] dBusAxi_rid;
    wire [1:0] dBusAxi_rresp;
    wire  dBusAxi_rlast;
    wire [12:0] i_rom_addr, d_rom_addr;
    wire i_rom_en, d_rom_en;
    wire [63:0] i_rom_data, d_rom_data;
    wire mmio_valid, mmio_write, mmio_ready, mmio_rsp_valid, mmio_rsp_ready;
    wire [11:0] mmio_addr;
    wire [31:0] mmio_wdata, mmio_rdata;
    wire [3:0] mmio_wstrb;
    wire [1:0] mmio_resp;
    reg mmio_rsp_valid_q;
    reg [31:0] mmio_rdata_q;
    reg [1:0] mmio_resp_q;
    reg [31:0] boot_age_q, heartbeat_age_q;
    reg [31:0] trap_cause_q, trap_pc_q;
    reg [31:0] internal_read_data;
    reg [1:0] internal_response;
    wire mmio_is_external = mmio_addr >= 12'h100;
    wire mmio_fire = mmio_valid && mmio_ready;
    wire ready_write = mmio_fire && mmio_write && mmio_addr == 12'h008 &&
        mmio_wstrb == 4'hf && mmio_wdata == 32'd1 && !cpu_fault_o && !cpu_ready_o;
    wire heartbeat_write = mmio_fire && mmio_write && mmio_addr == 12'h00c &&
        mmio_wstrb == 4'hf && mmio_wdata != heartbeat_o && !cpu_fault_o;
    wire firmware_fault_write = mmio_fire && mmio_write && mmio_addr == 12'h010 &&
        mmio_wstrb == 4'hf && mmio_wdata != 0 && !cpu_fault_o;
    localparam integer BOOT_TIMEOUT_CYCLES = CLK_HZ * 2;
    localparam integer HEARTBEAT_TIMEOUT_CYCLES = CLK_HZ / 5;
    wire boot_timeout_due = !cpu_ready_o && !cpu_fault_o &&
        boot_age_q >= BOOT_TIMEOUT_CYCLES-1 && !ready_write;
    wire heartbeat_timeout_due = cpu_ready_o && !cpu_fault_o &&
        heartbeat_age_q >= HEARTBEAT_TIMEOUT_CYCLES-1 && !heartbeat_write;

    assign mmio_ready = mmio_is_external ? (!rst && io_ready_i) :
                        (!rst && !mmio_rsp_valid_q);
    assign mmio_rsp_valid = mmio_rsp_valid_q;
    assign mmio_rdata = mmio_rdata_q;
    assign mmio_resp = mmio_resp_q;

    always @* begin
        io_valid_o = 1'b0;
        io_write_o = mmio_write;
        io_addr_o = mmio_addr;
        io_wdata_o = mmio_wdata;
        io_wstrb_o = mmio_wstrb;
        if (!rst && mmio_valid && mmio_is_external) io_valid_o = 1'b1;

        internal_read_data = 32'd0;
        internal_response = 2'b00;
        if (mmio_write) begin
            case (mmio_addr)
                12'h008: if (mmio_wstrb != 4'hf || mmio_wdata != 32'd1 ||
                             cpu_fault_o || cpu_ready_o) internal_response = 2'b10;
                12'h00c: if (mmio_wstrb != 4'hf || cpu_fault_o) internal_response = 2'b10;
                12'h010: if (mmio_wstrb != 4'hf) internal_response = 2'b10;
                12'h014, 12'h018: if (mmio_wstrb != 4'hf) internal_response = 2'b10;
                default: begin end
            endcase
        end else begin
            case (mmio_addr)
                12'h000: internal_read_data = {30'd0, cpu_fault_o, cpu_ready_o};
                12'h004: internal_read_data = heartbeat_o;
                12'h008: internal_read_data = {31'd0, cpu_ready_o};
                12'h00c: internal_read_data = heartbeat_o;
                12'h010: internal_read_data = {24'd0, fault_code_o};
                12'h014: internal_read_data = trap_cause_q;
                12'h018: internal_read_data = trap_pc_q;
                12'h01c: internal_read_data = heartbeat_age_q;
                default: begin end
            endcase
        end
    end

    VexRiscvAxiPlicClint cpu (
        .clint_awvalid(1'd0),
        .clint_awready(),
        .clint_awaddr(16'd0),
        .clint_awprot(3'd0),
        .clint_wvalid(1'd0),
        .clint_wready(),
        .clint_wdata(32'd0),
        .clint_wstrb(4'd0),
        .clint_bvalid(),
        .clint_bready(1'b1),
        .clint_bresp(),
        .clint_arvalid(1'd0),
        .clint_arready(),
        .clint_araddr(16'd0),
        .clint_arprot(3'd0),
        .clint_rvalid(),
        .clint_rready(1'b1),
        .clint_rdata(),
        .clint_rresp(),
        .plic_awvalid(1'd0),
        .plic_awready(),
        .plic_awaddr(22'd0),
        .plic_awprot(3'd0),
        .plic_wvalid(1'd0),
        .plic_wready(),
        .plic_wdata(32'd0),
        .plic_wstrb(4'd0),
        .plic_bvalid(),
        .plic_bready(1'b1),
        .plic_bresp(),
        .plic_arvalid(1'd0),
        .plic_arready(),
        .plic_araddr(22'd0),
        .plic_arprot(3'd0),
        .plic_rvalid(),
        .plic_rready(1'b1),
        .plic_rdata(),
        .plic_rresp(),
        .plicInterrupts(32'd0),
        .jtag_tms(1'b1),
        .jtag_tdi(1'd0),
        .jtag_tdo(),
        .jtag_tck(1'd0),
        .ndmreset(),
        .reset(rst),
        .stoptime(),
        .iBusAxi_arvalid(iBusAxi_arvalid),
        .iBusAxi_arready(iBusAxi_arready),
        .iBusAxi_araddr(iBusAxi_araddr),
        .iBusAxi_arid(iBusAxi_arid),
        .iBusAxi_arregion(iBusAxi_arregion),
        .iBusAxi_arlen(iBusAxi_arlen),
        .iBusAxi_arsize(iBusAxi_arsize),
        .iBusAxi_arburst(iBusAxi_arburst),
        .iBusAxi_arlock(iBusAxi_arlock),
        .iBusAxi_arcache(iBusAxi_arcache),
        .iBusAxi_arqos(iBusAxi_arqos),
        .iBusAxi_arprot(iBusAxi_arprot),
        .iBusAxi_rvalid(iBusAxi_rvalid),
        .iBusAxi_rready(iBusAxi_rready),
        .iBusAxi_rdata(iBusAxi_rdata),
        .iBusAxi_rid(iBusAxi_rid),
        .iBusAxi_rresp(iBusAxi_rresp),
        .iBusAxi_rlast(iBusAxi_rlast),
        .dBusAxi_awvalid(dBusAxi_awvalid),
        .dBusAxi_awready(dBusAxi_awready),
        .dBusAxi_awaddr(dBusAxi_awaddr),
        .dBusAxi_awid(dBusAxi_awid),
        .dBusAxi_awregion(dBusAxi_awregion),
        .dBusAxi_awlen(dBusAxi_awlen),
        .dBusAxi_awsize(dBusAxi_awsize),
        .dBusAxi_awburst(dBusAxi_awburst),
        .dBusAxi_awlock(dBusAxi_awlock),
        .dBusAxi_awcache(dBusAxi_awcache),
        .dBusAxi_awqos(dBusAxi_awqos),
        .dBusAxi_awprot(dBusAxi_awprot),
        .dBusAxi_wvalid(dBusAxi_wvalid),
        .dBusAxi_wready(dBusAxi_wready),
        .dBusAxi_wdata(dBusAxi_wdata),
        .dBusAxi_wstrb(dBusAxi_wstrb),
        .dBusAxi_wlast(dBusAxi_wlast),
        .dBusAxi_bvalid(dBusAxi_bvalid),
        .dBusAxi_bready(dBusAxi_bready),
        .dBusAxi_bid(dBusAxi_bid),
        .dBusAxi_bresp(dBusAxi_bresp),
        .dBusAxi_arvalid(dBusAxi_arvalid),
        .dBusAxi_arready(dBusAxi_arready),
        .dBusAxi_araddr(dBusAxi_araddr),
        .dBusAxi_arid(dBusAxi_arid),
        .dBusAxi_arregion(dBusAxi_arregion),
        .dBusAxi_arlen(dBusAxi_arlen),
        .dBusAxi_arsize(dBusAxi_arsize),
        .dBusAxi_arburst(dBusAxi_arburst),
        .dBusAxi_arlock(dBusAxi_arlock),
        .dBusAxi_arcache(dBusAxi_arcache),
        .dBusAxi_arqos(dBusAxi_arqos),
        .dBusAxi_arprot(dBusAxi_arprot),
        .dBusAxi_rvalid(dBusAxi_rvalid),
        .dBusAxi_rready(dBusAxi_rready),
        .dBusAxi_rdata(dBusAxi_rdata),
        .dBusAxi_rid(dBusAxi_rid),
        .dBusAxi_rresp(dBusAxi_rresp),
        .dBusAxi_rlast(dBusAxi_rlast),
        .clk(clk),
        .debugReset(rst)
    );
    bx72_nav_axi adapter (
        .clk(clk),
        .rst(rst),
        .iBusAxi_arvalid(iBusAxi_arvalid),
        .iBusAxi_arready(iBusAxi_arready),
        .iBusAxi_araddr(iBusAxi_araddr),
        .iBusAxi_arid(iBusAxi_arid),
        .iBusAxi_arregion(iBusAxi_arregion),
        .iBusAxi_arlen(iBusAxi_arlen),
        .iBusAxi_arsize(iBusAxi_arsize),
        .iBusAxi_arburst(iBusAxi_arburst),
        .iBusAxi_arlock(iBusAxi_arlock),
        .iBusAxi_arcache(iBusAxi_arcache),
        .iBusAxi_arqos(iBusAxi_arqos),
        .iBusAxi_arprot(iBusAxi_arprot),
        .iBusAxi_rvalid(iBusAxi_rvalid),
        .iBusAxi_rready(iBusAxi_rready),
        .iBusAxi_rdata(iBusAxi_rdata),
        .iBusAxi_rid(iBusAxi_rid),
        .iBusAxi_rresp(iBusAxi_rresp),
        .iBusAxi_rlast(iBusAxi_rlast),
        .dBusAxi_awvalid(dBusAxi_awvalid),
        .dBusAxi_awready(dBusAxi_awready),
        .dBusAxi_awaddr(dBusAxi_awaddr),
        .dBusAxi_awid(dBusAxi_awid),
        .dBusAxi_awregion(dBusAxi_awregion),
        .dBusAxi_awlen(dBusAxi_awlen),
        .dBusAxi_awsize(dBusAxi_awsize),
        .dBusAxi_awburst(dBusAxi_awburst),
        .dBusAxi_awlock(dBusAxi_awlock),
        .dBusAxi_awcache(dBusAxi_awcache),
        .dBusAxi_awqos(dBusAxi_awqos),
        .dBusAxi_awprot(dBusAxi_awprot),
        .dBusAxi_wvalid(dBusAxi_wvalid),
        .dBusAxi_wready(dBusAxi_wready),
        .dBusAxi_wdata(dBusAxi_wdata),
        .dBusAxi_wstrb(dBusAxi_wstrb),
        .dBusAxi_wlast(dBusAxi_wlast),
        .dBusAxi_bvalid(dBusAxi_bvalid),
        .dBusAxi_bready(dBusAxi_bready),
        .dBusAxi_bid(dBusAxi_bid),
        .dBusAxi_bresp(dBusAxi_bresp),
        .dBusAxi_arvalid(dBusAxi_arvalid),
        .dBusAxi_arready(dBusAxi_arready),
        .dBusAxi_araddr(dBusAxi_araddr),
        .dBusAxi_arid(dBusAxi_arid),
        .dBusAxi_arregion(dBusAxi_arregion),
        .dBusAxi_arlen(dBusAxi_arlen),
        .dBusAxi_arsize(dBusAxi_arsize),
        .dBusAxi_arburst(dBusAxi_arburst),
        .dBusAxi_arlock(dBusAxi_arlock),
        .dBusAxi_arcache(dBusAxi_arcache),
        .dBusAxi_arqos(dBusAxi_arqos),
        .dBusAxi_arprot(dBusAxi_arprot),
        .dBusAxi_rvalid(dBusAxi_rvalid),
        .dBusAxi_rready(dBusAxi_rready),
        .dBusAxi_rdata(dBusAxi_rdata),
        .dBusAxi_rid(dBusAxi_rid),
        .dBusAxi_rresp(dBusAxi_rresp),
        .dBusAxi_rlast(dBusAxi_rlast),
        .i_rom_en_o(i_rom_en),
        .i_rom_addr_o(i_rom_addr),
        .d_rom_en_o(d_rom_en),
        .d_rom_addr_o(d_rom_addr),
        .mmio_valid_o(mmio_valid),
        .mmio_write_o(mmio_write),
        .mmio_addr_o(mmio_addr),
        .mmio_wdata_o(mmio_wdata),
        .mmio_wstrb_o(mmio_wstrb),
        .mmio_rsp_ready_o(mmio_rsp_ready),
        .i_rom_data_i(i_rom_data),
        .d_rom_data_i(d_rom_data),
        .mmio_ready_i(mmio_ready),
        .mmio_rsp_valid_i(mmio_rsp_valid),
        .mmio_rdata_i(mmio_rdata),
        .mmio_resp_i(mmio_resp)
    );
    bx72_nav_rom rom (
        .clk(clk), .i_en_i(i_rom_en), .i_addr_i(i_rom_addr), .i_data_o(i_rom_data),
        .d_en_i(d_rom_en), .d_addr_i(d_rom_addr), .d_data_o(d_rom_data)
    );

    always @(posedge clk) begin
        if (rst) begin
            cpu_ready_o <= 1'b0;
            cpu_fault_o <= 1'b0;
            fault_code_o <= 8'd0;
            heartbeat_o <= 32'd0;
            boot_age_q <= 0;
            heartbeat_age_q <= 0;
            trap_cause_q <= 0;
            trap_pc_q <= 0;
            mmio_rsp_valid_q <= 1'b0;
            mmio_rdata_q <= 0;
            mmio_resp_q <= 0;
        end else begin
            if (mmio_rsp_valid_q && mmio_rsp_ready) mmio_rsp_valid_q <= 1'b0;
            if (mmio_fire) begin
                mmio_rsp_valid_q <= 1'b1;
                if (mmio_is_external) begin
                    mmio_rdata_q <= io_rdata_i;
                    mmio_resp_q <= 2'b00;
                end else begin
                    mmio_rdata_q <= internal_read_data;
                    mmio_resp_q <= internal_response;
                    if (mmio_write && internal_response == 2'b00) begin
                        case (mmio_addr)
                            12'h014: trap_cause_q <= mmio_wdata;
                            12'h018: trap_pc_q <= mmio_wdata;
                            default: begin end
                        endcase
                    end
                end
            end

            if (ready_write) begin
                cpu_ready_o <= 1'b1;
                boot_age_q <= 0;
                heartbeat_age_q <= 0;
            end else if (boot_timeout_due) begin
                cpu_fault_o <= 1'b1;
                cpu_ready_o <= 1'b0;
                fault_code_o <= 8'd3;
            end else if (!cpu_ready_o && !cpu_fault_o &&
                         boot_age_q < BOOT_TIMEOUT_CYCLES)
                boot_age_q <= boot_age_q + 1'b1;

            if (heartbeat_write) begin
                heartbeat_o <= mmio_wdata;
                if (cpu_ready_o) heartbeat_age_q <= 0;
            end else if (heartbeat_timeout_due) begin
                cpu_fault_o <= 1'b1;
                cpu_ready_o <= 1'b0;
                fault_code_o <= 8'd3;
            end else if (cpu_ready_o && !cpu_fault_o &&
                         heartbeat_age_q < HEARTBEAT_TIMEOUT_CYCLES)
                heartbeat_age_q <= heartbeat_age_q + 1'b1;

            if (firmware_fault_write) begin
                cpu_fault_o <= 1'b1;
                cpu_ready_o <= 1'b0;
                fault_code_o <= mmio_wdata[7:0] == 0 ? 8'd4 : mmio_wdata[7:0];
            end
        end
    end
endmodule

`define NUM_OF_AXI_PORT 2
`define AXI_ADDR_WIDTH  32
`define AXI_DATA_WIDTH  32
`define AXI_USER_WIDTH  6
`define AXI_ID_WIDTH    6
`define AXI_DATA_STRB   `AXI_DATA_WIDTH/8

module tools_core (

// LEDs
output [3:0]    LED,

// USB3 FT601 Interface
input           ftdi_clk,
input           ftdi_rxf_n,
input           ftdi_txe_n,
output          ftdi_oe_n,
output          ftdi_rd_n,
output          ftdi_wr_n,
input  [31:0]   ftdi_data_IN,
output [31:0]   ftdi_data_OUT,
output [31:0]   ftdi_data_OE,
input  [3:0]    ftdi_be_IN,
output [3:0]    ftdi_be_OUT,
output [3:0]    ftdi_be_OE,

// 100MHz clock for USB processing
input           clk_100,

// DDR Interface
input           axi0_ACLK,
output          axi0_ARESETn,
output          axi0_ARQOS,
output          axi0_AWQOS,
output [5:0]    axi0_AWID,
output [32:0]   axi0_AWADDR,
output [7:0]    axi0_AWLEN,
output [2:0]    axi0_AWSIZE,
output [1:0]    axi0_AWBURST,
output          axi0_AWVALID,
output [3:0]    axi0_AWCACHE,
output          axi0_AWCOBUF,
output          axi0_AWLOCK,
output          axi0_AWAPCMD,
output          axi0_AWALLSTRB,
output [5:0]    axi0_ARID,
output [32:0]   axi0_ARADDR,
output [7:0]    axi0_ARLEN,
output [2:0]    axi0_ARSIZE,
output [1:0]    axi0_ARBURST,
output          axi0_ARVALID,
output          axi0_ARLOCK,
output          axi0_ARAPCMD,
output          axi0_WLAST,
output          axi0_WVALID,
output [511:0]  axi0_WDATA,
output [63:0]   axi0_WSTRB,
output          axi0_BREADY,
output          axi0_RREADY,
input           axi0_AWREADY,
input           axi0_ARREADY,
input           axi0_WREADY,
input [5:0]     axi0_BID,
input [1:0]     axi0_BRESP,
input           axi0_BVALID,
input [5:0]     axi0_RID,
input           axi0_RLAST,
input           axi0_RVALID,
input [511:0]   axi0_RDATA,
input [1:0]     axi0_RRESP,

input           axi1_ACLK,
output          axi1_ARESETn,
output          axi1_ARQOS,
output          axi1_AWQOS,
output [5:0]    axi1_AWID,
output [32:0]   axi1_AWADDR,
output [7:0]    axi1_AWLEN,
output [2:0]    axi1_AWSIZE,
output [1:0]    axi1_AWBURST,
output          axi1_AWVALID,
output [3:0]    axi1_AWCACHE,
output          axi1_AWCOBUF,
output          axi1_AWLOCK,
output          axi1_AWAPCMD,
output          axi1_AWALLSTRB,
output [5:0]    axi1_ARID,
output [32:0]   axi1_ARADDR,
output [7:0]    axi1_ARLEN,
output [2:0]    axi1_ARSIZE,
output [1:0]    axi1_ARBURST,
output          axi1_ARVALID,
output          axi1_ARLOCK,
output          axi1_ARAPCMD,
output          axi1_WLAST,
output          axi1_WVALID,
output [511:0]  axi1_WDATA,
output [63:0]   axi1_WSTRB,
output          axi1_BREADY,
output          axi1_RREADY,
input           axi1_AWREADY,
input           axi1_ARREADY,
input           axi1_WREADY,
input [5:0]     axi1_BID,
input [1:0]     axi1_BRESP,
input           axi1_BVALID,
input [5:0]     axi1_RID,
input           axi1_RLAST,
input           axi1_RVALID,
input [511:0]   axi1_RDATA,
input [1:0]     axi1_RRESP,

output          cfg_sel,
output          cfg_start,
output          cfg_reset,
input           cfg_done,
output          phy_rstn,
output          ctrl_rstn,
input           ddr_pll_lock,
output          ddr_pll_rstn,
//config
input           regACLK,
output [14:0]   regARADDR,
output [5:0]    regARID,
output [7:0]    regARLEN,
output [2:0]    regARSIZE,
output [1:0]    regARBURST,
output          regARVALID,
input           regARREADY,
input [31:0]    regRDATA,
input           regRVALID,
input           regRLAST,
input [1:0]     regRRESP,
input [5:0]     regRID,
output          regRREADY,
output [14:0]   regAWADDR,
output [5:0]    regAWID,
output [7:0]    regAWLEN,
output [2:0]    regAWSIZE,
output [1:0]    regAWBURST,
output          regAWVALID,
input           regAWREADY,

output [31:0]   regWDATA,
output [3:0]    regWSTRB,
output          regWLAST,
output          regWVALID,
input           regWREADY,

output          regBREADY,
input [5:0]     regBID,
input [1:0]     regBRESP,
input           regBVALID,

output          regARESETn,

input  wire jtag_inst1_CAPTURE,
input  wire jtag_inst1_DRCK,
input  wire jtag_inst1_RESET,
input  wire jtag_inst1_RUNTEST,
input  wire jtag_inst1_SEL,
input  wire jtag_inst1_SHIFT,
input  wire jtag_inst1_TCK,
input  wire jtag_inst1_TDI,
input  wire jtag_inst1_TMS,
input  wire jtag_inst1_UPDATE,
output wire jtag_inst1_TDO
);

//assign  cfg_sel         = 1'b1;
//assign  cfg_start       = ddr_pll_lock;
wire    done_0;
wire    fail_0;
assign  ddr_pll_rstn = 1'b1;

wire memtest_start;
wire memtest_rstn;
wire [63:0]	w_memtest_data;
wire [31:0]	w_memtest_size;
wire       	w_memtest_lfsr_en;
wire x16_en;
wire [31:0] dq_fail;
wire [7:0]  reg_axi_arlen;

memory_checker_lfsr checker0(
    .axi_clk    (axi0_ACLK),
    .rstn       (memtest_rstn),
    .start      (memtest_start),
    .awid       (axi0_AWID),
    .awaddr     (axi0_AWADDR),
    .awlen      (axi0_AWLEN),
    .awsize     (axi0_AWSIZE),
    .awburst    (axi0_AWBURST),
    .awcache    (axi0_AWCACHE),
    .awlock     (axi0_AWLOCK),
    .awvalid    (axi0_AWVALID),
    .awcobuf    (axi0_AWCOBUF),
    .awapcmd    (axi0_AWAPCMD),
    .awallstrb  (axi0_AWALLSTRB),
    .awready    (axi0_AWREADY),
    .awqos      (axi0_AWQOS),
    .arid       (axi0_ARID),
    .araddr     (axi0_ARADDR),
    .arlen      (axi0_ARLEN),
    .arsize     (axi0_ARSIZE),
    .arburst    (axi0_ARBURST),
    .arlock     (axi0_ARLOCK),
    .arvalid    (axi0_ARVALID),
    .arapcmd    (axi0_ARAPCMD),
    .arready    (axi0_ARREADY),
    .arqos      (axi0_ARQOS),
    .wdata      (axi0_WDATA),
    .wstrb      (axi0_WSTRB),
    .wlast      (axi0_WLAST),
    .wvalid     (axi0_WVALID),
    .wready     (axi0_WREADY),
    .rid        (axi0_RID),
    .rdata      (axi0_RDATA),
    .rlast      (axi0_RLAST),
    .rvalid     (axi0_RVALID),
    .rready     (axi0_RREADY),
    .rresp      (axi0_RRESP),
    .bid        (axi0_BID),
    .bvalid     (axi0_BVALID),
    .bready     (axi0_BREADY),
    .fail       (fail_0),
    .done       (done_0),
    .dq_fail_expression (dq_fail),
    .i_lfsr_seed        ({2{w_memtest_data}}),
    .lfsr_en            (w_memtest_lfsr_en),
    .x16_en             (x16_en),
    .check_mask         (32'hFFFFFFFF),
    .test_size          (w_memtest_size)
);

wire w_loop_done;
wire [63:0] w_loop_cnt;
wire [63:0] w_loop_len;
wire w_compare_error;
wire w_tester_rst;
wire[31:0]w_tester_pattern;

// ============================================================================
// CORRECTED BANK-INTERLEAVED ADDRESSING FOR LPDDR4
// ============================================================================
// Based on Efinix Ti DDR Controller User Guide v2.8 (page 17):
// - Bank address is at bits [11:9] for x32 width LPDDR4
// - Bursts must not cross 4KB boundaries (page 11)
//
// Address mapping (x32 LPDDR4):
//   [31:15] = Row address
//   [14:12] = Higher column bits
//   [11:9]  = Bank address (8 banks: 000-111)
//   [8:2]   = Lower column bits
//   [1:0]   = Datapath select
//
// Strategy: Cycle through all 8 banks with proper bank interleaving
// ============================================================================

wire [528:0] w_seq_addr_w;
wire [528:0] w_seq_addr_r;

// OPTION 1: Bank interleaving with Row 0 - Maximize page hits (BEST: 77%)
// Strategy: All Row 0, bank-interleaved via bits[11:9] with 512B stride
// This maximizes page hits (no activate/precharge overhead)
wire [528:0] w_addr_opt1 = {
    33'h00000E00,  // Row 0, Bank 7: bits[11:9]=111
    33'h00000C00,  // Row 0, Bank 6: bits[11:9]=110
    33'h00000A00,  // Row 0, Bank 5: bits[11:9]=101
    33'h00000800,  // Row 0, Bank 4: bits[11:9]=100
    33'h00000600,  // Row 0, Bank 3: bits[11:9]=011
    33'h00000400,  // Row 0, Bank 2: bits[11:9]=010
    33'h00000200,  // Row 0, Bank 1: bits[11:9]=001
    33'h00000000,  // Row 0, Bank 0: bits[11:9]=000
    // Repeat pattern for 16 total addresses
    33'h00000E00,  // Row 0, Bank 7
    33'h00000C00,  // Row 0, Bank 6
    33'h00000A00,  // Row 0, Bank 5
    33'h00000800,  // Row 0, Bank 4
    33'h00000600,  // Row 0, Bank 3
    33'h00000400,  // Row 0, Bank 2
    33'h00000200,  // Row 0, Bank 1
    33'h00000000   // Row 0, Bank 0
};

// OPTION 2: 4KB-aligned addresses, Row 0 - Also maximizes page hits (BEST: 77%)
// Strategy: 4KB-aligned, all Row 0, naturally spreads across banks
wire [528:0] w_addr_opt2 = {
    33'h0000F000,  // Row 0, 60KB
    33'h0000E000,  // Row 0, 56KB
    33'h0000D000,  // Row 0, 52KB
    33'h0000C000,  // Row 0, 48KB
    33'h0000B000,  // Row 0, 44KB
    33'h0000A000,  // Row 0, 40KB
    33'h00009000,  // Row 0, 36KB
    33'h00008000,  // Row 0, 32KB
    33'h00007000,  // Row 0, 28KB
    33'h00006000,  // Row 0, 24KB
    33'h00005000,  // Row 0, 20KB
    33'h00004000,  // Row 0, 16KB
    33'h00003000,  // Row 0, 12KB
    33'h00002000,  // Row 0, 8KB
    33'h00001000,  // Row 0, 4KB
    33'h00000000   // Row 0, 0KB
};

// OPTION 3: Simple large stride - tests sequential without bank optimization
wire [528:0] w_addr_opt3 = {
    33'h0003C000, 33'h00038000, 33'h00034000, 33'h00030000,
    33'h0002C000, 33'h00028000, 33'h00024000, 33'h00020000,
    33'h0001C000, 33'h00018000, 33'h00014000, 33'h00010000,
    33'h0000C000, 33'h00008000, 33'h00004000, 33'h00000000
};

// SELECT WHICH OPTION TO USE (change this to try different options)
assign w_seq_addr_w = w_addr_opt1;  // <<< CHANGE THIS to opt1/opt2/opt3

assign w_seq_addr_r = w_seq_addr_w;  // Use same addresses for reads

axi_ctrl_ver3 #(
    .NUM_AXI_IDS(16)  // Use multiple outstanding transaction IDs for pipelining
) tester0(

.axi_clk(axi1_ACLK),
.rstn(w_tester_rst),

.W_INADDR(w_seq_addr_w),
.R_INADDR(w_seq_addr_r),//new
.TRIGGER(1'b1),
.LOOP_N(32'd0),
.PATTERN_NUMBER(4'd1),
.W_IN_PATTERN(w_tester_pattern[15:0]),
.R_IN_PATTERN(w_tester_pattern[31:16]),//new

//AXI 4
.DDR_ARID_0(axi1_ARID),
.DDR_ARADDR_0(axi1_ARADDR),
.DDR_ARLEN_0(axi1_ARLEN),
.DDR_ARSIZE_0(axi1_ARSIZE),
.DDR_ARBURST_0(axi1_ARBURST),
.DDR_ARLOCK_0(axi1_ARLOCK),
.DDR_ARVALID_0(axi1_ARVALID),
.DDR_ARREADY_0(axi1_ARREADY),
.DDR_ARQOS_0(axi1_ARQOS),
.DDR_ARAPCMD_0(axi1_ARAPCMD),

.DDR_AWID_0(axi1_AWID),
.DDR_AWADDR_0(axi1_AWADDR),
.DDR_AWLEN_0(axi1_AWLEN),
.DDR_AWSIZE_0(axi1_AWSIZE),
.DDR_AWBURST_0(axi1_AWBURST),
.DDR_AWLOCK_0(axi1_AWLOCK),
.DDR_AWVALID_0(axi1_AWVALID),
.DDR_AWREADY_0(axi1_AWREADY),
.DDR_AWQOS_0(axi1_AWQOS),
.DDR_AWAPCMD_0(axi1_AWAPCMD),
.DDR_AWCACHE_0(axi1_AWCACHE),
.DDR_AWALLSTRB_0(axi1_AWALLSTRB),
.DDR_AWCOBUF_0(axi1_AWCOBUF),

//==========================================================
.DDR_WID_0(axi1_WID),
.DDR_WDATA_0(axi1_WDATA),
.DDR_WSTRB_0(axi1_WSTRB),
.DDR_WLAST_0(axi1_WLAST),
.DDR_WVALID_0(axi1_WVALID),
.DDR_WREADY_0(axi1_WREADY),

.DDR_RID_0(axi1_RID),
.DDR_RDATA_0(axi1_RDATA),
.DDR_RLAST_0(axi1_RLAST),
.DDR_RVALID_0(axi1_RVALID),
.DDR_RREADY_0(axi1_RREADY),
.DDR_RRESP_0(axi1_RRESP),

.DDR_BID_0(axi1_BID),
.DDR_BVALID_0(axi1_BVALID),
.DDR_BREADY_0(axi1_BREADY),

//==========================================================

.o_states(),
.o_out_trig(),

.i_pattern_len({16{8'd255}}), // 256-transfer bursts (0-255) for maximum efficiency
//.i_pattern_len({16{8'd63}}),  // 64-transfer bursts = 4KB (no boundary crossing)
.o_pattern_cnt(),
.o_pattern_done(w_loop_done),
.o_Start(),
.o_time_counter(),
.o_write_cnt(),
.o_total_len(),
.o_loop_n(),
.i_lfsr_seed({4{32'h5555AAAA}}),
.i_active_factor(8'b0),
.i_fix_data_pattern({8{32'h5555AAAA}}),
.i_pause(1'b0),
.loop_done(),
.loop_cnt(w_loop_cnt),
.loop_len(w_loop_len),

.o_compare_error(w_compare_error)
);

// ================================================================
// ==================== J2A Session Start =========================
// ================================================================

// ==================== AXI Write Channel =========================
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_aw_ready;
wire [`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:0]   axi_master_aw_id;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_aw_valid;
wire [`NUM_OF_AXI_PORT*`AXI_ADDR_WIDTH-1:0] axi_master_aw_addr;
wire [`NUM_OF_AXI_PORT*8-1:0]               axi_master_aw_len;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_aw_size;
wire [`NUM_OF_AXI_PORT*2-1:0]               axi_master_aw_burst;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_aw_lock;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_aw_prot;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_aw_region;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_aw_cache;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_aw_qos;
wire [`NUM_OF_AXI_PORT*`AXI_USER_WIDTH-1:0] axi_master_aw_user;
// ==================== AXI Read  Channel =========================
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_ar_ready;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_ar_valid;
wire [`NUM_OF_AXI_PORT*`AXI_ADDR_WIDTH-1:0] axi_master_ar_addr;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_ar_prot;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_ar_region;
wire [`NUM_OF_AXI_PORT*8-1:0]               axi_master_ar_len;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_ar_size;
wire [`NUM_OF_AXI_PORT*2-1:0]               axi_master_ar_burst;
wire [`NUM_OF_AXI_PORT*3-1:0]               axi_master_ar_lock;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_ar_cache;
wire [`NUM_OF_AXI_PORT*4-1:0]               axi_master_ar_qos;
wire [`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:0]   axi_master_ar_id;
wire [`NUM_OF_AXI_PORT*`AXI_USER_WIDTH-1:0] axi_master_ar_user;
// ==================== AXI Write Data Channel ====================
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_w_ready;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_w_valid;
wire [`NUM_OF_AXI_PORT*`AXI_DATA_WIDTH-1:0] axi_master_w_data;
wire [`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:0]   axi_master_w_id;
wire [`NUM_OF_AXI_PORT*`AXI_DATA_STRB-1:0]  axi_master_w_strb;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_w_last;
wire [`NUM_OF_AXI_PORT*`AXI_USER_WIDTH-1:0] axi_master_w_user;
// ==================== AXI Write Response Channel ================
wire [`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:0]   axi_master_b_id;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_b_valid;
wire [`NUM_OF_AXI_PORT*2-1:0]               axi_master_b_resp;
wire [`NUM_OF_AXI_PORT*`AXI_USER_WIDTH-1:0] axi_master_b_user;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_b_ready;
// ==================== AXI Read Data Channel =====================
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_r_valid;
wire [`NUM_OF_AXI_PORT*`AXI_DATA_WIDTH-1:0] axi_master_r_data;
wire [`NUM_OF_AXI_PORT*2-1:0]               axi_master_r_resp;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_r_last;
wire [`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:0]   axi_master_r_id;
wire [`NUM_OF_AXI_PORT*`AXI_USER_WIDTH-1:0] axi_master_r_user;
wire [`NUM_OF_AXI_PORT-1:0]                 axi_master_r_ready;

// =============== To replace below module with direct J2A signas ===============
    assign regARADDR                              = axi_master_ar_addr[14:0];              // DDR design only used up to 15 bits, hence discarded the top
    assign regARID                                = 6'h1;//axi_master_ar_id[`AXI_ID_WIDTH:0];
    //assign regARLEN                               = 8'h0;//axi_master_ar_len[7:0];
    assign regARLEN                               = reg_axi_arlen;//axi_master_ar_len[7:0];
    assign regARSIZE                              = 3'h2;//axi_master_ar_size[2:0];
    assign regARBURST                             = 2'h1;//axi_master_ar_burst[1:0];
    assign regARVALID                             = axi_master_ar_valid[0];
    assign axi_master_ar_ready[0]                 = regARREADY;

    assign axi_master_r_data[`AXI_DATA_WIDTH-1:0] = regRDATA;
    assign axi_master_r_valid[0]                  = regRVALID;
    assign axi_master_r_last[0]                   = regRLAST;
    assign axi_master_r_resp[1:0]                 = regRRESP;
    assign axi_master_r_id[`AXI_ID_WIDTH-1:0]     = regRID;
    assign regRREADY                              = axi_master_r_ready[0];

    assign regAWADDR                              = axi_master_aw_addr[14:0];             // DDR design only used up to 15 bits, hence discarded the top
    assign regAWID                                = 6'h1;//axi_master_aw_id[`AXI_ID_WIDTH:0];
    assign regAWLEN                               = 8'h0;//axi_master_aw_len[7:0];
    assign regAWSIZE                              = 3'h2;//axi_master_aw_size[2:0];
    assign regAWBURST                             = 2'h1;//axi_master_aw_burst[1:0];
    assign regAWVALID                             = axi_master_aw_valid[0];
    assign axi_master_aw_ready[0]                 = regAWREADY;

    assign regWDATA                               = axi_master_w_data[`AXI_DATA_WIDTH-1:0];
    assign regWSTRB                               = axi_master_w_strb[`AXI_DATA_WIDTH/8-1:0];
    assign regWLAST                               = axi_master_w_last[0];
    assign regWVALID                              = axi_master_w_valid[0];
    assign axi_master_w_ready[0]                  = regWREADY;

    assign regBREADY                              = axi_master_b_ready[0];
    assign axi_master_b_id[`AXI_ID_WIDTH-1:0]     = regBID;
    assign axi_master_b_valid[0]                  = regBVALID;
    assign axi_master_b_resp[1:0]                 = regBRESP;

// =========================================
// =============== J2A Debug ===============
// =========================================

assign osc_inst1_ENA = 1'b1;


jtag2axi_debugger #(
    .NUM_OF_AXI_PORT (`NUM_OF_AXI_PORT),
    .AXI_ADDR_WIDTH  (`AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH  (`AXI_DATA_WIDTH),
    .AXI_USER_WIDTH  (`AXI_USER_WIDTH),
    .AXI_ID_WIDTH    (`AXI_ID_WIDTH)
) jtag2axi_debugger (
    .jtag_inst1_CAPTURE     (jtag_inst1_CAPTURE),
    .jtag_inst1_DRCK        (jtag_inst1_DRCK),
    .jtag_inst1_RESET       (jtag_inst1_RESET),
    .jtag_inst1_RUNTEST     (jtag_inst1_RUNTEST),
    .jtag_inst1_SEL         (jtag_inst1_SEL),
    .jtag_inst1_SHIFT       (jtag_inst1_SHIFT),
    .jtag_inst1_TCK         (jtag_inst1_TCK),
    .jtag_inst1_TDI         (jtag_inst1_TDI),
    .jtag_inst1_TMS         (jtag_inst1_TMS),
    .jtag_inst1_UPDATE      (jtag_inst1_UPDATE),
    .jtag_inst1_TDO         (jtag_inst1_TDO),
    .axi_aclk               (regACLK),
    .axi_aresetn            (ddr_pll_lock),
    .axi_master_aw_ready    (axi_master_aw_ready),
    .axi_master_aw_valid    (axi_master_aw_valid),
    .axi_master_aw_addr     (axi_master_aw_addr),
    .axi_master_aw_len      (axi_master_aw_len),
    .axi_master_aw_size     (axi_master_aw_size),
    .axi_master_aw_burst    (axi_master_aw_burst),
    .axi_master_aw_lock     (axi_master_aw_lock),
    .axi_master_aw_id       (axi_master_aw_id),
    .axi_master_ar_valid    (axi_master_ar_valid),
    .axi_master_ar_addr     (axi_master_ar_addr),
    .axi_master_ar_len      (axi_master_ar_len),
    .axi_master_ar_size     (axi_master_ar_size),
    .axi_master_ar_ready    (axi_master_ar_ready),
    .axi_master_w_valid     (axi_master_w_valid),
    .axi_master_w_data      (axi_master_w_data),
    .axi_master_w_strb      (axi_master_w_strb),
    .axi_master_w_id        (axi_master_w_id),
    .axi_master_w_last      (axi_master_w_last),
    .axi_master_w_ready     (axi_master_w_ready),
    .axi_master_b_valid     (axi_master_b_valid),
    .axi_master_b_id        (axi_master_b_id),
    .axi_master_b_resp      (axi_master_b_resp),
    .axi_master_b_ready     (axi_master_b_ready),
    .axi_master_r_valid     (axi_master_r_valid),
    .axi_master_r_data      (axi_master_r_data),
    .axi_master_r_resp      (axi_master_r_resp),
    .axi_master_r_last      (axi_master_r_last),
    .axi_master_r_id        (axi_master_r_id),
    .axi_master_r_ready     (axi_master_r_ready)
);

//AW
wire [31:0]	s_axi_awaddr;
wire 		s_axi_awready;
wire 		s_axi_awvalid;

//W
wire		s_axi_wready;
wire [31:0]	s_axi_wdata;
wire		s_axi_wvalid;
wire		s_axi_wlast;
wire [3:0]  s_axi_wstrb;

//B
wire [7:0]	s_axi_bid;        //not use
wire [1:0]	s_axi_bresp;      //not use
wire 		s_axi_bvalid;
wire		s_axi_bready;

//AR
wire[31:0]	s_axi_araddr;
wire		s_axi_arvalid;
wire		s_axi_arready;

//R
wire [7:0]	s_axi_rid;        //not use
wire [1:0]	s_axi_rresp;      //not use
wire 		s_axi_rready;
wire [31:0]	s_axi_rdata;
wire 		s_axi_rvalid;
wire 		s_axi_rlast;


//AW
assign axi_master_aw_ready[1]   =s_axi_awready;
assign s_axi_awaddr             =axi_master_aw_addr[`NUM_OF_AXI_PORT*`AXI_ADDR_WIDTH-1:`AXI_ADDR_WIDTH];
assign s_axi_awvalid            =axi_master_aw_valid[1];


assign axi_master_w_ready[1] =s_axi_wready;
assign s_axi_wdata  =axi_master_w_data[`NUM_OF_AXI_PORT*`AXI_DATA_WIDTH-1:`AXI_DATA_WIDTH];
assign s_axi_wvalid =axi_master_w_valid[1];
assign s_axi_wlast  =axi_master_w_last[1];
assign s_axi_wstrb  =4'b1111;


assign axi_master_b_id[`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:`AXI_ID_WIDTH]   = s_axi_bid[`AXI_ID_WIDTH-1:0];        //not use
assign axi_master_b_resp[`NUM_OF_AXI_PORT*2-1:2]                        =s_axi_bresp;      //not use
assign axi_master_b_valid[1]                                            =s_axi_bvalid;
assign s_axi_bready                                                     =axi_master_b_ready[1];


assign axi_master_ar_ready[1] =s_axi_arready;
assign s_axi_araddr = axi_master_ar_addr[`NUM_OF_AXI_PORT*`AXI_ADDR_WIDTH-1:`AXI_ADDR_WIDTH];
assign s_axi_arvalid =axi_master_ar_valid[1];


assign s_axi_rready =axi_master_r_ready[1];

assign axi_master_r_id[`NUM_OF_AXI_PORT*`AXI_ID_WIDTH-1:`AXI_ID_WIDTH]       =s_axi_rid[`AXI_ID_WIDTH-1:0];        //not use
assign axi_master_r_resp[`NUM_OF_AXI_PORT*2-1:2]                            =s_axi_rresp;      //not use
assign axi_master_r_data[`NUM_OF_AXI_PORT*`AXI_DATA_WIDTH-1:`AXI_DATA_WIDTH] =s_axi_rdata;
assign axi_master_r_valid[1]                                                =s_axi_rvalid;
assign axi_master_r_last[1]                                                 =s_axi_rlast;


axi_lite_slave axilite_inst
(
    .axi_aclk(regACLK),
	.axi_resetn(ddr_pll_lock),

	.axi_awaddr (s_axi_awaddr),
	.axi_awready(s_axi_awready),
	.axi_awvalid(s_axi_awvalid),

    .axi_wready (s_axi_wready),
	.axi_wdata  (s_axi_wdata),
    .axi_wvalid (s_axi_wvalid),
    .axi_wlast  (s_axi_wlast),
	.axi_wstrb  (s_axi_wstrb),

	.axi_bid    (s_axi_bid),        //not use
	.axi_bresp  (s_axi_bresp),      //not use
	.axi_bvalid (s_axi_bvalid),
	.axi_bready (s_axi_bready),

    .axi_araddr (s_axi_araddr),
	.axi_arvalid(s_axi_arvalid),
	.axi_arready(s_axi_arready),

	.axi_rid    (s_axi_rid),        //not use
	.axi_rresp  (s_axi_rresp),      //not use
    .axi_rready (s_axi_rready),
    .axi_rdata  (s_axi_rdata),
    .axi_rvalid (s_axi_rvalid),
    .axi_rlast  (s_axi_rlast),

    .memtest_start  (memtest_start),
	.memtest_rstn   (memtest_rstn),
	.memtest_fail   (fail_0),
	.memtest_done   (done_0),
    .memtest_data   (w_memtest_data),
    .memtest_lfsr_en(w_memtest_lfsr_en),
    .memtest_x16_en (x16_en),
    .phy_rstn       (phy_rstn),
    .ctrl_rstn      (ctrl_rstn),
    .reg_axi_rstn   (regARESETn),
    .axi0_rstn      (axi0_ARESETn),
    .axi1_rstn      (axi1_ARESETn),
    .reg_axi_arlen  (reg_axi_arlen),
    .memtest_size   (w_memtest_size),
	.dq_fail        (dq_fail),
    .config_rst     (cfg_reset),
    .config_sel     (cfg_sel),
    .config_start   (cfg_start),
    .config_done    (cfg_done),
    .tester_loop_len(w_loop_len),
    .tester_loop_cnt(w_loop_cnt),
    .tester_loop_done(w_loop_done),
    .tester_error(w_compare_error),
    .tester_rst(w_tester_rst),
    .tester_pattern(w_tester_pattern)
);

// ================================================================
// ==================== USB3 Interface ============================
// ================================================================
// Simple USB3 test interface: receives 4 bytes (length),
// then sends back that many bytes (for usb_rx_mass.py testing)

// USB RX/TX AXI-Stream signals
wire        usb_rx_tready;
wire        usb_rx_tvalid;
wire [ 7:0] usb_rx_tdata;

wire        usb_tx_tready;
wire        usb_tx_tvalid;
wire [31:0] usb_tx_tdata;
wire [ 3:0] usb_tx_tkeep;
wire        usb_tx_tlast;

// FTDI 245FIFO interface controller
ftdi_245fifo_top #(
    .TX_EW                 ( 2                  ),   // TX data stream width,  2=32bit
    .TX_EA                 ( 14                 ),   // TX FIFO depth = 2^14 = 16384
    .RX_EW                 ( 0                  ),   // RX data stream width,  0=8bit
    .RX_EA                 ( 8                  ),   // RX FIFO depth = 2^8 = 256
    .CHIP_TYPE             ( "FT601"            )
) u_ftdi_245fifo_top (
    .rstn_async            ( 1'b1               ),
    .tx_clk                ( clk_100            ),
    .tx_tready             ( usb_tx_tready      ),
    .tx_tvalid             ( usb_tx_tvalid      ),
    .tx_tdata              ( usb_tx_tdata       ),
    .tx_tkeep              ( usb_tx_tkeep       ),
    .tx_tlast              ( usb_tx_tlast       ),
    .rx_clk                ( clk_100            ),
    .rx_tready             ( usb_rx_tready      ),
    .rx_tvalid             ( usb_rx_tvalid      ),
    .rx_tdata              ( usb_rx_tdata       ),
    .rx_tkeep              (                    ),
    .rx_tlast              (                    ),
    .ftdi_clk              ( ftdi_clk           ),
    .ftdi_rxf_n            ( ftdi_rxf_n         ),
    .ftdi_txe_n            ( ftdi_txe_n         ),
    .ftdi_oe_n             ( ftdi_oe_n          ),
    .ftdi_rd_n             ( ftdi_rd_n          ),
    .ftdi_wr_n             ( ftdi_wr_n          ),
    .ftdi_data_IN          ( ftdi_data_IN       ),
    .ftdi_data_OUT         ( ftdi_data_OUT      ),
    .ftdi_data_OE          ( ftdi_data_OE       ),
    .ftdi_be_IN            ( ftdi_be_IN         ),
    .ftdi_be_OUT           ( ftdi_be_OUT        ),
    .ftdi_be_OE            ( ftdi_be_OE         )
);

// TX specified length module (receives 4 bytes as length, sends that many bytes back)
tx_specified_len u_tx_specified_len (
    .rstn                  ( 1'b1               ),
    .clk                   ( clk_100            ),
    .i_tready              ( usb_rx_tready      ),
    .i_tvalid              ( usb_rx_tvalid      ),
    .i_tdata               ( usb_rx_tdata       ),
    .o_tready              ( usb_tx_tready      ),
    .o_tvalid              ( usb_tx_tvalid      ),
    .o_tdata               ( usb_tx_tdata       ),
    .o_tkeep               ( usb_tx_tkeep       ),
    .o_tlast               ( usb_tx_tlast       )
);

// ================================================================
// ==================== LED Indicators ============================
// ================================================================
// LED[1:0] - Show low 2 bits of last received USB data
// LED[2]   - Heartbeat from clk_100 (blinks at 5Hz)
// LED[3]   - Heartbeat from ftdi_clk (blinks at 5Hz)

reg [1:0] usb_tdata_d = 2'h0;

always @ (posedge clk_100)
    if (usb_rx_tvalid)
        usb_tdata_d <= usb_rx_tdata[1:0];

wire clk_100_beat;
wire ftdi_clk_beat;

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_clk_100_beat (
    .clk                   ( clk_100            ),
    .beat                  ( clk_100_beat       )
);

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_ftdi_clk_beat (
    .clk                   ( ftdi_clk           ),
    .beat                  ( ftdi_clk_beat      )
);

assign LED[1:0] = usb_tdata_d;
assign LED[2]   = clk_100_beat;
assign LED[3]   = ftdi_clk_beat;

endmodule

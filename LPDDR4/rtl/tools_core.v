`define NUM_OF_AXI_PORT 1
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

output          regARESETn
);

// ================================================================
// DDR Auto-Initialization (Hardware Mode)
// ================================================================
// Configure DDR to use built-in initialization (cfg_sel=0)
// and automatically start when DDR PLL locks
assign  cfg_sel   = 1'b0;           // Use built-in configuration from bitstream
assign  cfg_reset = 1'b0;           // No reset assertion
assign  cfg_start = ddr_pll_lock;   // Start initialization when DDR PLL locks

// Note: cfg_done will go high when DDR initialization completes
// This typically takes 100-500ms after PLL lock

// Release all DDR resets when PLL locks
assign  phy_rstn      = ddr_pll_lock;   // PHY reset released when PLL locks
assign  ctrl_rstn     = ddr_pll_lock;   // Controller reset released when PLL locks
assign  regARESETn    = ddr_pll_lock;   // Register AXI reset released when PLL locks
assign  axi0_ARESETn  = ddr_pll_lock;   // AXI0 reset released when PLL locks
assign  axi1_ARESETn  = ddr_pll_lock;   // AXI1 reset released when PLL locks

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

// =============== DDR Controller Register Access (JTAG + USB) ===============
// Allow both JTAG (axi_master_*[0]) and USB (bridge_m1_*) to access DDR registers
// Track which master made the request and route response back correctly

    // ========== Clock Domain Note ==========
    // USB bridge runs on clk_100 (100 MHz), DDR registers run on regACLK (100 MHz)
    // BOTH CLOCKS ARE THE SAME FREQUENCY - No clock domain crossing!
    //
    // Previous attempts with regACLK at 50 MHz caused issues:
    // 1. Adding CDC synchronizers broke AXI handshake timing (latency issues)
    // 2. Direct connection without sync had metastability/sampling issues
    // 3. Using async FIFOs or AXI clock converter would add complexity
    //
    // Solution: Set regACLK = 100 MHz to match clk_100
    // This eliminates all CDC issues and allows direct, reliable connections.
    // All AXI signals are now in the same clock domain.

    // Latch which master initiated the read transaction
    // IMPORTANT: Use regACLK (DDR register clock) and unsynchronized signals
    reg rd_from_usb = 1'b0;
    always @(posedge regACLK) begin
        if (regARREADY && regARVALID) begin
            // Latch: did USB initiate this read? (using direct signal - safe for mesochronous)
            rd_from_usb <= bridge_m1_axi_arvalid;
        end
    end

    // Latch which master initiated the write transaction
    reg wr_from_usb = 1'b0;
    always @(posedge regACLK) begin
        if (regAWREADY && regAWVALID) begin
            // Latch: did USB initiate this write? (using direct signal - safe for mesochronous)
            wr_from_usb <= bridge_m1_axi_awvalid;
        end
    end

    // Read Address Channel - USB has priority
    // Use direct signals (no sync) since clocks are mesochronous
    // IMPORTANT: Must mux ALL transaction attributes based on which master is active
    assign regARADDR                              = bridge_m1_axi_arvalid ? bridge_m1_axi_araddr : axi_master_ar_addr[14:0];
    assign regARID                                = 6'h1;
    assign regARLEN                               = bridge_m1_axi_arvalid ? 8'h0 : reg_axi_arlen;    // USB: single beat
    assign regARSIZE                              = 3'h2;                                             // Both: 4 bytes
    assign regARBURST                             = bridge_m1_axi_arvalid ? 2'h1 : 2'h1;             // Both: INCR
    assign regARVALID                             = bridge_m1_axi_arvalid | axi_master_ar_valid[0];
    assign bridge_m1_axi_arready                  = regARREADY & bridge_m1_axi_arvalid;
    assign axi_master_ar_ready[0]                 = regARREADY & !bridge_m1_axi_arvalid;

    // Read Data Channel - route based on who made the request
    assign bridge_m1_axi_rdata                    = regRDATA;
    assign bridge_m1_axi_rvalid                   = regRVALID & rd_from_usb;
    assign bridge_m1_axi_rresp                    = regRRESP;
    assign axi_master_r_data[`AXI_DATA_WIDTH-1:0] = regRDATA;
    assign axi_master_r_valid[0]                  = regRVALID & !rd_from_usb;
    assign axi_master_r_last[0]                   = regRLAST;
    assign axi_master_r_resp[1:0]                 = regRRESP;
    assign axi_master_r_id[`AXI_ID_WIDTH-1:0]     = regRID;
    assign regRREADY                              = rd_from_usb ? bridge_m1_axi_rready : axi_master_r_ready[0];

    // Write Address Channel - USB has priority
    // Use direct signals (no sync) since clocks are mesochronous
    // IMPORTANT: Must mux ALL transaction attributes based on which master is active
    assign regAWADDR                              = bridge_m1_axi_awvalid ? bridge_m1_axi_awaddr : axi_master_aw_addr[14:0];
    assign regAWID                                = 6'h1;
    assign regAWLEN                               = 8'h0;                  // Both: single beat
    assign regAWSIZE                              = 3'h2;                  // Both: 4 bytes
    assign regAWBURST                             = 2'h1;                  // Both: INCR
    assign regAWVALID                             = bridge_m1_axi_awvalid | axi_master_aw_valid[0];
    assign bridge_m1_axi_awready                  = regAWREADY & bridge_m1_axi_awvalid;
    assign axi_master_aw_ready[0]                 = regAWREADY & !bridge_m1_axi_awvalid;

    // Write Data Channel - route based on who made the request
    // Note: wvalid doesn't need sync since it follows awvalid which is already synced
    assign regWDATA                               = bridge_m1_axi_wvalid ? bridge_m1_axi_wdata : axi_master_w_data[`AXI_DATA_WIDTH-1:0];
    assign regWSTRB                               = bridge_m1_axi_wvalid ? bridge_m1_axi_wstrb : axi_master_w_strb[`AXI_DATA_WIDTH/8-1:0];
    assign regWLAST                               = bridge_m1_axi_wvalid ? 1'b1 : axi_master_w_last[0];
    assign regWVALID                              = bridge_m1_axi_wvalid | axi_master_w_valid[0];
    assign bridge_m1_axi_wready                   = regWREADY & bridge_m1_axi_wvalid;
    assign axi_master_w_ready[0]                  = regWREADY & !bridge_m1_axi_wvalid;

    // Write Response Channel - route based on who made the request
    assign bridge_m1_axi_bvalid                   = regBVALID & wr_from_usb;
    assign bridge_m1_axi_bresp                    = regBRESP;
    assign axi_master_b_valid[0]                  = regBVALID & !wr_from_usb;
    assign axi_master_b_id[`AXI_ID_WIDTH-1:0]     = regBID;
    assign axi_master_b_resp[1:0]                 = regBRESP;
    assign regBREADY                              = wr_from_usb ? bridge_m1_axi_bready : axi_master_b_ready[0];

// ================================================================
// ==================== AXI-Lite Slave (Control Registers) =======
// ================================================================

axi_lite_slave axilite_inst
(
    .axi_aclk(regACLK),
	.axi_resetn(ddr_pll_lock),  // Use DDR PLL lock as reset (active when PLL locked)

	.axi_awaddr ({17'h0, bridge_m0_axi_awaddr}),
	.axi_awready(bridge_m0_axi_awready),
	.axi_awvalid(bridge_m0_axi_awvalid),

    .axi_wready (bridge_m0_axi_wready),
	.axi_wdata  (bridge_m0_axi_wdata),
    .axi_wvalid (bridge_m0_axi_wvalid),
    .axi_wlast  (1'b1),
	.axi_wstrb  (bridge_m0_axi_wstrb),

	.axi_bid    (  ),        //not use
	.axi_bresp  (bridge_m0_axi_bresp),
	.axi_bvalid (bridge_m0_axi_bvalid),
	.axi_bready (bridge_m0_axi_bready),

    .axi_araddr ({17'h0, bridge_m0_axi_araddr}),
	.axi_arvalid(bridge_m0_axi_arvalid),
	.axi_arready(bridge_m0_axi_arready),

	.axi_rid    (  ),        //not use
	.axi_rresp  (bridge_m0_axi_rresp),
    .axi_rready (bridge_m0_axi_rready),
    .axi_rdata  (bridge_m0_axi_rdata),
    .axi_rvalid (bridge_m0_axi_rvalid),
    .axi_rlast  (  ),

    .memtest_start  (memtest_start),
	.memtest_rstn   (memtest_rstn),
	.memtest_fail   (fail_0),
	.memtest_done   (done_0),
    .memtest_data   (w_memtest_data),
    .memtest_lfsr_en(w_memtest_lfsr_en),
    .memtest_x16_en (x16_en),
    .phy_rstn       (),              // Not used - phy_rstn driven by hardware
    .ctrl_rstn      (),              // Not used - ctrl_rstn driven by hardware
    .reg_axi_rstn   (),              // Not used - regARESETn driven by hardware
    .axi0_rstn      (),              // Not used - axi0_ARESETn driven by hardware
    .axi1_rstn      (),              // Not used - axi1_ARESETn driven by hardware
    .reg_axi_arlen  (reg_axi_arlen),
    .memtest_size   (w_memtest_size),
	.dq_fail        (dq_fail),
    .config_rst     (),              // Not used - cfg_reset driven by hardware
    .config_sel     (),              // Not used - cfg_sel driven by hardware
    .config_start   (),              // Not used - cfg_start driven by hardware
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
    .tx_clk                ( regACLK            ),
    .tx_tready             ( usb_tx_tready      ),
    .tx_tvalid             ( usb_tx_tvalid      ),
    .tx_tdata              ( usb_tx_tdata       ),
    .tx_tkeep              ( usb_tx_tkeep       ),
    .tx_tlast              ( usb_tx_tlast       ),
    .rx_clk                ( regACLK            ),
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

// AXI-Lite signals from USB command handler to axi_lite_slave
wire [14:0] usb_axi_awaddr;
wire        usb_axi_awvalid;
wire        usb_axi_awready;

wire [31:0] usb_axi_wdata;
wire [3:0]  usb_axi_wstrb;
wire        usb_axi_wvalid;
wire        usb_axi_wready;

wire [1:0]  usb_axi_bresp;
wire        usb_axi_bvalid;
wire        usb_axi_bready;

wire [14:0] usb_axi_araddr;
wire        usb_axi_arvalid;
wire        usb_axi_arready;

wire [31:0] usb_axi_rdata;
wire [1:0]  usb_axi_rresp;
wire        usb_axi_rvalid;
wire        usb_axi_rready;

// USB Command handler (receives CMD, executes commands)
// Command 0x01: TX_MASS - sends back specified number of bytes
// Command 0x02: REG_WRITE - writes to AXI-Lite register
// Command 0x03: REG_READ - reads from AXI-Lite register
usb_command_handler u_usb_command_handler (
    .rstn                  ( 1'b1               ),
    .clk                   ( regACLK            ),

    // USB RX/TX streams
    .i_tready              ( usb_rx_tready      ),
    .i_tvalid              ( usb_rx_tvalid      ),
    .i_tdata               ( usb_rx_tdata       ),
    .o_tready              ( usb_tx_tready      ),
    .o_tvalid              ( usb_tx_tvalid      ),
    .o_tdata               ( usb_tx_tdata       ),
    .o_tkeep               ( usb_tx_tkeep       ),
    .o_tlast               ( usb_tx_tlast       ),

    // AXI-Lite Master (for register access)
    .axi_awaddr            ( usb_axi_awaddr     ),
    .axi_awvalid           ( usb_axi_awvalid    ),
    .axi_awready           ( usb_axi_awready    ),

    .axi_wdata             ( usb_axi_wdata      ),
    .axi_wstrb             ( usb_axi_wstrb      ),
    .axi_wvalid            ( usb_axi_wvalid     ),
    .axi_wready            ( usb_axi_wready     ),

    .axi_bresp             ( usb_axi_bresp      ),
    .axi_bvalid            ( usb_axi_bvalid     ),
    .axi_bready            ( usb_axi_bready     ),

    .axi_araddr            ( usb_axi_araddr     ),
    .axi_arvalid           ( usb_axi_arvalid    ),
    .axi_arready           ( usb_axi_arready    ),

    .axi_rdata             ( usb_axi_rdata      ),
    .axi_rresp             ( usb_axi_rresp      ),
    .axi_rvalid            ( usb_axi_rvalid     ),
    .axi_rready            ( usb_axi_rready     ),

    // Debug status
    .ddr_pll_lock          ( ddr_pll_lock       )
);

// ================================================================
// ============= USB to Register Bridge ===========================
// ================================================================
// Routes USB AXI requests to either:
// - axi_lite_slave (addr < 0x80) for control registers
// - DDR controller (addr >= 0x80) for CTL/PI/PHY registers

// AXI signals from bridge to axi_lite_slave
wire [14:0] bridge_m0_axi_awaddr;
wire        bridge_m0_axi_awvalid;
wire        bridge_m0_axi_awready;
wire [31:0] bridge_m0_axi_wdata;
wire [3:0]  bridge_m0_axi_wstrb;
wire        bridge_m0_axi_wvalid;
wire        bridge_m0_axi_wready;
wire [1:0]  bridge_m0_axi_bresp;
wire        bridge_m0_axi_bvalid;
wire        bridge_m0_axi_bready;
wire [14:0] bridge_m0_axi_araddr;
wire        bridge_m0_axi_arvalid;
wire        bridge_m0_axi_arready;
wire [31:0] bridge_m0_axi_rdata;
wire [1:0]  bridge_m0_axi_rresp;
wire        bridge_m0_axi_rvalid;
wire        bridge_m0_axi_rready;

// AXI signals from bridge to DDR controller
wire [14:0] bridge_m1_axi_awaddr;
wire        bridge_m1_axi_awvalid;
wire        bridge_m1_axi_awready;
wire [31:0] bridge_m1_axi_wdata;
wire [3:0]  bridge_m1_axi_wstrb;
wire        bridge_m1_axi_wvalid;
wire        bridge_m1_axi_wready;
wire [1:0]  bridge_m1_axi_bresp;
wire        bridge_m1_axi_bvalid;
wire        bridge_m1_axi_bready;
wire [14:0] bridge_m1_axi_araddr;
wire        bridge_m1_axi_arvalid;
wire        bridge_m1_axi_arready;
wire [31:0] bridge_m1_axi_rdata;
wire [1:0]  bridge_m1_axi_rresp;
wire        bridge_m1_axi_rvalid;
wire        bridge_m1_axi_rready;

usb2reg_bridge usb_bridge_inst (
    .clk                (regACLK),  // Use regACLK
    .rstn               (ddr_pll_lock),

    // Slave (from USB command handler)
    .s_axi_awaddr       (usb_axi_awaddr),
    .s_axi_awvalid      (usb_axi_awvalid),
    .s_axi_awready      (usb_axi_awready),
    .s_axi_wdata        (usb_axi_wdata),
    .s_axi_wstrb        (usb_axi_wstrb),
    .s_axi_wvalid       (usb_axi_wvalid),
    .s_axi_wready       (usb_axi_wready),
    .s_axi_bresp        (usb_axi_bresp),
    .s_axi_bvalid       (usb_axi_bvalid),
    .s_axi_bready       (usb_axi_bready),
    .s_axi_araddr       (usb_axi_araddr),
    .s_axi_arvalid      (usb_axi_arvalid),
    .s_axi_arready      (usb_axi_arready),
    .s_axi_rdata        (usb_axi_rdata),
    .s_axi_rresp        (usb_axi_rresp),
    .s_axi_rvalid       (usb_axi_rvalid),
    .s_axi_rready       (usb_axi_rready),

    // Master 0 (to axi_lite_slave)
    .m0_axi_awaddr      (bridge_m0_axi_awaddr),
    .m0_axi_awvalid     (bridge_m0_axi_awvalid),
    .m0_axi_awready     (bridge_m0_axi_awready),
    .m0_axi_wdata       (bridge_m0_axi_wdata),
    .m0_axi_wstrb       (bridge_m0_axi_wstrb),
    .m0_axi_wvalid      (bridge_m0_axi_wvalid),
    .m0_axi_wready      (bridge_m0_axi_wready),
    .m0_axi_bresp       (bridge_m0_axi_bresp),
    .m0_axi_bvalid      (bridge_m0_axi_bvalid),
    .m0_axi_bready      (bridge_m0_axi_bready),
    .m0_axi_araddr      (bridge_m0_axi_araddr),
    .m0_axi_arvalid     (bridge_m0_axi_arvalid),
    .m0_axi_arready     (bridge_m0_axi_arready),
    .m0_axi_rdata       (bridge_m0_axi_rdata),
    .m0_axi_rresp       (bridge_m0_axi_rresp),
    .m0_axi_rvalid      (bridge_m0_axi_rvalid),
    .m0_axi_rready      (bridge_m0_axi_rready),

    // Master 1 (to DDR controller)
    .m1_axi_awaddr      (bridge_m1_axi_awaddr),
    .m1_axi_awvalid     (bridge_m1_axi_awvalid),
    .m1_axi_awready     (bridge_m1_axi_awready),
    .m1_axi_wdata       (bridge_m1_axi_wdata),
    .m1_axi_wstrb       (bridge_m1_axi_wstrb),
    .m1_axi_wvalid      (bridge_m1_axi_wvalid),
    .m1_axi_wready      (bridge_m1_axi_wready),
    .m1_axi_bresp       (bridge_m1_axi_bresp),
    .m1_axi_bvalid      (bridge_m1_axi_bvalid),
    .m1_axi_bready      (bridge_m1_axi_bready),
    .m1_axi_araddr      (bridge_m1_axi_araddr),
    .m1_axi_arvalid     (bridge_m1_axi_arvalid),
    .m1_axi_arready     (bridge_m1_axi_arready),
    .m1_axi_rdata       (bridge_m1_axi_rdata),
    .m1_axi_rresp       (bridge_m1_axi_rresp),
    .m1_axi_rvalid      (bridge_m1_axi_rvalid),
    .m1_axi_rready      (bridge_m1_axi_rready)
);

// ================================================================
// ============= DDR Keep-Alive (prevents optimization) ===========
// ================================================================
// Minimal DDR exerciser to prevent synthesis from optimizing away
// the DDR controller. Performs periodic read/write to address 0.


// ================================================================
// ==================== LED Indicators ============================
// ================================================================
// LED[1:0] - Show low 2 bits of last received USB data
// LED[2]   - Heartbeat from regACLK (blinks at 5Hz)
// LED[3]   - Heartbeat from ftdi_clk (blinks at 5Hz)

reg [1:0] usb_tdata_d = 2'h0;

always @ (posedge regACLK)
    if (usb_rx_tvalid)
        usb_tdata_d <= usb_rx_tdata[1:0];

wire regACLK_beat;
wire ftdi_clk_beat;

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_regACLK_beat (
    .clk                   ( regACLK            ),
    .beat                  ( regACLK_beat       )
);

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_ftdi_clk_beat (
    .clk                   ( ftdi_clk           ),
    .beat                  ( ftdi_clk_beat      )
);

assign LED[1:0] = usb_tdata_d;
assign LED[2]   = regACLK_beat;
assign LED[3]   = ftdi_clk_beat;

endmodule

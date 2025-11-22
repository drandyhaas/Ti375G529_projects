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

// ============================================================================
// axi_ctrl_ver3 (tester0) REMOVED
// ============================================================================
// The axi_ctrl_ver3 module was removed because it depended on the axi1 interface
// which we removed. We only use memory_checker_lfsr (checker0) for DDR testing.

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

// ================================================================
// ========== DDR Controller Register Interface (Unused) ==========
// ================================================================
// Tie off DDR controller register interface since we don't access it anymore
// (hardware auto-init handles DDR initialization)
// NOTE: We still need to acknowledge any transactions by keeping READY signals high!
assign regARADDR   = 15'h0;
assign regARID     = 6'h0;
assign regARLEN    = 8'h0;
assign regARSIZE   = 3'h0;
assign regARBURST  = 2'h0;
assign regARVALID  = 1'b0;
assign regRREADY   = 1'b1;  // Always ready to accept read data

assign regAWADDR   = 15'h0;
assign regAWID     = 6'h0;
assign regAWLEN    = 8'h0;
assign regAWSIZE   = 3'h0;
assign regAWBURST  = 2'h0;
assign regAWVALID  = 1'b0;

assign regWDATA    = 32'h0;
assign regWSTRB    = 4'h0;
assign regWLAST    = 1'b0;
assign regWVALID   = 1'b0;

assign regBREADY   = 1'b1;  // Always ready to accept write response

// ================================================================
// ==================== AXI-Lite Slave (Control Registers) =======
// ================================================================
// Connect USB command handler directly to axi_lite_slave
// No need for usb2reg_bridge since we only access control registers (< 0x80)

axi_lite_slave axilite_inst
(
    .axi_aclk(regACLK),
	.axi_resetn(ddr_pll_lock),  // Use DDR PLL lock as reset (active when PLL locked)

	.axi_awaddr ({17'h0, usb_axi_awaddr}),
	.axi_awready(usb_axi_awready),
	.axi_awvalid(usb_axi_awvalid),

    .axi_wready (usb_axi_wready),
	.axi_wdata  (usb_axi_wdata),
    .axi_wvalid (usb_axi_wvalid),
    .axi_wlast  (1'b1),
	.axi_wstrb  (usb_axi_wstrb),

	.axi_bid    (  ),        //not use
	.axi_bresp  (usb_axi_bresp),
	.axi_bvalid (usb_axi_bvalid),
	.axi_bready (usb_axi_bready),

    .axi_araddr ({17'h0, usb_axi_araddr}),
	.axi_arvalid(usb_axi_arvalid),
	.axi_arready(usb_axi_arready),

	.axi_rid    (  ),        //not use
	.axi_rresp  (usb_axi_rresp),
    .axi_rready (usb_axi_rready),
    .axi_rdata  (usb_axi_rdata),
    .axi_rvalid (usb_axi_rvalid),
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
    .tester_loop_len(64'h0),         // axi_ctrl_ver3 removed - tie off
    .tester_loop_cnt(64'h0),         // axi_ctrl_ver3 removed - tie off
    .tester_loop_done(1'b0),         // axi_ctrl_ver3 removed - tie off
    .tester_error(1'b0),             // axi_ctrl_ver3 removed - tie off
    .tester_rst(),                   // axi_ctrl_ver3 removed - not connected
    .tester_pattern()                // axi_ctrl_ver3 removed - not connected
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
// ============= DDR Keep-Alive (prevents optimization) ===========
// ================================================================
// Prevent synthesis from optimizing away DDR and memory checker
// by using their outputs in LED assignments

// XOR all DDR read data bits and memory test status
wire ddr_activity = ^axi0_RDATA ^ done_0 ^ fail_0;

// ================================================================
// ==================== LED Indicators ============================
// ================================================================
// LED[0]   - DDR activity (XOR of read data + test status)
// LED[1]   - USB RX activity
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

assign LED[0]   = ddr_activity;      // DDR + memory test activity
assign LED[1]   = usb_tdata_d[0];    // USB RX data bit 0
assign LED[2]   = regACLK_beat;      // regACLK heartbeat
assign LED[3]   = ftdi_clk_beat;     // ftdi_clk heartbeat

endmodule

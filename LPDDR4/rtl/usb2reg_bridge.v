//--------------------------------------------------------------------------------------------------------
// Module  : usb2reg_bridge
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: USB AXI-Lite bridge with address decoding
//           Routes addresses < 0x80 to axi_lite_slave (control registers)
//           Routes addresses >= 0x80 to DDR controller registers (CTL/PI/PHY)
//--------------------------------------------------------------------------------------------------------

module usb2reg_bridge (
    input  wire        clk,
    input  wire        rstn,

    // AXI-Lite Slave (from USB command handler)
    input  wire [14:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,

    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,

    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [14:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,

    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // AXI-Lite Master 0 (to axi_lite_slave - control registers)
    output wire [14:0] m0_axi_awaddr,
    output wire        m0_axi_awvalid,
    input  wire        m0_axi_awready,

    output wire [31:0] m0_axi_wdata,
    output wire [3:0]  m0_axi_wstrb,
    output wire        m0_axi_wvalid,
    input  wire        m0_axi_wready,

    input  wire [1:0]  m0_axi_bresp,
    input  wire        m0_axi_bvalid,
    output wire        m0_axi_bready,

    output wire [14:0] m0_axi_araddr,
    output wire        m0_axi_arvalid,
    input  wire        m0_axi_arready,

    input  wire [31:0] m0_axi_rdata,
    input  wire [1:0]  m0_axi_rresp,
    input  wire        m0_axi_rvalid,
    output wire        m0_axi_rready,

    // AXI-Lite Master 1 (to DDR controller registers)
    output wire [14:0] m1_axi_awaddr,
    output wire        m1_axi_awvalid,
    input  wire        m1_axi_awready,

    output wire [31:0] m1_axi_wdata,
    output wire [3:0]  m1_axi_wstrb,
    output wire        m1_axi_wvalid,
    input  wire        m1_axi_wready,

    input  wire [1:0]  m1_axi_bresp,
    input  wire        m1_axi_bvalid,
    output wire        m1_axi_bready,

    output wire [14:0] m1_axi_araddr,
    output wire        m1_axi_arvalid,
    input  wire        m1_axi_arready,

    input  wire [31:0] m1_axi_rdata,
    input  wire [1:0]  m1_axi_rresp,
    input  wire        m1_axi_rvalid,
    output wire        m1_axi_rready
);

// Address decoding
// Addresses 0x0000-0x007F (0-127, 32 registers * 4 bytes) -> axi_lite_slave
// Addresses 0x0080-0x7FFF (128+)                          -> DDR controller
localparam ADDR_THRESHOLD = 15'h0080;

// Write address channel routing
wire wr_sel = (s_axi_awaddr >= ADDR_THRESHOLD);  // 0=m0, 1=m1
reg  wr_sel_latched;

// Latch write routing decision at address phase
// Keep latched value until next write - don't auto-clear
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        wr_sel_latched <= 1'b0;
    end else if (s_axi_awvalid && s_axi_awready) begin
        // Latch routing when write address handshake completes
        // This overwrites previous value
        wr_sel_latched <= wr_sel;
    end
end

assign m0_axi_awaddr  = s_axi_awaddr;
assign m0_axi_awvalid = s_axi_awvalid && !wr_sel;
assign m1_axi_awaddr  = s_axi_awaddr - ADDR_THRESHOLD;  // Subtract 0x80 offset for DDR controller
assign m1_axi_awvalid = s_axi_awvalid && wr_sel;

always @(*) begin
    if (wr_sel)
        s_axi_awready = m1_axi_awready;
    else
        s_axi_awready = m0_axi_awready;
end

// Write data channel routing
assign m0_axi_wdata  = s_axi_wdata;
assign m0_axi_wstrb  = s_axi_wstrb;
assign m0_axi_wvalid = s_axi_wvalid && !wr_sel_latched;
assign m1_axi_wdata  = s_axi_wdata;
assign m1_axi_wstrb  = s_axi_wstrb;
assign m1_axi_wvalid = s_axi_wvalid && wr_sel_latched;

always @(*) begin
    if (wr_sel_latched)
        s_axi_wready = m1_axi_wready;
    else
        s_axi_wready = m0_axi_wready;
end

// Write response channel routing
assign m0_axi_bready = s_axi_bready && !wr_sel_latched;
assign m1_axi_bready = s_axi_bready && wr_sel_latched;

always @(*) begin
    if (wr_sel_latched) begin
        s_axi_bresp  = m1_axi_bresp;
        s_axi_bvalid = m1_axi_bvalid;
    end else begin
        s_axi_bresp  = m0_axi_bresp;
        s_axi_bvalid = m0_axi_bvalid;
    end
end

// Read address channel routing
wire rd_sel = (s_axi_araddr >= ADDR_THRESHOLD);  // 0=m0, 1=m1
reg  rd_sel_latched;

// Latch read routing decision at address phase
// Keep latched value until next read - don't auto-clear
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        rd_sel_latched <= 1'b0;
    end else if (s_axi_arvalid && s_axi_arready) begin
        // Latch routing when read address handshake completes
        // This overwrites previous value
        rd_sel_latched <= rd_sel;
    end
end

assign m0_axi_araddr  = s_axi_araddr;
assign m0_axi_arvalid = s_axi_arvalid && !rd_sel;
assign m1_axi_araddr  = s_axi_araddr - ADDR_THRESHOLD;  // Subtract 0x80 offset for DDR controller
assign m1_axi_arvalid = s_axi_arvalid && rd_sel;

always @(*) begin
    if (rd_sel)
        s_axi_arready = m1_axi_arready;
    else
        s_axi_arready = m0_axi_arready;
end

// Read data channel routing
assign m0_axi_rready = s_axi_rready && !rd_sel_latched;
assign m1_axi_rready = s_axi_rready && rd_sel_latched;

always @(*) begin
    if (rd_sel_latched) begin
        s_axi_rdata  = m1_axi_rdata;
        s_axi_rresp  = m1_axi_rresp;
        s_axi_rvalid = m1_axi_rvalid;
    end else begin
        s_axi_rdata  = m0_axi_rdata;
        s_axi_rresp  = m0_axi_rresp;
        s_axi_rvalid = m0_axi_rvalid;
    end
end

endmodule

//--------------------------------------------------------------------------------------------------------
// Module  : axi_lite_cdc
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: AXI-Lite Clock Domain Crossing
//           Crosses AXI-Lite transactions from source clock (s_clk) to destination clock (m_clk)
//           Uses handshake protocol with 2-FF synchronizers for safe CDC
//           Supports single-beat transactions only (no bursts)
//--------------------------------------------------------------------------------------------------------

module axi_lite_cdc (
    // Source clock domain (from command_processor via usb2reg_bridge)
    input  wire        s_clk,
    input  wire        s_rstn,

    // Destination clock domain (DDR register interface)
    input  wire        m_clk,
    input  wire        m_rstn,

    // AXI-Lite Slave interface (source clock domain)
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

    // AXI-Lite Master interface (destination clock domain)
    output reg  [14:0] m_axi_awaddr,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,

    output reg  [31:0] m_axi_wdata,
    output reg  [3:0]  m_axi_wstrb,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,

    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,

    output reg  [14:0] m_axi_araddr,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,

    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready
);

// ============================================================================
// Write Path CDC
// ============================================================================

// Source domain write state machine
localparam S_WR_IDLE = 2'd0;
localparam S_WR_WAIT_ACK = 2'd1;
localparam S_WR_RESP = 2'd2;

reg [1:0] s_wr_state;
reg s_wr_req;           // Request signal (source domain)
reg [14:0] s_wr_addr;   // Latched address
reg [31:0] s_wr_data;   // Latched data
reg [3:0]  s_wr_strb;   // Latched strobe
reg [1:0]  s_wr_resp_latched;

// Destination domain write state machine
localparam M_WR_IDLE = 2'd0;
localparam M_WR_ADDR = 2'd1;
localparam M_WR_DATA = 2'd2;
localparam M_WR_RESP = 2'd3;

reg [1:0] m_wr_state;
reg m_wr_ack;           // Acknowledge signal (destination domain)
reg [1:0] m_wr_resp;    // Response from destination

// Synchronizers for write handshake
reg s_wr_req_sync1, s_wr_req_sync2;  // s_wr_req synchronized to m_clk
reg m_wr_ack_sync1, m_wr_ack_sync2;  // m_wr_ack synchronized to s_clk

// Synchronize s_wr_req to m_clk domain
always @(posedge m_clk or negedge m_rstn) begin
    if (!m_rstn) begin
        s_wr_req_sync1 <= 1'b0;
        s_wr_req_sync2 <= 1'b0;
    end else begin
        s_wr_req_sync1 <= s_wr_req;
        s_wr_req_sync2 <= s_wr_req_sync1;
    end
end

// Synchronize m_wr_ack to s_clk domain
always @(posedge s_clk or negedge s_rstn) begin
    if (!s_rstn) begin
        m_wr_ack_sync1 <= 1'b0;
        m_wr_ack_sync2 <= 1'b0;
    end else begin
        m_wr_ack_sync1 <= m_wr_ack;
        m_wr_ack_sync2 <= m_wr_ack_sync1;
    end
end

// Source domain write state machine
always @(posedge s_clk or negedge s_rstn) begin
    if (!s_rstn) begin
        s_wr_state <= S_WR_IDLE;
        s_wr_req <= 1'b0;
        s_wr_addr <= 15'd0;
        s_wr_data <= 32'd0;
        s_wr_strb <= 4'd0;
        s_wr_resp_latched <= 2'd0;
        s_axi_awready <= 1'b0;
        s_axi_wready <= 1'b0;
        s_axi_bvalid <= 1'b0;
        s_axi_bresp <= 2'd0;
    end else begin
        case (s_wr_state)
        S_WR_IDLE: begin
            s_axi_bvalid <= 1'b0;
            // Accept write address and data together
            if (s_axi_awvalid && s_axi_wvalid && !s_wr_req) begin
                s_wr_addr <= s_axi_awaddr;
                s_wr_data <= s_axi_wdata;
                s_wr_strb <= s_axi_wstrb;
                s_axi_awready <= 1'b1;
                s_axi_wready <= 1'b1;
                s_wr_req <= 1'b1;  // Assert request
                s_wr_state <= S_WR_WAIT_ACK;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready <= 1'b0;
            end
        end

        S_WR_WAIT_ACK: begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            // Wait for acknowledge from destination domain
            if (m_wr_ack_sync2) begin
                s_wr_req <= 1'b0;  // Deassert request
                s_wr_state <= S_WR_RESP;
            end
        end

        S_WR_RESP: begin
            // Wait for ack to deassert, then send response
            if (!m_wr_ack_sync2) begin
                s_axi_bresp <= m_wr_resp;  // Safe to sample - stable after handshake
                s_axi_bvalid <= 1'b1;
                if (s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                    s_wr_state <= S_WR_IDLE;
                end
            end
        end

        default: s_wr_state <= S_WR_IDLE;
        endcase
    end
end

// Destination domain write state machine
always @(posedge m_clk or negedge m_rstn) begin
    if (!m_rstn) begin
        m_wr_state <= M_WR_IDLE;
        m_wr_ack <= 1'b0;
        m_wr_resp <= 2'd0;
        m_axi_awaddr <= 15'd0;
        m_axi_awvalid <= 1'b0;
        m_axi_wdata <= 32'd0;
        m_axi_wstrb <= 4'd0;
        m_axi_wvalid <= 1'b0;
        m_axi_bready <= 1'b0;
    end else begin
        case (m_wr_state)
        M_WR_IDLE: begin
            m_axi_bready <= 1'b0;
            // Wait for request from source domain
            if (s_wr_req_sync2 && !m_wr_ack) begin
                // Sample data (stable because source is waiting)
                m_axi_awaddr <= s_wr_addr;
                m_axi_wdata <= s_wr_data;
                m_axi_wstrb <= s_wr_strb;
                m_axi_awvalid <= 1'b1;
                m_axi_wvalid <= 1'b1;
                m_wr_state <= M_WR_ADDR;
            end
        end

        M_WR_ADDR: begin
            // Wait for address handshake
            if (m_axi_awready) begin
                m_axi_awvalid <= 1'b0;
            end
            if (m_axi_wready) begin
                m_axi_wvalid <= 1'b0;
            end
            if (!m_axi_awvalid && !m_axi_wvalid) begin
                m_axi_bready <= 1'b1;
                m_wr_state <= M_WR_RESP;
            end
        end

        M_WR_RESP: begin
            // Wait for write response
            if (m_axi_bvalid) begin
                m_wr_resp <= m_axi_bresp;
                m_axi_bready <= 1'b0;
                m_wr_ack <= 1'b1;  // Signal completion
                m_wr_state <= M_WR_IDLE;
            end
        end

        default: m_wr_state <= M_WR_IDLE;
        endcase

        // Deassert ack when request goes low
        if (!s_wr_req_sync2 && m_wr_ack) begin
            m_wr_ack <= 1'b0;
        end
    end
end

// ============================================================================
// Read Path CDC
// ============================================================================

// Source domain read state machine
localparam S_RD_IDLE = 2'd0;
localparam S_RD_WAIT_ACK = 2'd1;
localparam S_RD_RESP = 2'd2;

reg [1:0] s_rd_state;
reg s_rd_req;           // Request signal (source domain)
reg [14:0] s_rd_addr;   // Latched address

// Destination domain read state machine
localparam M_RD_IDLE = 2'd0;
localparam M_RD_ADDR = 2'd1;
localparam M_RD_DATA = 2'd2;

reg [1:0] m_rd_state;
reg m_rd_ack;           // Acknowledge signal (destination domain)
reg [31:0] m_rd_data;   // Read data from destination
reg [1:0] m_rd_resp;    // Response from destination

// Synchronizers for read handshake
reg s_rd_req_sync1, s_rd_req_sync2;  // s_rd_req synchronized to m_clk
reg m_rd_ack_sync1, m_rd_ack_sync2;  // m_rd_ack synchronized to s_clk

// Synchronize s_rd_req to m_clk domain
always @(posedge m_clk or negedge m_rstn) begin
    if (!m_rstn) begin
        s_rd_req_sync1 <= 1'b0;
        s_rd_req_sync2 <= 1'b0;
    end else begin
        s_rd_req_sync1 <= s_rd_req;
        s_rd_req_sync2 <= s_rd_req_sync1;
    end
end

// Synchronize m_rd_ack to s_clk domain
always @(posedge s_clk or negedge s_rstn) begin
    if (!s_rstn) begin
        m_rd_ack_sync1 <= 1'b0;
        m_rd_ack_sync2 <= 1'b0;
    end else begin
        m_rd_ack_sync1 <= m_rd_ack;
        m_rd_ack_sync2 <= m_rd_ack_sync1;
    end
end

// Source domain read state machine
always @(posedge s_clk or negedge s_rstn) begin
    if (!s_rstn) begin
        s_rd_state <= S_RD_IDLE;
        s_rd_req <= 1'b0;
        s_rd_addr <= 15'd0;
        s_axi_arready <= 1'b0;
        s_axi_rvalid <= 1'b0;
        s_axi_rdata <= 32'd0;
        s_axi_rresp <= 2'd0;
    end else begin
        case (s_rd_state)
        S_RD_IDLE: begin
            s_axi_rvalid <= 1'b0;
            // Accept read address
            if (s_axi_arvalid && !s_rd_req) begin
                s_rd_addr <= s_axi_araddr;
                s_axi_arready <= 1'b1;
                s_rd_req <= 1'b1;  // Assert request
                s_rd_state <= S_RD_WAIT_ACK;
            end else begin
                s_axi_arready <= 1'b0;
            end
        end

        S_RD_WAIT_ACK: begin
            s_axi_arready <= 1'b0;
            // Wait for acknowledge from destination domain
            if (m_rd_ack_sync2) begin
                s_rd_req <= 1'b0;  // Deassert request
                s_rd_state <= S_RD_RESP;
            end
        end

        S_RD_RESP: begin
            // Wait for ack to deassert, then send response
            if (!m_rd_ack_sync2) begin
                s_axi_rdata <= m_rd_data;  // Safe to sample - stable after handshake
                s_axi_rresp <= m_rd_resp;
                s_axi_rvalid <= 1'b1;
                if (s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                    s_rd_state <= S_RD_IDLE;
                end
            end
        end

        default: s_rd_state <= S_RD_IDLE;
        endcase
    end
end

// Destination domain read state machine
always @(posedge m_clk or negedge m_rstn) begin
    if (!m_rstn) begin
        m_rd_state <= M_RD_IDLE;
        m_rd_ack <= 1'b0;
        m_rd_data <= 32'd0;
        m_rd_resp <= 2'd0;
        m_axi_araddr <= 15'd0;
        m_axi_arvalid <= 1'b0;
        m_axi_rready <= 1'b0;
    end else begin
        case (m_rd_state)
        M_RD_IDLE: begin
            m_axi_rready <= 1'b0;
            // Wait for request from source domain
            if (s_rd_req_sync2 && !m_rd_ack) begin
                // Sample address (stable because source is waiting)
                m_axi_araddr <= s_rd_addr;
                m_axi_arvalid <= 1'b1;
                m_rd_state <= M_RD_ADDR;
            end
        end

        M_RD_ADDR: begin
            // Wait for address handshake
            if (m_axi_arready) begin
                m_axi_arvalid <= 1'b0;
                m_axi_rready <= 1'b1;
                m_rd_state <= M_RD_DATA;
            end
        end

        M_RD_DATA: begin
            // Wait for read data
            if (m_axi_rvalid) begin
                m_rd_data <= m_axi_rdata;
                m_rd_resp <= m_axi_rresp;
                m_axi_rready <= 1'b0;
                m_rd_ack <= 1'b1;  // Signal completion
                m_rd_state <= M_RD_IDLE;
            end
        end

        default: m_rd_state <= M_RD_IDLE;
        endcase

        // Deassert ack when request goes low
        if (!s_rd_req_sync2 && m_rd_ack) begin
            m_rd_ack <= 1'b0;
        end
    end
end

endmodule


//--------------------------------------------------------------------------------------------------------
// Module  : usb_command_handler
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: USB command handler with command byte and AXI-Lite master for register access
//           Protocol: [CMD][DATA...]
//
//           Supported commands:
//           0x01 - TX_MASS: [LENGTH(4B,LE)] - Send back specified number of bytes
//           0x02 - REG_WRITE: [ADDR(4B,LE)][DATA(4B,LE)] - Write to AXI-Lite register
//           0x03 - REG_READ: [ADDR(4B,LE)] - Read from AXI-Lite register, returns [DATA(4B,LE)]
//--------------------------------------------------------------------------------------------------------

module usb_command_handler (
    input  wire        rstn,
    input  wire        clk,

    // AXI-stream slave (from USB RX)
    output wire        i_tready,
    input  wire        i_tvalid,
    input  wire [ 7:0] i_tdata,

    // AXI-stream master (to USB TX)
    input  wire        o_tready,
    output reg         o_tvalid,
    output reg  [31:0] o_tdata,
    output reg  [ 3:0] o_tkeep,
    output reg         o_tlast,

    // AXI-Lite Master interface (for register access)
    output reg  [14:0] axi_awaddr,
    output reg         axi_awvalid,
    input  wire        axi_awready,

    output reg  [31:0] axi_wdata,
    output reg  [3:0]  axi_wstrb,
    output reg         axi_wvalid,
    input  wire        axi_wready,

    input  wire [1:0]  axi_bresp,
    input  wire        axi_bvalid,
    output reg         axi_bready,

    output reg  [14:0] axi_araddr,
    output reg         axi_arvalid,
    input  wire        axi_arready,

    input  wire [31:0] axi_rdata,
    input  wire [1:0]  axi_rresp,
    input  wire        axi_rvalid,
    output reg         axi_rready,

    // Debug status inputs
    input  wire        ddr_pll_lock
);


localparam [4:0] RX_CMD      = 5'd0,
                 // TX_MASS states
                 RX_LEN0     = 5'd1,
                 RX_LEN1     = 5'd2,
                 RX_LEN2     = 5'd3,
                 RX_LEN3     = 5'd4,
                 TX_DATA     = 5'd5,
                 // REG_WRITE states
                 RX_ADDR0    = 5'd6,
                 RX_ADDR1    = 5'd7,
                 RX_ADDR2    = 5'd8,
                 RX_ADDR3    = 5'd9,
                 RX_DATA0    = 5'd10,
                 RX_DATA1    = 5'd11,
                 RX_DATA2    = 5'd12,
                 RX_DATA3    = 5'd13,
                 AXI_WRITE   = 5'd14,
                 AXI_WRESP   = 5'd15,
                 // REG_READ states
                 AXI_READ    = 5'd16,
                 AXI_RRESP   = 5'd17,
                 TX_RDATA    = 5'd18,
                 // GET_VERSION state
                 LOAD_VERSION= 5'd19,
                 ERROR       = 5'd31;

// Command codes
localparam [7:0] CMD_TX_MASS   = 8'h01;
localparam [7:0] CMD_REG_WRITE = 8'h02;
localparam [7:0] CMD_REG_READ  = 8'h03;
localparam [7:0] CMD_GET_VERSION = 8'h04;  // Returns 4-byte version: 0x20250120 (2025-01-20)
localparam [7:0] CMD_GET_STATUS = 8'h05;  // Returns 4-byte status with debug info

reg [4:0]  state = RX_CMD;
reg [7:0]  command = 8'h00;
reg [31:0] length = 0;
reg [31:0] reg_addr = 0;
reg [31:0] reg_data = 0;

// Write response handling removed - now using proper AXI protocol in state machine

// Watchdog timeout counter - reset state machine if stuck for too long
// At 100MHz, 28 bits = ~2.68 seconds max
reg [27:0] timeout_counter = 0;
localparam [27:0] TIMEOUT_MAX = 28'd200_000_000;  // 2 seconds at 100MHz


always @ (posedge clk or negedge rstn)
    if (~rstn) begin
        state      <= RX_CMD;
        command    <= 8'h00;
        length     <= 0;
        reg_addr   <= 0;
        reg_data   <= 0;
        o_tvalid   <= 1'b0;
        o_tdata    <= 32'h0;
        o_tkeep    <= 4'h0;
        o_tlast    <= 1'b0;
        axi_awaddr <= 15'h0;
        axi_awvalid<= 1'b0;
        axi_wdata  <= 32'h0;
        axi_wstrb  <= 4'hF;
        axi_wvalid <= 1'b0;
        axi_bready <= 1'b0;
        axi_araddr <= 15'h0;
        axi_arvalid<= 1'b0;
        axi_rready <= 1'b0;
        timeout_counter <= 0;
    end else begin
        // Watchdog timeout - if stuck in non-idle state for too long, reset to RX_CMD
        if (state != RX_CMD) begin
            if (timeout_counter >= TIMEOUT_MAX) begin
                // Timeout! Force reset to RX_CMD
                state <= RX_CMD;
                o_tvalid <= 1'b0;
                axi_awvalid <= 1'b0;
                axi_wvalid <= 1'b0;
                axi_bready <= 1'b0;  // Clear bready on timeout
                axi_arvalid <= 1'b0;
                axi_rready <= 1'b0;
                timeout_counter <= 0;
            end else begin
                timeout_counter <= timeout_counter + 1;
            end
        end else begin
            timeout_counter <= 0;
        end
        case (state)
            // ============================================
            // Receive command byte
            // ============================================
            RX_CMD : begin
                // Make sure all TX signals are clean when waiting for new command
                o_tvalid <= 1'b0;
                o_tlast  <= 1'b0;
                o_tkeep  <= 4'h0;

                if (i_tvalid) begin
                    command <= i_tdata;
                    // Dispatch to appropriate state based on command
                    case (i_tdata)
                        CMD_TX_MASS:     state <= RX_LEN0;
                        CMD_REG_WRITE:   state <= RX_ADDR0;
                        CMD_REG_READ:    state <= RX_ADDR0;
                        CMD_GET_VERSION: state <= LOAD_VERSION;  // Load version then transmit
                        CMD_GET_STATUS:  state <= LOAD_VERSION;  // Reuse LOAD_VERSION state for status too
                        default:         state <= ERROR;
                    endcase
                end
            end

            LOAD_VERSION : begin
                // Clear tvalid and wait one cycle before starting TX
                // This ensures clean transition into TX_DATA state
                o_tvalid <= 1'b0;
                o_tlast  <= 1'b0;
                length <= 4;  // Send 4 bytes
                if (command == CMD_GET_VERSION) begin
                    reg_data <= 32'hDEADBEEF;  // Version: DEADBEEF (test if bitstream loaded)
                end else if (command == CMD_GET_STATUS) begin
                    // Status: bit[0]=ddr_pll_lock, bit[1]=axi_arready, bit[2]=axi_rvalid, bit[3:31]=reserved
                    reg_data <= {29'h0, axi_rvalid, axi_arready, ddr_pll_lock};
                end else begin
                    reg_data <= 32'hDEADBEEF;  // Should never happen
                end
                state <= TX_DATA;
            end

            // ============================================
            // CMD_TX_MASS: Receive length, send data
            // ============================================
            RX_LEN0 : if (i_tvalid) begin
                length[7:0] <= i_tdata;
                state <= RX_LEN1;
            end

            RX_LEN1 : if (i_tvalid) begin
                length[15:8] <= i_tdata;
                state <= RX_LEN2;
            end

            RX_LEN2 : if (i_tvalid) begin
                length[23:16] <= i_tdata;
                state <= RX_LEN3;
            end

            RX_LEN3 : if (i_tvalid) begin
                length[31:24] <= i_tdata;
                state <= TX_DATA;
            end

            TX_DATA : begin
                o_tvalid <= 1'b1;
                // Use reg_data if command returns fixed data, otherwise use counting pattern
                if ((command == CMD_GET_VERSION || command == CMD_REG_READ || command == CMD_GET_STATUS) && length == 4) begin
                    o_tdata  <= reg_data;
                end else begin
                    o_tdata  <= {length[7:0] - 8'd4,
                                 length[7:0] - 8'd3,
                                 length[7:0] - 8'd2,
                                 length[7:0] - 8'd1};
                end
                o_tkeep  <= (length>=4) ? 4'b1111 :
                            (length==3) ? 4'b0111 :
                            (length==2) ? 4'b0011 :
                            (length==1) ? 4'b0001 :
                                          4'b0000;
                o_tlast  <= (length>4) ? 1'b0 : 1'b1;  // Fixed: > not >= so 4-byte sends tlast=1

                if (o_tready) begin
                    if (length >= 4) begin
                        length <= length - 4;
                    end else begin
                        length <= 0;
                        o_tvalid <= 1'b0;
                        state <= RX_CMD;
                    end
                end
            end

            // ============================================
            // CMD_REG_WRITE / CMD_REG_READ: Receive address
            // ============================================
            RX_ADDR0 : if (i_tvalid) begin
                reg_addr[7:0] <= i_tdata;
                state <= RX_ADDR1;
            end

            RX_ADDR1 : if (i_tvalid) begin
                reg_addr[15:8] <= i_tdata;
                state <= RX_ADDR2;
            end

            RX_ADDR2 : if (i_tvalid) begin
                reg_addr[23:16] <= i_tdata;
                state <= RX_ADDR3;
            end

            RX_ADDR3 : if (i_tvalid) begin
                reg_addr[31:24] <= i_tdata;
                // Dispatch based on command
                if (command == CMD_REG_WRITE)
                    state <= RX_DATA0;
                else if (command == CMD_REG_READ)
                    state <= AXI_READ;
                else
                    state <= ERROR;
            end

            // ============================================
            // CMD_REG_WRITE: Receive data and write to AXI
            // ============================================
            RX_DATA0 : if (i_tvalid) begin
                reg_data[7:0] <= i_tdata;
                state <= RX_DATA1;
            end

            RX_DATA1 : if (i_tvalid) begin
                reg_data[15:8] <= i_tdata;
                state <= RX_DATA2;
            end

            RX_DATA2 : if (i_tvalid) begin
                reg_data[23:16] <= i_tdata;
                state <= RX_DATA3;
            end

            RX_DATA3 : if (i_tvalid) begin
                reg_data[31:24] <= i_tdata;
                state <= AXI_WRITE;
            end

            AXI_WRITE : begin
                // Assert write address and data, wait for handshake
                axi_awaddr  <= reg_addr[14:0];  // 15-bit address
                axi_awvalid <= 1'b1;
                axi_wdata   <= reg_data;
                axi_wstrb   <= 4'hF;  // All bytes valid
                axi_wvalid  <= 1'b1;
                // Wait for both awready and wready before transitioning
                if (axi_awready && axi_wready) begin
                    state <= AXI_WRESP;
                end
            end

            AXI_WRESP : begin
                // Wait for write response with proper AXI protocol
                axi_awvalid <= 1'b0;
                axi_wvalid  <= 1'b0;
                axi_bready  <= 1'b1;  // Ready to accept write response

                if (axi_bvalid) begin
                    axi_bready <= 1'b0;
                    state <= RX_CMD;  // Return to command state after receiving response
                end
            end

            // ============================================
            // CMD_REG_READ: Read from AXI and send data
            // ============================================
            AXI_READ : begin
                // Real AXI read path - assert arvalid and wait for arready
                axi_araddr  <= reg_addr[14:0];
                axi_arvalid <= 1'b1;
                axi_rready  <= 1'b1;
                // Wait for arready before transitioning
                if (axi_arready) begin
                    state <= AXI_RRESP;
                end
            end

            AXI_RRESP : begin
                // Clear arvalid after one cycle
                axi_arvalid <= 1'b0;
                axi_rready <= 1'b1;
                if (axi_rvalid) begin
                    reg_data <= axi_rdata;
                    axi_rready <= 1'b0;
                    length <= 4;
                    state <= TX_DATA;
                end
            end

            TX_RDATA : begin
                // Set outputs unconditionally (like TX_DATA does)
                o_tvalid <= 1'b1;
                o_tdata  <= reg_data;
                o_tkeep  <= 4'b1111;
                o_tlast  <= 1'b1;

                // Only transition when handshake completes
                if (o_tready) begin
                    o_tvalid <= 1'b0;
                    state <= RX_CMD;
                end
            end

            // ============================================
            // ERROR: Invalid command
            // ============================================
            ERROR : begin
                state <= RX_CMD;
            end

            default : state <= RX_CMD;
        endcase
    end


// Only accept new RX data when in RX states or RX_CMD
assign i_tready = (state == RX_CMD) || (state == RX_LEN0) || (state == RX_LEN1) ||
                  (state == RX_LEN2) || (state == RX_LEN3) ||
                  (state == RX_ADDR0) || (state == RX_ADDR1) ||
                  (state == RX_ADDR2) || (state == RX_ADDR3) ||
                  (state == RX_DATA0) || (state == RX_DATA1) ||
                  (state == RX_DATA2) || (state == RX_DATA3);


endmodule

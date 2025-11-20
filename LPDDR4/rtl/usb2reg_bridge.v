// USB to Register Bridge
// Converts USB AXI-Stream (8-bit) to AXI-Lite register transactions
// Protocol: [CMD][ADDR(4B,LE)][DATA(4B,LE)] over 8-bit AXI-Stream
// CMD: 0x01 = read, 0x02 = write

module usb2reg_bridge (
    input wire clk,
    input wire rst_n,

    // USB RX interface (AXI-Stream slave, 8-bit)
    input wire [7:0] rx_tdata,
    input wire       rx_tvalid,
    output reg       rx_tready,

    // USB TX interface (AXI-Stream master, 8-bit)
    output reg [7:0] tx_tdata,
    output reg       tx_tvalid,
    input wire       tx_tready,
    output reg       tx_tlast,  // Indicates last byte of response

    // AXI-Lite Master interface (to register decoder)
    output reg [14:0] axi_awaddr,
    output reg        axi_awvalid,
    input wire        axi_awready,

    output reg [31:0] axi_wdata,
    output reg [3:0]  axi_wstrb,
    output reg        axi_wvalid,
    input wire        axi_wready,

    input wire [1:0]  axi_bresp,
    input wire        axi_bvalid,
    output reg        axi_bready,

    output reg [14:0] axi_araddr,
    output reg        axi_arvalid,
    input wire        axi_arready,

    input wire [31:0] axi_rdata,
    input wire [1:0]  axi_rresp,
    input wire        axi_rvalid,
    output reg        axi_rready
);

// State machine
localparam IDLE       = 4'd0;
localparam RX_CMD     = 4'd1;
localparam RX_ADDR0   = 4'd2;
localparam RX_ADDR1   = 4'd3;
localparam RX_ADDR2   = 4'd4;
localparam RX_ADDR3   = 4'd5;
localparam RX_DATA0   = 4'd6;
localparam RX_DATA1   = 4'd7;
localparam RX_DATA2   = 4'd8;
localparam RX_DATA3   = 4'd9;
localparam DO_WRITE   = 4'd10;
localparam DO_READ    = 4'd11;
localparam TX_DATA0   = 4'd12;
localparam TX_DATA1   = 4'd13;
localparam TX_DATA2   = 4'd14;
localparam TX_DATA3   = 4'd15;

reg [3:0] state;
reg [7:0] cmd;
reg [31:0] addr;
reg [31:0] data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        rx_tready <= 1'b0;
        tx_tdata <= 8'h0;
        tx_tvalid <= 1'b0;
        tx_tlast <= 1'b0;
        axi_awaddr <= 15'h0;
        axi_awvalid <= 1'b0;
        axi_wdata <= 32'h0;
        axi_wstrb <= 4'h0;
        axi_wvalid <= 1'b0;
        axi_bready <= 1'b0;
        axi_araddr <= 15'h0;
        axi_arvalid <= 1'b0;
        axi_rready <= 1'b0;
        cmd <= 8'h0;
        addr <= 32'h0;
        data <= 32'h0;
    end else begin
        case (state)
            IDLE: begin
                rx_tready <= 1'b1;
                tx_tvalid <= 1'b0;
                axi_awvalid <= 1'b0;
                axi_wvalid <= 1'b0;
                axi_arvalid <= 1'b0;
                axi_bready <= 1'b0;
                axi_rready <= 1'b0;
                if (rx_tvalid && rx_tready) begin
                    cmd <= rx_tdata;
                    state <= RX_ADDR0;
                end
            end

            RX_ADDR0: begin
                if (rx_tvalid && rx_tready) begin
                    addr[7:0] <= rx_tdata;
                    state <= RX_ADDR1;
                end
            end

            RX_ADDR1: begin
                if (rx_tvalid && rx_tready) begin
                    addr[15:8] <= rx_tdata;
                    state <= RX_ADDR2;
                end
            end

            RX_ADDR2: begin
                if (rx_tvalid && rx_tready) begin
                    addr[23:16] <= rx_tdata;
                    state <= RX_ADDR3;
                end
            end

            RX_ADDR3: begin
                if (rx_tvalid && rx_tready) begin
                    addr[31:24] <= rx_tdata;
                    if (cmd == 8'h02) begin  // Write command
                        state <= RX_DATA0;
                    end else begin  // Read command (0x01)
                        state <= DO_READ;
                        rx_tready <= 1'b0;
                    end
                end
            end

            RX_DATA0: begin
                if (rx_tvalid && rx_tready) begin
                    data[7:0] <= rx_tdata;
                    state <= RX_DATA1;
                end
            end

            RX_DATA1: begin
                if (rx_tvalid && rx_tready) begin
                    data[15:8] <= rx_tdata;
                    state <= RX_DATA2;
                end
            end

            RX_DATA2: begin
                if (rx_tvalid && rx_tready) begin
                    data[23:16] <= rx_tdata;
                    state <= RX_DATA3;
                end
            end

            RX_DATA3: begin
                if (rx_tvalid && rx_tready) begin
                    data[31:24] <= rx_tdata;
                    state <= DO_WRITE;
                    rx_tready <= 1'b0;
                end
            end

            DO_WRITE: begin
                axi_awaddr <= addr[14:0];
                axi_awvalid <= 1'b1;
                axi_wdata <= data;
                axi_wstrb <= 4'hF;
                axi_wvalid <= 1'b1;
                axi_bready <= 1'b1;

                if (axi_awvalid && axi_awready) begin
                    axi_awvalid <= 1'b0;
                end

                if (axi_wvalid && axi_wready) begin
                    axi_wvalid <= 1'b0;
                end

                if (axi_bvalid && axi_bready) begin
                    axi_bready <= 1'b0;
                    state <= IDLE;
                end
            end

            DO_READ: begin
                axi_araddr <= addr[14:0];
                axi_arvalid <= 1'b1;
                axi_rready <= 1'b1;

                if (axi_arvalid && axi_arready) begin
                    axi_arvalid <= 1'b0;
                end

                if (axi_rvalid && axi_rready) begin
                    axi_rready <= 1'b0;
                    data <= axi_rdata;
                    state <= TX_DATA0;
                end
            end

            TX_DATA0: begin
                tx_tdata <= data[7:0];
                tx_tvalid <= 1'b1;
                tx_tlast <= 1'b0;
                if (tx_tvalid && tx_tready) begin
                    state <= TX_DATA1;
                end
            end

            TX_DATA1: begin
                tx_tdata <= data[15:8];
                tx_tvalid <= 1'b1;
                tx_tlast <= 1'b0;
                if (tx_tvalid && tx_tready) begin
                    state <= TX_DATA2;
                end
            end

            TX_DATA2: begin
                tx_tdata <= data[23:16];
                tx_tvalid <= 1'b1;
                tx_tlast <= 1'b0;
                if (tx_tvalid && tx_tready) begin
                    state <= TX_DATA3;
                end
            end

            TX_DATA3: begin
                tx_tdata <= data[31:24];
                tx_tvalid <= 1'b1;
                tx_tlast <= 1'b1;  // This is the last byte!
                if (tx_tvalid && tx_tready) begin
                    tx_tvalid <= 1'b0;
                    tx_tlast <= 1'b0;
                    state <= IDLE;
                end
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule

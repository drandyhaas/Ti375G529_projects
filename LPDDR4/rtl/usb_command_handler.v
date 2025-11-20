
//--------------------------------------------------------------------------------------------------------
// Module  : usb_command_handler
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: USB command handler with command byte
//           Protocol: [CMD][LENGTH(4B,LE)][optional data...]
//
//           Supported commands:
//           0x01 - TX_MASS: Receive length, send back that many bytes (usb_rx_mass.py test)
//           0x02-0xFF - Reserved for future use
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
    output reg         o_tlast
);


localparam [3:0] RX_CMD   = 4'd0,
                 RX_LEN0  = 4'd1,
                 RX_LEN1  = 4'd2,
                 RX_LEN2  = 4'd3,
                 RX_LEN3  = 4'd4,
                 TX_DATA  = 4'd5,
                 ERROR    = 4'd15;

// Command codes
localparam [7:0] CMD_TX_MASS = 8'h01;

reg [3:0]  state = RX_CMD;
reg [7:0]  command = 8'h00;
reg [31:0] length = 0;


always @ (posedge clk or negedge rstn)
    if (~rstn) begin
        state   <= RX_CMD;
        command <= 8'h00;
        length  <= 0;
        o_tvalid <= 1'b0;
        o_tdata  <= 32'h0;
        o_tkeep  <= 4'h0;
        o_tlast  <= 1'b0;
    end else begin
        case (state)
            // Receive command byte
            RX_CMD : if (i_tvalid) begin
                command <= i_tdata;
                state <= RX_LEN0;
            end

            // Receive length bytes (little-endian)
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
                // Dispatch based on command
                case (command)
                    CMD_TX_MASS: state <= TX_DATA;
                    default:     state <= ERROR;  // Unknown command
                endcase
            end

            // TX_DATA state: Send data back (for TX_MASS command)
            TX_DATA : begin
                o_tvalid <= 1'b1;
                o_tdata  <= {length[7:0] - 8'd4,
                             length[7:0] - 8'd3,
                             length[7:0] - 8'd2,
                             length[7:0] - 8'd1};
                o_tkeep  <= (length>=4) ? 4'b1111 :
                            (length==3) ? 4'b0111 :
                            (length==2) ? 4'b0011 :
                            (length==1) ? 4'b0001 :
                                          4'b0000;
                o_tlast  <= (length>=4) ? 1'b0 : 1'b1;

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

            // ERROR state: Invalid command received, return to idle
            ERROR : begin
                state <= RX_CMD;
            end

            default : state <= RX_CMD;
        endcase
    end


assign i_tready = (state != TX_DATA);


endmodule
